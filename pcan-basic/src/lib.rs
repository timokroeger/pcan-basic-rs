pub mod prelude {
    pub use embedded_can::{Can as _, Frame as _};
}

pub use embedded_can::{ExtendedId, Id, StandardId};

use std::{
    ffi::{c_void, CString},
    fmt,
    mem::{self, MaybeUninit},
    ptr,
};

use pcan_basic_sys::*;
use winapi::{
    shared::minwindef::FALSE,
    um::{synchapi, winbase::INFINITE, winnt::HANDLE},
};

#[derive(Debug)]
pub struct Error(String);

impl Error {
    fn new(error_code: u32) -> Self {
        unsafe {
            let raw_error_msg = CString::from_vec_unchecked(Vec::with_capacity(256)).into_raw();
            CAN_GetErrorText(error_code, 0, raw_error_msg);
            Self(String::from_utf8_unchecked(
                CString::from_raw(raw_error_msg).into_bytes(),
            ))
        }
    }
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl std::error::Error for Error {}

pub struct Interface {
    channel: u16,
    event_handle: HANDLE,
}

impl Interface {
    pub fn init() -> Result<Self, Error> {
        let pcan_channel = PCAN_USBBUS1 as u16;

        // When running with 125kbps the STM32 bootloader sets the acknowledge bit early.
        // Choose a nominal sample point of 75% to prevent form errors in the CRC delimiter.
        // Value calculated using http://www.bittiming.can-wiki.info/ (NXP SJA1000)
        const BAUDRATE_CONFIG: u16 = 0x033A;
        let result = unsafe { CAN_Initialize(pcan_channel, BAUDRATE_CONFIG, 0, 0, 0) };
        if result != PCAN_ERROR_OK {
            return Err(Error::new(result));
        }

        let mut parameter_off = PCAN_PARAMETER_OFF;
        unsafe {
            CAN_SetValue(
                pcan_channel,
                PCAN_ALLOW_STATUS_FRAMES as u8,
                &mut parameter_off as *mut _ as *mut c_void,
                mem::size_of_val(&parameter_off) as u32,
            );
        }

        let mut event_handle =
            unsafe { synchapi::CreateEventA(ptr::null_mut(), FALSE, FALSE, ptr::null()) };
        if event_handle.is_null() {
            return Err(Error::new(result));
        }

        unsafe {
            CAN_SetValue(
                pcan_channel,
                PCAN_RECEIVE_EVENT as u8,
                &mut event_handle as *mut _ as *mut c_void,
                mem::size_of_val(&event_handle) as u32,
            );
        };

        let mut this = Self {
            channel: pcan_channel,
            event_handle,
        };

        // Drain all messages that were received since `init()` has been called.
        loop {
            if let Err(nb::Error::WouldBlock) = this.receive() {
                break;
            }
        }

        Ok(this)
    }
}

impl Drop for Interface {
    fn drop(&mut self) {
        unsafe { CAN_Uninitialize(self.channel) };
    }
}

#[derive(Debug)]
pub struct Frame(TPCANMsg);

impl embedded_can::Frame for Frame {
    fn new(id: impl Into<Id>, data: &[u8]) -> Result<Frame, ()> {
        if data.len() > 8 {
            return Err(());
        }

        let (id, msg_type) = match id.into() {
            Id::Standard(id) => (id.as_raw() as u32, PCAN_MESSAGE_STANDARD),
            Id::Extended(id) => (id.as_raw(), PCAN_MESSAGE_EXTENDED),
        };

        let mut msg = TPCANMsg {
            ID: id,
            MSGTYPE: msg_type as u8,
            LEN: data.len() as u8,
            DATA: [0; 8],
        };
        msg.DATA[0..data.len()].copy_from_slice(data);
        Ok(Frame(msg))
    }

    fn new_remote(id: impl Into<Id>, dlc: usize) -> Result<Frame, ()> {
        if dlc >= 8 {
            return Err(());
        }

        let mut frame = Frame::new(id, &[])?;
        frame.0.MSGTYPE |= PCAN_MESSAGE_RTR as u8;
        frame.0.LEN = dlc as u8;
        Ok(frame)
    }

    fn is_extended(&self) -> bool {
        self.0.MSGTYPE & PCAN_MESSAGE_EXTENDED as u8 != 0
    }

    fn is_remote_frame(&self) -> bool {
        self.0.MSGTYPE & PCAN_MESSAGE_RTR as u8 != 0
    }

    fn id(&self) -> Id {
        if self.is_extended() {
            ExtendedId::new(self.0.ID).unwrap().into()
        } else {
            StandardId::new(self.0.ID as u16).unwrap().into()
        }
    }

    fn dlc(&self) -> usize {
        self.0.LEN as usize
    }

    fn data(&self) -> &[u8] {
        &self.0.DATA[0..self.0.LEN as usize]
    }
}

impl Interface {
    fn transmit(&mut self, frame: &Frame) -> nb::Result<Option<Frame>, Error> {
        let result = unsafe { CAN_Write(self.channel, &frame.0 as *const _ as *mut _) };
        if result == PCAN_ERROR_OK {
            Ok(None)
        } else {
            Err(nb::Error::Other(Error::new(result)))
        }
    }

    fn receive(&mut self) -> nb::Result<Frame, Error> {
        let mut msg = MaybeUninit::<TPCANMsg>::uninit();
        let (result, msg) = unsafe {
            (
                CAN_Read(self.channel, msg.as_mut_ptr(), ptr::null_mut()),
                msg.assume_init(),
            )
        };

        match result {
            PCAN_ERROR_QRCVEMPTY => Err(nb::Error::WouldBlock),
            PCAN_ERROR_OK => Ok(Frame(msg)),
            _ => Err(nb::Error::Other(Error::new(result))),
        }
    }
}

impl embedded_can::Can for Interface {
    type Frame = Frame;
    type Error = Error;

    fn try_transmit(&mut self, frame: &Frame) -> nb::Result<Option<Frame>, Error> {
        self.transmit(frame)
    }

    fn try_receive(&mut self) -> nb::Result<Frame, Error> {
        self.receive()
    }
}

impl embedded_can::blocking::Can for Interface {
    type Frame = Frame;
    type Error = Error;

    fn try_write(&mut self, frame: &Frame) -> Result<(), Error> {
        match self.transmit(frame) {
            Ok(_) => Ok(()),
            Err(nb::Error::Other(err)) => Err(err),
            _ => panic!("The PCAN driver should never block!"),
        }
    }

    fn try_read(&mut self) -> Result<Frame, Error> {
        match self.receive() {
            Err(nb::Error::WouldBlock) => {
                unsafe { synchapi::WaitForSingleObject(self.event_handle, INFINITE) };
                match self.receive() {
                    Ok(frame) => Ok(frame),
                    Err(nb::Error::Other(err)) => Err(err),
                    _ => panic!("Receive queue should not be empty!"),
                }
            }
            Ok(frame) => Ok(frame),
            Err(nb::Error::Other(err)) => Err(err),
        }
    }
}

pub struct Filter {
    accept_all: bool,
    is_extended: bool,
    id: u32,
    mask: u32,
}

impl Filter {
    pub fn accept_all() -> Self {
        // TODO: Fix
        Self {
            accept_all: true,
            is_extended: true,
            id: 0,
            mask: 0,
        }
    }

    pub fn new(id: Id) -> Self {
        match id {
            Id::Standard(id) => Self {
                accept_all: false,
                is_extended: false,
                id: id.as_raw() as u32,
                mask: 0x7FF,
            },
            Id::Extended(id) => Self {
                accept_all: false,
                is_extended: true,
                id: id.as_raw(),
                mask: 0x1FFF_FFFF,
            },
        }
    }

    pub fn with_mask(&mut self, mask: u32) -> &mut Self {
        self.mask = mask;
        self
    }
}

impl Interface {
    pub fn add_filter(&mut self, filter: &Filter) -> Result<(), Error> {
        let mut filter_state = 0u32;
        unsafe {
            CAN_GetValue(
                self.channel,
                PCAN_MESSAGE_FILTER as u8,
                &mut filter_state as *mut _ as *mut c_void,
                mem::size_of_val(&filter_state) as u32,
            );
        }
        if filter_state == PCAN_FILTER_CUSTOM {
            return Err(Error("Cannot configure more than one filter".to_string()));
        }

        if filter.accept_all {
            let mut filter_open = PCAN_FILTER_OPEN;
            unsafe {
                CAN_SetValue(
                    self.channel,
                    PCAN_MESSAGE_FILTER as u8,
                    &mut filter_open as *mut _ as *mut c_void,
                    mem::size_of_val(&filter_open) as u32,
                );
            };
        } else {
            let mut value = [filter.mask.to_le(), filter.id.to_le()];
            unsafe {
                CAN_SetValue(
                    self.channel,
                    if filter.is_extended {
                        PCAN_ACCEPTANCE_FILTER_29BIT
                    } else {
                        PCAN_ACCEPTANCE_FILTER_11BIT
                    } as u8,
                    &mut value as *mut _ as *mut c_void,
                    mem::size_of_val(&value) as u32,
                );
            };
        }

        Ok(())
    }

    pub fn clear_filters(&mut self) {
        let mut filter_open = PCAN_FILTER_CLOSE;
        unsafe {
            CAN_SetValue(
                self.channel,
                PCAN_MESSAGE_FILTER as u8,
                &mut filter_open as *mut _ as *mut c_void,
                mem::size_of_val(&filter_open) as u32,
            );
        };
    }
}
