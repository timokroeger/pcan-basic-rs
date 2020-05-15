pub mod prelude {
    pub use embedded_hal::can::{
        Filter as _, FilteredReceiver as _, Frame as _, Receiver as _, Transmitter as _,
    };
}

use std::{
    ffi::{c_void, CString},
    fmt,
    mem::{self, MaybeUninit},
    ptr,
};

use embedded_hal::can::{self, Receiver as _};
use pcan_basic_sys::*;
use prelude::*;
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
    pcan_channel: u16,
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

        Ok(Self {
            pcan_channel,
            event_handle,
        })
    }

    pub fn split(&self) -> (Rx, Tx) {
        // By default do not receive messages.
        let mut rx = Rx(self);
        rx.clear_filters();

        // Drain all messages that were received since `init()` has been called.
        loop {
            if let Err(nb::Error::WouldBlock) = rx.receive() {
                break;
            }
        }

        (rx, Tx(self))
    }
}

impl Drop for Interface {
    fn drop(&mut self) {
        unsafe { CAN_Uninitialize(self.pcan_channel) };
    }
}

#[derive(Debug)]
pub struct Frame(TPCANMsg);

impl can::Frame for Frame {
    fn new_standard(id: u32, data: &[u8]) -> Self {
        assert!(data.len() <= 8);

        let mut msg = TPCANMsg {
            ID: id,
            MSGTYPE: PCAN_MESSAGE_STANDARD as u8,
            LEN: data.len() as u8,
            DATA: [0; 8],
        };
        msg.DATA[0..data.len()].copy_from_slice(data);
        Self(msg)
    }

    fn new_extended(id: u32, data: &[u8]) -> Self {
        assert!(data.len() <= 8);

        let mut msg = TPCANMsg {
            ID: id,
            MSGTYPE: PCAN_MESSAGE_EXTENDED as u8,
            LEN: data.len() as u8,
            DATA: [0; 8],
        };
        msg.DATA[0..data.len()].copy_from_slice(data);
        Self(msg)
    }

    fn set_rtr(&mut self, rtr: bool) -> &mut Self {
        if rtr {
            self.0.MSGTYPE |= PCAN_MESSAGE_RTR as u8;
        } else {
            self.0.MSGTYPE &= !(PCAN_MESSAGE_RTR as u8);
        }
        self
    }

    fn is_extended(&self) -> bool {
        self.0.MSGTYPE & PCAN_MESSAGE_EXTENDED as u8 != 0
    }

    fn is_remote_frame(&self) -> bool {
        self.0.MSGTYPE & PCAN_MESSAGE_RTR as u8 != 0
    }

    fn id(&self) -> u32 {
        self.0.ID
    }

    fn data(&self) -> &[u8] {
        &self.0.DATA[0..self.0.LEN as usize]
    }
}

pub struct Tx<'a>(&'a Interface);

impl<'a> can::Transmitter for Tx<'a> {
    type Frame = Frame;
    type Error = Error;

    fn transmit(&mut self, frame: &Self::Frame) -> nb::Result<Option<Self::Frame>, Self::Error> {
        let result = unsafe { CAN_Write(self.0.pcan_channel, &frame.0 as *const _ as *mut _) };
        if result == PCAN_ERROR_OK {
            Ok(None)
        } else {
            Err(nb::Error::Other(Error::new(result)))
        }
    }
}

pub struct Rx<'a>(&'a Interface);

impl<'a> Rx<'a> {
    pub fn receive_blocking(&mut self) -> Result<Frame, Error> {
        loop {
            match self.receive() {
                Ok(frame) => break Ok(frame),
                Err(nb::Error::Other(e)) => break Err(e),
                Err(nb::Error::WouldBlock) => unsafe {
                    synchapi::WaitForSingleObject(self.0.event_handle, INFINITE);
                },
            }
        }
    }
}

impl<'a> can::Receiver for Rx<'a> {
    type Frame = Frame;
    type Error = Error;

    fn receive(&mut self) -> nb::Result<Self::Frame, Self::Error> {
        let mut msg = MaybeUninit::<TPCANMsg>::uninit();
        let (result, msg) = unsafe {
            (
                CAN_Read(self.0.pcan_channel, msg.as_mut_ptr(), ptr::null_mut()),
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

pub struct Filter {
    accept_all: bool,
    is_extended: bool,
    id: u32,
    mask: u32,
}

impl can::Filter for Filter {
    fn accept_all() -> Self {
        // TODO: Fix
        Self {
            accept_all: true,
            is_extended: true,
            id: 0,
            mask: 0,
        }
    }

    fn new_standard(id: u32) -> Self {
        Self {
            accept_all: false,
            is_extended: false,
            id,
            mask: 0x7FF,
        }
    }

    fn new_extended(id: u32) -> Self {
        Self {
            accept_all: false,
            is_extended: true,
            id,
            mask: 0x1FFF_FFFF,
        }
    }

    fn set_mask(&mut self, mask: u32) -> &mut Self {
        self.mask = mask;
        self
    }
}

impl<'a> can::FilteredReceiver for Rx<'a> {
    type Filter = Filter;

    const NUM_FILTERS: usize = 1;
    const NUM_MASKS: usize = 1;

    fn add_filter(&mut self, filter: &Self::Filter) -> Result<(), Self::Error> {
        let mut filter_state = 0u32;
        unsafe {
            CAN_GetValue(
                self.0.pcan_channel,
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
                    self.0.pcan_channel,
                    PCAN_MESSAGE_FILTER as u8,
                    &mut filter_open as *mut _ as *mut c_void,
                    mem::size_of_val(&filter_open) as u32,
                );
            };
        } else {
            let mut value = [filter.mask.to_le(), filter.id.to_le()];
            unsafe {
                CAN_SetValue(
                    self.0.pcan_channel,
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

    fn clear_filters(&mut self) {
        let mut filter_open = PCAN_FILTER_CLOSE;
        unsafe {
            CAN_SetValue(
                self.0.pcan_channel,
                PCAN_MESSAGE_FILTER as u8,
                &mut filter_open as *mut _ as *mut c_void,
                mem::size_of_val(&filter_open) as u32,
            );
        };
    }
}
