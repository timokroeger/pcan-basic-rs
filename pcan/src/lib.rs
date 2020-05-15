pub mod prelude {
    pub use embedded_hal::can::{
        Frame as _, Receiver as _, Transmitter as _,
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

    pub fn receive_blocking(&mut self) -> Result<Frame, Error> {
        loop {
            match self.receive() {
                Ok(frame) => break Ok(frame),
                Err(nb::Error::Other(e)) => break Err(e),
                Err(nb::Error::WouldBlock) => unsafe {
                    synchapi::WaitForSingleObject(self.event_handle, INFINITE);
                },
            }
        }
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

impl can::Transmitter for Interface {
    type Frame = Frame;
    type Error = Error;

    fn transmit(&mut self, frame: &Self::Frame) -> nb::Result<Option<Self::Frame>, Self::Error> {
        let result = unsafe { CAN_Write(self.pcan_channel, &frame.0 as *const _ as *mut _) };
        if result == PCAN_ERROR_OK {
            Ok(None)
        } else {
            Err(nb::Error::Other(Error::new(result)))
        }
    }
}

impl can::Receiver for Interface {
    type Frame = Frame;
    type Error = Error;

    fn receive(&mut self) -> nb::Result<Self::Frame, Self::Error> {
        let mut msg = MaybeUninit::<TPCANMsg>::uninit();
        let (result, msg) = unsafe {
            (
                CAN_Read(self.pcan_channel, msg.as_mut_ptr(), ptr::null_mut()),
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

/* TODO
impl can::MessageFilter for Interface {
    type FilterId = Id;

    fn add_filter(&mut self, id: Id) -> Result<(), Self::Error> {
        self.add_filter_with_mask(id, Id { id: 0, eid: id.eid })
    }

    fn add_filter_with_mask(&mut self, id: Id, mask: Id) -> Result<(), Self::Error> {
        let mut filter_state = 0u32;
        unsafe {
            CAN_GetValue(
                self.pcan_channel,
                PCAN_MESSAGE_FILTER as u8,
                &mut filter_state as *mut _ as *mut c_void,
                mem::size_of_val(&filter_state) as u32,
            );
        }
        if filter_state == PCAN_FILTER_CUSTOM {
            return Err(Error("Cannot configure more than one filter".to_string()));
        }

        let mut value = [mask.id.to_le(), id.id.to_le()];
        unsafe {
            CAN_SetValue(
                self.pcan_channel,
                if id.eid {
                    PCAN_ACCEPTANCE_FILTER_29BIT
                } else {
                    PCAN_ACCEPTANCE_FILTER_11BIT
                } as u8,
                &mut value as *mut _ as *mut c_void,
                mem::size_of_val(&value) as u32,
            );
        };

        Ok(())
    }

    fn clear_filters(&mut self) {
        let mut filter_open = PCAN_FILTER_OPEN;
        unsafe {
            CAN_SetValue(
                self.pcan_channel,
                PCAN_MESSAGE_FILTER as u8,
                &mut filter_open as *mut _ as *mut c_void,
                mem::size_of_val(&filter_open) as u32,
            );
        };
    }
}
*/
