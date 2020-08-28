pub mod prelude {
    pub use embedded_hal::can::{
        Can as _, Filter as _, FilterGroup as _, FilteredReceiver as _, Frame as _,
    };
}

use std::{
    ffi::{c_void, CString},
    fmt,
    iter::{self, Once},
    mem::{self, MaybeUninit},
    ptr,
};

use embedded_hal::can::{self, Id, MaskType, RtrFilterBehavior};
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
    channel: u16,
    is_blocking: bool,
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
            is_blocking: false,
            event_handle,
        };

        // Do not receive any messages by default.
        this.clear_filters();

        // Drain all messages that were received since `init()` has been called.
        loop {
            if let Err(nb::Error::WouldBlock) = this.receive() {
                break;
            }
        }

        Ok(this)
    }

    pub fn set_blocking(&mut self, is_blocking: bool) {
        self.is_blocking = is_blocking;
    }
}

impl Drop for Interface {
    fn drop(&mut self) {
        unsafe { CAN_Uninitialize(self.channel) };
    }
}

#[derive(Debug)]
pub struct Frame(TPCANMsg);

impl can::Frame for Frame {
    fn new(id: Id, data: &[u8]) -> Result<Frame, ()> {
        if !id.valid() || data.len() > 8 {
            return Err(());
        }

        let (id, msg_type) = match id {
            Id::Standard(id) => (id as u32, PCAN_MESSAGE_STANDARD),
            Id::Extended(id) => (id, PCAN_MESSAGE_EXTENDED),
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

    fn new_remote(id: Id, dlc: usize) -> Result<Frame, ()> {
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
            Id::Extended(self.0.ID)
        } else {
            Id::Standard(self.0.ID)
        }
    }

    fn dlc(&self) -> usize {
        self.0.LEN as usize
    }

    fn data(&self) -> &[u8] {
        &self.0.DATA[0..self.0.LEN as usize]
    }
}

impl can::Can for Interface {
    type Frame = Frame;
    type Error = Error;

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
            PCAN_ERROR_QRCVEMPTY => {
                if self.is_blocking {
                    unsafe { synchapi::WaitForSingleObject(self.event_handle, INFINITE) };
                    self.receive()
                } else {
                    Err(nb::Error::WouldBlock)
                }
            }
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

    fn new(id: Id) -> Self {
        match id {
            Id::Standard(id) => Self {
                accept_all: false,
                is_extended: false,
                id: id as u32,
                mask: 0x7FF,
            },
            Id::Extended(id) => Self {
                accept_all: false,
                is_extended: true,
                id,
                mask: 0x1FFF_FFFF,
            },
        }
    }

    fn with_mask(&mut self, mask: u32) -> &mut Self {
        self.mask = mask;
        self
    }

    /// Not supported as reported in the capabilities.
    fn allow_remote(&mut self) -> &mut Self {
        self
    }

    /// Not supported as reported in the capabilities.
    fn remote_only(&mut self) -> &mut Self {
        self
    }
}

pub struct FilterGroup;

impl can::FilterGroup for FilterGroup {
    fn num_filters(&self) -> usize {
        1
    }

    fn extended(&self) -> bool {
        true
    }

    fn mask(&self) -> Option<MaskType> {
        Some(MaskType::Individual)
    }

    fn rtr(&self) -> RtrFilterBehavior {
        RtrFilterBehavior::RemoteAlwaysAllowed
    }
}

impl can::FilteredReceiver for Interface {
    type Filter = Filter;
    type FilterGroup = FilterGroup;
    type FilterGroups = Once<FilterGroup>;

    fn filter_groups(&self) -> Once<FilterGroup> {
        iter::once(FilterGroup)
    }

    fn add_filter(&mut self, filter: &Filter) -> Result<(), Error> {
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

    fn clear_filters(&mut self) {
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
