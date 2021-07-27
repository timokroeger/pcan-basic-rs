#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]

mod bindings;

pub use bindings::*;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn init_fail() {
        assert_eq!(
            unsafe { CAN_Initialize(0xFFFF, 0, 0, 0, 0) },
            PCAN_ERROR_ILLHANDLE
        );
    }
}
