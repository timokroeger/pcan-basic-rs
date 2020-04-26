use pcan::{prelude::*, Error, Id, Interface};

fn main() -> Result<(), Error> {
    let mut can = Interface::init()?;
    //can.add_filter(Id::new(0x02))?;
    can.add_filter_with_mask(Id::new_extended(0x152), Id::new(0x0F0))?;

    // PCAN only supports one filter
    assert!(can.add_filter(Id::new(1 << 5)).is_err());
    //can.clear_filters();

    loop {
        let msg = can.receive_blocking()?;
        println!("ID: 0x{:03X}", msg.id());
    }
}
