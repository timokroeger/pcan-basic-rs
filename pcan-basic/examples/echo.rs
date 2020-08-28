use nb::block;
use pcan_basic;

struct Driver<Can>(Can);

impl<Can> Driver<Can>
where
    Can: embedded_hal::can::Can,
    Can::Error: core::fmt::Debug,
{
    pub fn echo(&mut self) {
        let frame = block!(self.0.receive()).unwrap();
        self.0.transmit(&frame).unwrap();
    }
}

fn main() -> anyhow::Result<()> {

    let mut can = pcan_basic::Interface::init()?;
    can.set_blocking(true);
    
    let mut driver = Driver(can);
    driver.echo();

    Ok(())
}
