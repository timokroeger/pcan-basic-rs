use pcan_basic;

struct Driver<Can>(Can);

impl<Can> Driver<Can>
where
    Can: embedded_hal::blocking::can::Can,
    Can::Error: core::fmt::Debug,
{
    pub fn echo(&mut self) {
        let frame = self.0.try_receive().unwrap();
        self.0.try_transmit(&frame).unwrap();
    }
}

fn main() -> anyhow::Result<()> {
    let can = pcan_basic::Interface::init()?;
    let mut driver = Driver(can);
    driver.echo();

    Ok(())
}
