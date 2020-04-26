use std::{env, error::Error, fmt, fs::File, io};

use anyhow::{anyhow, Result};
use nb::block;
use pcan::{
    self,
    can::{Frame, Receiver, Transmitter},
};

const BOOTLOADER_BLOCK_LEN: usize = 256;

struct Bootloader<I>(I);

impl<I> Bootloader<I>
where
    I: Receiver + Transmitter,
    <I as Receiver>::Error: Error + Send + Sync + 'static,
    <I as Transmitter>::Error: Error + Send + Sync + 'static,
    <I as Receiver>::Frame: fmt::Debug,
    <I as Transmitter>::Frame: fmt::Debug,
{
    pub fn new(interface: I) -> Self {
        Self(interface)
    }

    // The bootloader listens on multiple communication inerfaces.
    // Send a synchronization message so it locks on the CAN interface.
    pub fn enable(&mut self) -> Result<()> {
        self.send(0x79, &[])?;
        self.receive_ack(0x79)
    }

    pub fn erase(&mut self) -> Result<()> {
        self.send(0x43, &[0xFF])?;
        self.receive_ack(0x43)?;
        self.receive_ack(0x43)
    }

    pub fn write(&mut self, addr: u32, data: &mut impl io::Read) -> Result<()> {
        for i in (0..).step_by(BOOTLOADER_BLOCK_LEN) {
            // One write operation supports up to 256 bytes.
            // Loop until all bytes of the file are written.
            let mut buf = [0; BOOTLOADER_BLOCK_LEN];
            let num_bytes = data.read(&mut buf)?;
            if num_bytes == 0 {
                break;
            }

            let addr = addr + i;
            let msg_start: [u8; 5] = [
                (addr >> 24) as u8,
                (addr >> 16) as u8,
                (addr >> 8) as u8,
                addr as u8,
                (num_bytes - 1) as u8,
            ];
            self.send(0x31, &msg_start)?;
            self.receive_ack(0x31)?;

            for msg_data in buf[..num_bytes].chunks(8) {
                self.send(0x04, msg_data)?;
                self.receive_ack(0x31)?;
            }

            self.receive_ack(0x31)?;
        }

        Ok(())
    }

    pub fn go(&mut self, addr: u32) -> Result<()> {
        let addr = addr.to_be_bytes();
        self.send(0x21, &addr)?;
        self.receive_ack(0x21)
    }

    pub fn send(&mut self, id: u32, data: &[u8]) -> Result<()> {
        block!(self
            .0
            .transmit(&<I as Transmitter>::Frame::new_standard(id, data)))?;
        Ok(())
    }

    fn receive_ack(&mut self, id: u32) -> Result<()> {
        let msg = block!(self.0.receive())?;
        if msg.id() == id && msg.data() == &[0x79] {
            return Ok(());
        }

        Err(anyhow!("Expected ACK message, got: {0:?}", msg))
    }
}

fn main() -> anyhow::Result<()> {
    let file_name = env::args().nth(1);
    if file_name.is_none() {
        println!("usage: stm32-fwupdate <image.bin>");
        return Ok(());
    }
    let file_name = file_name.unwrap();
    let mut file = File::open(file_name)?;

    let mut bl = Bootloader::new(pcan::Interface::init()?);

    bl.enable()?;
    bl.erase()?;
    bl.write(0x0800_0000, &mut file)?;

    // Overwrite bootloader magic entry constant at the end of SRAM so that the
    // device will not switch to bootloader mode again after the GO command.
    bl.write(0x2000_FFFC, &mut [0u8; 4].as_ref())?;

    // Jump the new firmware in flash.
    bl.go(0x0800_0000)?;

    Ok(())
}
