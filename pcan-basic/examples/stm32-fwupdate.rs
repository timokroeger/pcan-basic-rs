use std::{env, error::Error, fmt, fs::File, io};

use anyhow::{anyhow, Result};
use embedded_hal::can::{self, Filter, FilterGroup, Frame};
use nb::block;
use pcan_basic;

const BOOTLOADER_BLOCK_LEN: usize = 256;

struct Bootloader<Can> {
    can: Can,
}

impl<Can> Bootloader<Can>
where
    Can: can::Can + can::FilteredReceiver,
    Can::Frame: fmt::Debug,
    Can::Error: Error + Send + Sync + 'static,
{
    pub fn new(mut can: Can) -> Self {
        let mut num_filters = 0;
        let mut has_mask = false;
        for fc in can.filter_groups() {
            num_filters += fc.num_filters();
            has_mask = has_mask || fc.mask().is_some();
        }

        let rx_ids = [0x79, 0x43, 0x31, 0x21];
        if num_filters >= rx_ids.len() {
            // Add filters in a simple list.
            for &rx_id in &rx_ids {
                can.add_filter(&Can::Filter::new_standard(rx_id)).unwrap();
            }
        } else if has_mask {
            // Combine all IDs into a masked filters.
            let mut id = 0;
            let mut mask = 0;
            for &rx_id in &rx_ids {
                id &= rx_id;
                mask |= rx_id;
            }
            can.add_filter(&Can::Filter::new_standard(id).with_mask(mask))
                .unwrap();
        } else {
            panic!("Not enough CAN filters are available.");
        }
        Self { can }
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
        let tx_frame = Can::Frame::new_standard(id, data).unwrap();
        block!(self.can.transmit(&tx_frame))?;
        Ok(())
    }

    fn receive_ack(&mut self, id: u32) -> Result<()> {
        let msg = block!(self.can.receive())?;
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

    let mut can = pcan_basic::Interface::init()?;
    can.set_blocking(true);

    let mut bl = Bootloader::new(can);

    bl.enable()?;
    bl.erase()?;
    bl.write(0x0800_0000, &mut file)?;

    // Jump the new firmware in flash.
    bl.go(0x0800_0000)?;

    // SCB_VTOR was modified by the bootloader and the vector points to system memory.
    // The flashed firmware must update SCB_VTOR to use interrupts with the correct
    // vector table.

    Ok(())
}
