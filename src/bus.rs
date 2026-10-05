//! The bus to the nRF70: the [`Bus`] trait the runner talks through, and [`SpiBus`], which
//! implements it on any embedded-hal-async `SpiDevice` with the nRF70's SPI commands (FAST_READ, PP
//! and the status register commands RDSR, RDSR1, RDSR2 and WRSR2: nRF7002 PS v1.2 section 7,
//! table 3).

use embedded_hal::spi::Operation;
use embedded_hal_async::spi::SpiDevice;

use crate::{slice8, slice8_mut};

/// Access to the nRF70's memory and status registers, over SPI or QSPI.
///
/// A transfer that fails turns the chip off: the runner reports [`Error::Bus`](crate::Error::Bus)
/// to what was under way, and [`Control::power_on`](crate::Control::power_on) starts it again.
pub trait Bus {
    /// What a failed transfer reports. The driver logs it.
    type Error: core::fmt::Debug;

    /// Reads `buf.len()` words from `addr`.
    async fn read(&mut self, addr: u32, buf: &mut [u32]) -> Result<(), Self::Error>;
    /// Writes `buf` to `addr`.
    async fn write(&mut self, addr: u32, buf: &[u32]) -> Result<(), Self::Error>;
    /// Reads status register 0 (RDSR, 0x05).
    async fn read_sr0(&mut self) -> Result<u8, Self::Error>;
    /// Reads status register 1 (RDSR1, 0x1F): whether the RPU is awake and ready.
    async fn read_sr1(&mut self) -> Result<u8, Self::Error>;
    /// Reads status register 2 (RDSR2, 0x2F): the wake-up request as the chip holds it.
    async fn read_sr2(&mut self) -> Result<u8, Self::Error>;
    /// Writes status register 2 (WRSR2, 0x3F): bit 0 asks the RPU to wake up and stay awake.
    async fn write_sr2(&mut self, val: u8) -> Result<(), Self::Error>;
}

/// A [`Bus`] on an SPI device.
pub struct SpiBus<T> {
    spi: T,
}

impl<T> SpiBus<T> {
    /// Wraps an SPI device, whose chip select is the nRF70's.
    pub fn new(spi: T) -> Self {
        Self { spi }
    }
}

impl<T: SpiDevice> SpiBus<T> {
    async fn read_sr(&mut self, cmd: u8) -> Result<u8, T::Error> {
        let mut buf = [0; 2];
        self.spi.transfer(&mut buf, &[cmd]).await?;
        defmt::trace!("read sr {:02x} = {:02x}", cmd, buf[1]);
        Ok(buf[1])
    }
}

impl<T: SpiDevice> Bus for SpiBus<T> {
    type Error = T::Error;

    async fn read(&mut self, addr: u32, buf: &mut [u32]) -> Result<(), T::Error> {
        self.spi
            .transaction(&mut [
                Operation::Write(&[0x0B, (addr >> 16) as u8, (addr >> 8) as u8, addr as u8, 0x00]),
                Operation::Read(slice8_mut(buf)),
            ])
            .await
    }

    async fn write(&mut self, addr: u32, buf: &[u32]) -> Result<(), T::Error> {
        self.spi
            .transaction(&mut [
                Operation::Write(&[0x02, (addr >> 16) as u8 | 0x80, (addr >> 8) as u8, addr as u8]),
                Operation::Write(slice8(buf)),
            ])
            .await
    }

    async fn read_sr0(&mut self) -> Result<u8, T::Error> {
        self.read_sr(0x05).await
    }

    async fn read_sr1(&mut self) -> Result<u8, T::Error> {
        self.read_sr(0x1f).await
    }

    async fn read_sr2(&mut self) -> Result<u8, T::Error> {
        self.read_sr(0x2f).await
    }

    async fn write_sr2(&mut self, val: u8) -> Result<(), T::Error> {
        defmt::trace!("write sr2 = {:02x}", val);
        self.spi.write(&[0x3f, val]).await
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use core::assert_eq;
    use std::vec;
    use std::vec::Vec;

    use embassy_futures::block_on;

    use super::*;
    use crate::rpu::{SR1_RPU_AWAKE, SR1_RPU_READY, SR2_RPU_WAKEUP_REQ};

    #[derive(Debug, PartialEq)]
    enum Op {
        Write(Vec<u8>),
        Read(usize),
        Transfer(Vec<u8>, usize),
    }

    /// An SPI device that records what it is asked to do, and answers reads from `read_data`.
    #[derive(Default)]
    struct MockSpi {
        ops: Vec<Op>,
        read_data: Vec<u8>,
    }

    impl MockSpi {
        fn fill(&mut self, buf: &mut [u8]) {
            for b in buf {
                *b = if self.read_data.is_empty() {
                    0
                } else {
                    self.read_data.remove(0)
                };
            }
        }
    }

    impl embedded_hal::spi::ErrorType for MockSpi {
        type Error = core::convert::Infallible;
    }

    impl SpiDevice for MockSpi {
        async fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), Self::Error> {
            for op in operations {
                match op {
                    Operation::Write(buf) => self.ops.push(Op::Write(buf.to_vec())),
                    Operation::Read(buf) => {
                        self.ops.push(Op::Read(buf.len()));
                        self.fill(buf);
                    }
                    Operation::Transfer(read, write) => {
                        self.ops.push(Op::Transfer(write.to_vec(), read.len()));
                        self.fill(read);
                    }
                    Operation::TransferInPlace(buf) => {
                        self.ops.push(Op::Transfer(buf.to_vec(), buf.len()));
                        self.fill(buf);
                    }
                    Operation::DelayNs(_) => {}
                }
            }
            Ok(())
        }
    }

    #[test]
    fn bus_read_is_a_fast_read_with_a_dummy_byte() {
        let mut bus = SpiBus::new(MockSpi {
            read_data: vec![0x44, 0x33, 0x22, 0x11, 0x88, 0x77, 0x66, 0x55],
            ..Default::default()
        });
        let mut buf = [0u32; 2];
        block_on(bus.read(0x0C_1234, &mut buf)).unwrap();
        assert_eq!(buf, [0x1122_3344, 0x5566_7788]);
        assert_eq!(
            bus.spi.ops,
            [Op::Write(vec![0x0B, 0x0C, 0x12, 0x34, 0x00]), Op::Read(8)]
        );
    }

    #[test]
    fn bus_write_sets_the_address_top_bit() {
        let mut bus = SpiBus::new(MockSpi::default());
        block_on(bus.write(0x0C_1234, &[0x1122_3344])).unwrap();
        assert_eq!(
            bus.spi.ops,
            [
                Op::Write(vec![0x02, 0x8C, 0x12, 0x34]),
                Op::Write(vec![0x44, 0x33, 0x22, 0x11]),
            ]
        );
    }

    #[test]
    fn bus_status_registers() {
        let mut bus = SpiBus::new(MockSpi {
            read_data: vec![0xFF, SR1_RPU_AWAKE | SR1_RPU_READY],
            ..Default::default()
        });
        assert_eq!(block_on(bus.read_sr1()), Ok(SR1_RPU_AWAKE | SR1_RPU_READY));
        block_on(bus.write_sr2(SR2_RPU_WAKEUP_REQ)).unwrap();
        assert_eq!(
            bus.spi.ops,
            [Op::Transfer(vec![0x1f], 2), Op::Write(vec![0x3f, SR2_RPU_WAKEUP_REQ])]
        );
    }
}
