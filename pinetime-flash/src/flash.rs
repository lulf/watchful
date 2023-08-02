use bitflags::bitflags;
use embedded_hal::spi::{Operation, SpiDevice};
use embedded_storage::nor_flash::{
    check_erase, check_write, ErrorType, NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash,
};

const PAGE_SIZE: usize = 256;
const ERASE_SIZE: usize = 4096;
const FLASH_SIZE: usize = 4 * 1024 * 1024;

#[repr(u8)]
#[allow(unused)]
enum OpCode {
    ReadId = 0x9F,
    WriteEnable = 0x06,
    WriteDisable = 0x04,
    ReadStatus = 0x05,
    Read = 0x03,
    FastRead = 0x0B,
    ProgPage = 0x02,
    EraseSector = 0x20,
    Wakeup = 0xAB,
    PowerDown = 0xB9,
}

bitflags! {
    /// Status register bits.
    #[derive(Debug)]
    pub struct StatusRegister: u8 {
        /// Erase or write in progress.
        const WIP = 1 << 0;
        /// Status of the **W**rite **E**nable **L**atch.
        const WEL = 1 << 1;
        /// The 3 protection region bits.
        const PROT = 0b00011100;
        /// **S**tatus **R**egister **W**rite **D**isable bit.
        const SRWD = 1 << 7;
    }
}

pub struct XtFlash<SPI: SpiDevice> {
    spi: SPI,
}

#[derive(Debug)]
pub enum Error<SPI> {
    Spi(SPI),
    Flash(NorFlashErrorKind),
    Busy,
    InvalidManufacturerId,
    InvalidMemoryType,
    NotInRam,
}

impl<SPI> From<SPI> for Error<SPI> {
    fn from(spi: SPI) -> Self {
        Self::Spi(spi)
    }
}

impl<SPI: SpiDevice> XtFlash<SPI> {
    pub fn new(mut spi: SPI) -> Result<Self, Error<SPI::Error>> {
        let mut value: [u8; 4] = [0xAB, 0x01, 0x02, 0x03];
        spi.transfer_in_place(&mut value[..])?;

        let mut value: [u8; 4] = [OpCode::ReadId as u8, 0, 0, 0];
        spi.transfer_in_place(&mut value[..])?;

        if value[1] != 0x0B {
            return Err(Error::InvalidManufacturerId);
        }

        if value[2] != 0x40 {
            return Err(Error::InvalidMemoryType);
        }

        let mut value: [u8; 1] = [0x98];
        spi.write(&value[..])?;

        let mut value: [u8; 1] = [0x50];
        spi.write(&value[..])?;

        let mut value: [u8; 3] = [0x01, 0x00, 0x00];
        spi.transfer_in_place(&mut value[..])?;

        Ok(Self { spi })
    }

    pub fn erase(&mut self, from: u32, to: u32) -> Result<(), Error<SPI::Error>> {
        check_erase(self, from, to).map_err(Error::Flash)?;
        self.wait_done()?;
        // info!("Erase 0x{:x} - 0x{:x}", from, to);
        for page in (from..to).step_by(ERASE_SIZE) {
            self.write_enable()?;

            let offset = page.to_be_bytes();
            self.spi.transaction(&mut [Operation::TransferInPlace(&mut [
                OpCode::EraseSector as u8,
                offset[1],
                offset[2],
                offset[3],
            ])])?;

            self.wait_done()?;
        }

        Ok(())
    }

    pub fn read_status(&mut self) -> Result<StatusRegister, Error<SPI::Error>> {
        let mut value = [OpCode::ReadStatus as u8, 0x00];
        self.spi
            .transaction(&mut [Operation::TransferInPlace(&mut value[..])])?;
        let status = StatusRegister::from_bits_truncate(value[1]);
        // info!("Read status...: 0x{:02x}", value[1]);
        Ok(status)
    }

    fn write_enable(&mut self) -> Result<(), Error<SPI::Error>> {
        self.spi
            .transaction(&mut [Operation::Write(&mut [OpCode::WriteEnable as u8])])?;

        while !self.read_status()?.contains(StatusRegister::WEL) {}
        Ok(())
    }

    fn wait_done(&mut self) -> Result<(), Error<SPI::Error>> {
        // info!("Waiting ready...");
        loop {
            let status = self.read_status()?;
            if !(status.contains(StatusRegister::WIP)) {
                break;
            }
        }
        Ok(())
    }

    pub fn write(&mut self, mut write_offset: u32, data: &[u8]) -> Result<(), Error<SPI::Error>> {
        check_write(self, write_offset, data.len()).map_err(Error::Flash)?;
        self.wait_done()?;

        //info!("Write {} bytes to {:x}", data.len(), write_offset);
        // TODO: Check for unaligned offset
        for chunk in data.chunks(PAGE_SIZE) {
            self.write_enable()?;

            let offset = write_offset.to_be_bytes();
            info!(
                "Write chunk of len {} to {}, offset[3] {}",
                chunk.len(),
                write_offset,
                offset[3]
            );
            let cmd = [OpCode::ProgPage as u8, offset[1], offset[2], offset[3]];
            self.spi
                .transaction(&mut [Operation::Write(&cmd[..]), Operation::Write(chunk)])?;

            self.wait_done()?;

            write_offset += chunk.len() as u32;
        }

        Ok(())
    }

    pub fn read(&mut self, offset: u32, data: &mut [u8]) -> Result<(), Error<SPI::Error>> {
        // info!("Read {} bytes from {:x}", data.len(), offset);
        self.wait_done()?;
        let offset = offset.to_be_bytes();
        let cmd = [OpCode::Read as u8, offset[1], offset[2], offset[3]];

        self.spi
            .transaction(&mut [Operation::Write(&cmd[..]), Operation::Read(data)])?;

        Ok(())
    }
}

impl<SPI: core::fmt::Debug> NorFlashError for Error<SPI> {
    fn kind(&self) -> NorFlashErrorKind {
        NorFlashErrorKind::Other
    }
}

impl<SPI: SpiDevice> ErrorType for XtFlash<SPI> {
    type Error = Error<SPI::Error>;
}

impl<SPI: SpiDevice> ReadNorFlash for XtFlash<SPI> {
    const READ_SIZE: usize = 1;
    fn read(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), Self::Error> {
        slice_in_ram_or(buf, Error::NotInRam)?;
        XtFlash::read(self, offset, buf)
    }

    fn capacity(&self) -> usize {
        FLASH_SIZE
    }
}

impl<SPI: SpiDevice> NorFlash for XtFlash<SPI> {
    const WRITE_SIZE: usize = 1;
    const ERASE_SIZE: usize = ERASE_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        XtFlash::erase(self, from, to)
    }

    fn write(&mut self, offset: u32, data: &[u8]) -> Result<(), Self::Error> {
        slice_in_ram_or(data, Error::NotInRam)?;
        XtFlash::write(self, offset, data)
    }
}

use core::mem;

const SRAM_LOWER: usize = 0x2000_0000;
const SRAM_UPPER: usize = 0x3000_0000;

// TODO: replace transmutes with core::ptr::metadata once it's stable
pub(crate) fn slice_ptr_parts<T>(slice: *const [T]) -> (*const T, usize) {
    unsafe { mem::transmute(slice) }
}

pub(crate) fn slice_ptr_parts_mut<T>(slice: *mut [T]) -> (*mut T, usize) {
    unsafe { mem::transmute(slice) }
}

/// Does this slice reside entirely within RAM?
pub(crate) fn slice_in_ram<T>(slice: *const [T]) -> bool {
    let (ptr, len) = slice_ptr_parts(slice);
    let ptr = ptr as usize;
    ptr >= SRAM_LOWER && (ptr + len * core::mem::size_of::<T>()) < SRAM_UPPER
}

/// Return an error if slice is not in RAM. Skips check if slice is zero-length.
pub(crate) fn slice_in_ram_or<T, E>(slice: *const [T], err: E) -> Result<(), E> {
    let (_, len) = slice_ptr_parts(slice);
    if len == 0 || slice_in_ram(slice) {
        Ok(())
    } else {
        Err(err)
    }
}
