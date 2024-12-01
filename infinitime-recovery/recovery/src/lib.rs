#![no_std]

use embedded_hal_async::delay::DelayNs;
use embedded_storage::nor_flash::NorFlash;

const MCUBOOT_DEST: u32 = 0x00000000;
const RECOVERY_DEST: u32 = 0x00008000;

pub async fn recover<N: NorFlash, D: DelayNs>(
    flash: &mut N,
    delay: &mut D,
    buf: &mut [u8],
    mcuboot: &[u8],
    recovery: &[u8],
) {
    flash.erase(0, 0x8000).unwrap();
    delay.delay_ms(1000).await;

    let magic: &[u8] = &[
        0xf3, 0x95, 0xc2, 0x77, 0x7f, 0xef, 0xd2, 0x60, 0x0f, 0x50, 0x52, 0x35, 0x80, 0x79, 0xb6, 0x2c,
    ];
    let mut pos = MCUBOOT_DEST;
    for chunk in mcuboot.chunks(4096) {
        buf[..chunk.len()].copy_from_slice(chunk);
        if chunk.len() < buf.len() {
            for slice in buf[chunk.len()..].chunks_mut(magic.len()) {
                let to_copy = slice.len();
                slice[0..to_copy].copy_from_slice(&magic[0..to_copy]);
            }
        }

        flash.write(pos, &buf[..]).unwrap();
        pos += chunk.len() as u32;
        delay.delay_ms(100).await;
    }

    for page in (0x8000..0x39000).step_by(4096) {
        flash.erase(page, page + 4096).unwrap();
        delay.delay_ms(100).await;
    }

    let mut pos = RECOVERY_DEST;
    for chunk in recovery.chunks(4096) {
        buf[..chunk.len()].copy_from_slice(chunk);
        if chunk.len() < buf.len() {
            for slice in buf[chunk.len()..].chunks_mut(magic.len()) {
                let to_copy = slice.len();
                slice[0..to_copy].copy_from_slice(&magic[0..to_copy]);
            }
        }

        flash.write(pos, &buf[..]).unwrap();
        pos += chunk.len() as u32;
        delay.delay_ms(100).await;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[futures_test::test]
    async fn test_recovery() {
        const SIZE: usize = 512 * 1024;
        let mut mem_flash: MemFlash<SIZE, 4096, 4> = MemFlash::new(0xFF);

        let mcuboot: [u8; 14555] = [0x42; 14555];
        let recovery: [u8; 68832] = [0x33; 68832];

        #[repr(align(4))]
        struct AlignedBuffer([u8; 4096]);

        let mut buf = AlignedBuffer([0; 4096]);
        let mut t = TestTimer;
        recover(&mut mem_flash, &mut t, &mut buf.0, &mcuboot[..], &recovery[..]).await;
        assert_eq!(&mem_flash.mem[..14555], mcuboot);
        assert_eq!(&mem_flash.mem[0x8000..0x8000 + recovery.len()], recovery);
    }

    struct TestTimer;
    impl embedded_hal_async::delay::DelayNs for TestTimer {
        async fn delay_ns(&mut self, _d: u32) {}
    }

    use core::ops::{Bound, Range, RangeBounds};

    use embedded_storage::nor_flash::{ErrorType, NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash};

    pub struct MemFlash<const SIZE: usize, const ERASE_SIZE: usize, const WRITE_SIZE: usize> {
        pub mem: [u8; SIZE],
        pub pending_write_successes: Option<usize>,
    }

    #[derive(Debug)]
    pub struct MemFlashError;

    impl<const SIZE: usize, const ERASE_SIZE: usize, const WRITE_SIZE: usize> MemFlash<SIZE, ERASE_SIZE, WRITE_SIZE> {
        pub const fn new(fill: u8) -> Self {
            Self {
                mem: [fill; SIZE],
                pending_write_successes: None,
            }
        }

        fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), MemFlashError> {
            let len = bytes.len();
            bytes.copy_from_slice(&self.mem[offset as usize..offset as usize + len]);
            Ok(())
        }

        fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), MemFlashError> {
            let offset = offset as usize;
            assert!(bytes.len() % WRITE_SIZE == 0);
            assert!(offset % WRITE_SIZE == 0);
            assert!(offset + bytes.len() <= SIZE);

            if let Some(pending_successes) = self.pending_write_successes {
                if pending_successes > 0 {
                    self.pending_write_successes = Some(pending_successes - 1);
                } else {
                    return Err(MemFlashError);
                }
            }

            for ((offset, mem_byte), new_byte) in self
                .mem
                .iter_mut()
                .enumerate()
                .skip(offset)
                .take(bytes.len())
                .zip(bytes)
            {
                assert_eq!(0xFF, *mem_byte, "Offset {} is not erased", offset);
                *mem_byte = *new_byte;
            }

            Ok(())
        }

        fn erase(&mut self, from: u32, to: u32) -> Result<(), MemFlashError> {
            let from = from as usize;
            let to = to as usize;
            assert!(from % ERASE_SIZE == 0);
            assert!(to % ERASE_SIZE == 0, "To: {}, erase size: {}", to, ERASE_SIZE);
            for i in from..to {
                self.mem[i] = 0xFF;
            }
            Ok(())
        }

        pub fn program(&mut self, offset: u32, bytes: &[u8]) -> Result<(), MemFlashError> {
            let offset = offset as usize;
            assert!(bytes.len() % WRITE_SIZE == 0);
            assert!(offset % WRITE_SIZE == 0);
            assert!(offset + bytes.len() <= SIZE);

            self.mem[offset..offset + bytes.len()].copy_from_slice(bytes);

            Ok(())
        }
    }

    impl<const SIZE: usize, const ERASE_SIZE: usize, const WRITE_SIZE: usize> Default
        for MemFlash<SIZE, ERASE_SIZE, WRITE_SIZE>
    {
        fn default() -> Self {
            Self::new(0xFF)
        }
    }

    impl<const SIZE: usize, const ERASE_SIZE: usize, const WRITE_SIZE: usize> ErrorType
        for MemFlash<SIZE, ERASE_SIZE, WRITE_SIZE>
    {
        type Error = MemFlashError;
    }

    impl NorFlashError for MemFlashError {
        fn kind(&self) -> NorFlashErrorKind {
            NorFlashErrorKind::Other
        }
    }

    impl<const SIZE: usize, const ERASE_SIZE: usize, const WRITE_SIZE: usize> ReadNorFlash
        for MemFlash<SIZE, ERASE_SIZE, WRITE_SIZE>
    {
        const READ_SIZE: usize = 1;

        fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
            self.read(offset, bytes)
        }

        fn capacity(&self) -> usize {
            SIZE
        }
    }

    impl<const SIZE: usize, const ERASE_SIZE: usize, const WRITE_SIZE: usize> NorFlash
        for MemFlash<SIZE, ERASE_SIZE, WRITE_SIZE>
    {
        const WRITE_SIZE: usize = WRITE_SIZE;
        const ERASE_SIZE: usize = ERASE_SIZE;

        fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
            self.write(offset, bytes)
        }

        fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
            self.erase(from, to)
        }
    }
}
