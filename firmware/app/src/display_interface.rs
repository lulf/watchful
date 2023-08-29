
use display_interface::{DataFormat, DisplayError, WriteOnlyDataCommand};
use embedded_hal::digital::OutputPin;
use embedded_hal::spi::SpiDevice;

/// SPI display interface.
///
/// This combines the SPI peripheral and a data/command pin
pub struct SPIDeviceInterface<SPI, DC> {
    spi: SPI,
    dc: DC,
}

impl<SPI, DC> SPIDeviceInterface<SPI, DC>
where
    SPI: SpiDevice,
    DC: OutputPin,
{
    /// Create new SPI interface for communciation with a display driver
    pub fn new(spi: SPI, dc: DC) -> Self {
        Self { spi, dc }
    }
}

impl<SPI, DC> WriteOnlyDataCommand for SPIDeviceInterface<SPI, DC>
where
    SPI: SpiDevice,
    DC: OutputPin,
{
    fn send_commands(&mut self, cmds: DataFormat<'_>) -> Result<(), DisplayError> {
        // 1 = data, 0 = command
        self.dc.set_low().map_err(|_| DisplayError::DCError)?;

        send_u8(&mut self.spi, cmds).map_err(|_| DisplayError::BusWriteError)?;
        Ok(())
    }

    fn send_data(&mut self, buf: DataFormat<'_>) -> Result<(), DisplayError> {
        // 1 = data, 0 = command
        self.dc.set_high().map_err(|_| DisplayError::DCError)?;

        send_u8(&mut self.spi, buf).map_err(|_| DisplayError::BusWriteError)?;
        Ok(())
    }
}

fn send_u8<T: SpiDevice>(spi: &mut T, words: DataFormat<'_>) -> Result<(), T::Error> {
    match words {
        DataFormat::U8(slice) => spi.write(slice),
        DataFormat::U16(slice) => {
            use byte_slice_cast::*;
            spi.write(slice.as_byte_slice())
        }
        DataFormat::U16LE(slice) => {
            use byte_slice_cast::*;
            for v in slice.as_mut() {
                *v = v.to_le();
            }
            spi.write(slice.as_byte_slice())
        }
        DataFormat::U16BE(slice) => {
            use byte_slice_cast::*;
            for v in slice.as_mut() {
                *v = v.to_be();
            }
            spi.write(slice.as_byte_slice())
        }
        DataFormat::U8Iter(iter) => {
            let mut buf = [0; 32];
            let mut i = 0;

            for v in iter.into_iter() {
                buf[i] = v;
                i += 1;

                if i == buf.len() {
                    spi.write(&buf)?;
                    i = 0;
                }
            }

            if i > 0 {
                spi.write(&buf[..i])?;
            }

            Ok(())
        }
        DataFormat::U16LEIter(iter) => {
            use byte_slice_cast::*;
            let mut buf = [0; 32];
            let mut i = 0;

            for v in iter.map(u16::to_le) {
                buf[i] = v;
                i += 1;

                if i == buf.len() {
                    spi.write(&buf.as_byte_slice())?;
                    i = 0;
                }
            }

            if i > 0 {
                spi.write(&buf[..i].as_byte_slice())?;
            }

            Ok(())
        }
        DataFormat::U16BEIter(iter) => {
            use byte_slice_cast::*;
            let mut buf = [0; 64];
            let mut i = 0;
            let len = buf.len();

            for v in iter.map(u16::to_be) {
                buf[i] = v;
                i += 1;

                if i == len {
                    spi.write(&buf.as_byte_slice())?;
                    i = 0;
                }
            }

            if i > 0 {
                spi.write(&buf[..i].as_byte_slice())?;
            }

            Ok(())
        }
        _ => unimplemented!(),
    }
}
