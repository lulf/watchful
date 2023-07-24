use embassy_boot::{AlignedBuffer, FirmwareUpdater, FirmwareUpdaterConfig, FirmwareUpdaterError};
use embassy_futures::block_on;
use embassy_futures::select::{select, Either};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::channel::{Channel, Receiver, Sender};
use embassy_sync::pipe::{Pipe, Reader, Writer};
use embedded_hal_02::blocking::delay::DelayMs;
use embedded_storage::nor_flash::{NorFlashError, NorFlashErrorKind};
use embedded_storage_async::nor_flash::NorFlash;

use crate::crc::*;

const DFU_PROTOCOL_VERSION: u8 = 0x01;
const OBJ_TYPE_COMMAND_IDX: usize = 0;
const OBJ_TYPE_DATA_IDX: usize = 1;

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Command {
    Swap { crc: u32, size: u32 },
}

/// Represents the target of a firmware update. Dispatches commands and data to
/// a channel and data pipe which can be consumed by a separate async task.
pub struct DfuTarget<DFU: NorFlash> {
    crc_receipt_interval: u16,
    receipt_count: u16,
    objects: [Object; 2],
    current: usize,
    fw_info: FirmwareInfo,
    hw_info: HardwareInfo,

    // TODO: Configurable
    magic: AlignedBuffer<4>,
    buffer: AlignedBuffer<4096>,

    flash: DFU,

    boffset: usize,
    offset: usize,
    mtu: u16,
}

/// Object representing a firmware blob. Tracks information about the firmware to be updated such as the CRC.
pub struct Object {
    obj_type: ObjectType,
    offset: u32,
    crc: Crc32,
    size: u32,
}

/// Information about the firmware.
pub struct FirmwareInfo {
    pub ftype: FirmwareType,
    pub version: u32,
    pub addr: u32,
    pub len: u32,
}

/// Information about the hardware.
pub struct HardwareInfo {
    pub part: u32,
    pub variant: u32,
    pub rom_size: u32,
    pub ram_size: u32,
    pub rom_page_size: u32,
}

/// Represents a DFU request according to the nRF DFU protocol from SDK 17.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DfuRequest<'m> {
    ProtocolVersion,
    Create { obj_type: ObjectType, obj_size: u32 },
    SetReceiptNotification { target: u16 },
    Crc,
    Execute,
    Select { obj_type: ObjectType },
    MtuGet,
    Write { data: &'m [u8] },
    Ping { id: u8 },
    HwVersion,
    FwVersion { image_id: u8 },
    Abort,
}

/// Represents a DFU resopnse according to the nRF DFU protocol from SDK 17.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DfuResponse<'m> {
    request: DfuRequest<'m>,
    result: DfuResult,
    body: Option<DfuResponseBody>,
}

/// Possible result values for a response.
#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DfuResult {
    Invalid,
    Success,
    OpNotSupported,
    InvalidParameter,
    InsufficientResources,
    InvalidObject,
    UnsupportedType,
    OpNotPermitted,
    OpFailed,
    ExtError(u8),
}

/// Possible payloads for a response.
#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DfuResponseBody {
    ProtocolVersion {
        version: u8,
    },
    Crc {
        offset: u32,
        crc: u32,
    },
    Select {
        offset: u32,
        crc: u32,
        max_size: u32,
    },
    Mtu {
        mtu: u16,
    },
    Write {
        crc: u32,
    },
    Ping {
        id: u8,
    },
    HwVersion {
        part: u32,
        variant: u32,
        rom_size: u32,
        ram_size: u32,
        rom_page_size: u32,
    },
    FwVersion {
        ftype: FirmwareType,
        version: u32,
        addr: u32,
        len: u32,
    },
}

/// Object types supported by the protocol.
#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ObjectType {
    Invalid = 0,
    Command = 1,
    Data = 2,
}

/// Firmware types supported by the protocol.
#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FirmwareType {
    Softdevice,
    Application,
    Bootloader,
    Unknown,
}

#[derive(Debug)]
pub enum Error {
    Updater(FirmwareUpdaterError),
    Flash(NorFlashErrorKind),
    Mtu,
}

impl From<FirmwareUpdaterError> for Error {
    fn from(error: FirmwareUpdaterError) -> Self {
        Self::Updater(error)
    }
}

impl From<NorFlashErrorKind> for Error {
    fn from(error: NorFlashErrorKind) -> Self {
        Self::Flash(error)
    }
}

impl<DFU: NorFlash> DfuTarget<DFU> {
    pub fn new(flash: DFU, mtu: u16, fw_info: FirmwareInfo, hw_info: HardwareInfo) -> Self {
        let flash_size = flash.capacity() as u32;
        Self {
            crc_receipt_interval: 0,
            receipt_count: 0,
            objects: [
                Object {
                    obj_type: ObjectType::Command,
                    offset: 0,
                    crc: Crc32::init(),
                    size: 512,
                },
                Object {
                    obj_type: ObjectType::Data,
                    offset: 0,
                    crc: Crc32::init(),
                    size: flash_size,
                },
            ],

            current: 0,
            fw_info,
            hw_info,

            magic: AlignedBuffer([0; 4]),
            buffer: AlignedBuffer([0; 4096]),

            offset: 0,
            boffset: 0,
            mtu,
            flash,
        }
    }

    pub fn process<'m>(&mut self, request: DfuRequest<'m>) -> DfuResponse<'m> {
        trace!("DFU REQUEST {:?}", request);
        let response = self.process_inner(request);
        trace!("DFU RESPONSE {:?}", response);

        /*
        let obj = self.objects[self.current];
        if response.request == DfuRequest::Execute
            && obj.obj_type == ObjectType::Data
            && response.result == DfuResult::Success
        {
            return response;
        } else {}*/
        response
    }

    pub fn process_inner<'m>(&mut self, request: DfuRequest<'m>) -> DfuResponse<'m> {
        match request {
            DfuRequest::ProtocolVersion => {
                DfuResponse::new(request, DfuResult::Success).body(DfuResponseBody::ProtocolVersion {
                    version: DFU_PROTOCOL_VERSION,
                })
            }
            DfuRequest::Create { obj_type, obj_size } => {
                let idx = match obj_type {
                    ObjectType::Command => Some(OBJ_TYPE_COMMAND_IDX),
                    ObjectType::Data => Some(OBJ_TYPE_DATA_IDX),
                    _ => None,
                };
                if let Some(idx) = idx {
                    self.objects[idx] = Object {
                        obj_type,
                        size: obj_size,
                        offset: 0,
                        crc: Crc32::init(),
                    };
                    self.current = idx;
                    if let ObjectType::Data = obj_type {
                        let size = self.objects[self.current].size;
                        let to = size + (DFU::ERASE_SIZE as u32 - size % DFU::ERASE_SIZE as u32);
                        match block_on(self.flash.erase(0, to as u32)) {
                            Ok(_) => {
                                info!("Erase operation complete");
                                self.objects[self.current].offset = 0;
                            }
                            Err(e) => {
                                #[cfg(feature = "defmt")]
                                let e = defmt::Debug2Format(&e);
                                warn!("Error during erase: {:?}", e);
                            }
                        }
                    }
                }
                self.receipt_count = 0;
                DfuResponse::new(request, DfuResult::Success)
            }
            DfuRequest::SetReceiptNotification { target } => {
                self.crc_receipt_interval = target;
                DfuResponse::new(request, DfuResult::Success)
            }
            DfuRequest::Crc => DfuResponse::new(request, DfuResult::Success).body(DfuResponseBody::Crc {
                offset: self.objects[self.current].offset,
                crc: self.objects[self.current].crc.finish(),
            }),
            DfuRequest::Execute => {
                let obj = &mut self.objects[self.current];
                if obj.offset != obj.size {
                    DfuResponse::new(request, DfuResult::OpNotSupported)
                } else {
                    if let ObjectType::Data = obj.obj_type {
                        if let Err(e) = self.flush() {
                            #[cfg(feature = "defmt")]
                            let e = defmt::Debug2Format(&e);
                            warn!("Error flushing buffer: {:?}", e);
                            DfuResponse::new(request, DfuResult::OpFailed)
                        } else {
                            DfuResponse::new(request, DfuResult::Success)
                        }
                    } else {
                        DfuResponse::new(request, DfuResult::Success)
                    }
                }
            }
            DfuRequest::Select { obj_type } => {
                let idx = match obj_type {
                    ObjectType::Command => Some(OBJ_TYPE_COMMAND_IDX),
                    ObjectType::Data => Some(OBJ_TYPE_DATA_IDX),
                    _ => None,
                };
                if let Some(idx) = idx {
                    DfuResponse::new(request, DfuResult::Success).body(DfuResponseBody::Select {
                        offset: self.objects[idx].offset,
                        crc: self.objects[idx].crc.finish(),
                        max_size: self.objects[idx].size,
                    })
                } else {
                    DfuResponse::new(request, DfuResult::InvalidObject)
                }
            }

            DfuRequest::MtuGet => {
                DfuResponse::new(request, DfuResult::Success).body(DfuResponseBody::Mtu { mtu: self.mtu })
            }
            DfuRequest::Write { data } => {
                if let ObjectType::Data = self.objects[self.current].obj_type {
                    if let Err(e) = self.write(data) {
                        #[cfg(feature = "defmt")]
                        let e = defmt::Debug2Format(&e);
                        warn!("Write Error: {:?}", e);
                        return DfuResponse::new(request, DfuResult::OpFailed);
                    }
                }
                let obj = &mut self.objects[self.current];
                obj.crc.add(data);
                obj.offset += data.len() as u32;

                let mut response = DfuResponse::new(request, DfuResult::Success);
                if self.crc_receipt_interval > 0 {
                    self.receipt_count += 1;
                    if self.receipt_count == self.crc_receipt_interval {
                        self.receipt_count = 0;
                        response = response.body(DfuResponseBody::Crc {
                            offset: obj.offset,
                            crc: obj.crc.finish(),
                        });
                    }
                } else {
                    response = response.body(DfuResponseBody::Crc {
                        offset: obj.offset,
                        crc: obj.crc.finish(),
                    });
                };
                response
            }
            DfuRequest::Ping { id } => DfuResponse::new(request, DfuResult::Success).body(DfuResponseBody::Ping { id }),
            DfuRequest::HwVersion => DfuResponse::new(request, DfuResult::Success).body(DfuResponseBody::HwVersion {
                part: self.hw_info.part,
                variant: self.hw_info.variant,
                rom_size: self.hw_info.rom_size,
                ram_size: self.hw_info.ram_size,
                rom_page_size: self.hw_info.rom_page_size,
            }),
            DfuRequest::FwVersion { image_id: _ } => {
                DfuResponse::new(request, DfuResult::Success).body(DfuResponseBody::FwVersion {
                    ftype: self.fw_info.ftype,
                    version: self.fw_info.version,
                    addr: self.fw_info.addr,
                    len: self.fw_info.len,
                })
            }
            DfuRequest::Abort => {
                self.objects[0].crc.reset();
                self.objects[0].offset = 0;
                self.objects[1].crc.reset();
                self.objects[1].offset = 0;
                self.receipt_count = 0;
                DfuResponse::new(request, DfuResult::Success)
            }
        }
    }

    fn write(&mut self, data: &[u8]) -> Result<(), Error> {
        let mut copied = 0;
        while copied < data.len() {
            if self.boffset == self.buffer.0.len() {
                match block_on(self.flash.write(self.offset as u32, &self.buffer.0)) {
                    Ok(_) => {
                        self.offset += self.boffset;
                        info!("Wrote {} bytes to flash (total {})", self.boffset, self.offset);
                        self.boffset = 0;
                    }
                    Err(e) => {
                        return Err(Error::Flash(e.kind()));
                    }
                }
            } else {
                let to_copy = core::cmp::min(self.buffer.0.len() - self.boffset, data.len() - copied);
                self.buffer.0[self.boffset as usize..self.boffset as usize + to_copy]
                    .copy_from_slice(&data[copied..copied + to_copy]);
                self.boffset += to_copy;
                copied += to_copy;
            }
        }
        Ok(())
    }

    fn flush(&mut self) -> Result<(), Error> {
        if self.boffset > 0 {
            let size = self.boffset as u32;
            let to = size + (DFU::WRITE_SIZE as u32 - size % DFU::WRITE_SIZE as u32);
            match block_on(self.flash.write(self.offset as u32, &self.buffer.0[..to as usize])) {
                Ok(_) => {
                    self.offset += self.boffset;
                    info!("Wrote {} bytes to flash (total {})", self.boffset, self.offset);
                    self.boffset = 0;
                }
                Err(e) => {
                    return Err(Error::Flash(e.kind()));
                }
            }
        }
        Ok(())
    }
}

impl<'m> DfuRequest<'m> {
    fn code(&self) -> u8 {
        match self {
            Self::ProtocolVersion => 0x00,
            Self::Create {
                obj_type: _,
                obj_size: _,
            } => 0x01,
            Self::SetReceiptNotification { target: _ } => 0x02,
            Self::Crc => 0x03,
            Self::Execute => 0x04,
            Self::Select { obj_type: _ } => 0x06,
            Self::MtuGet => 0x07,
            Self::Write { data: _ } => 0x08,
            Self::Ping { id: _ } => 0x09,
            Self::HwVersion => 0x0A,
            Self::FwVersion { image_id: _ } => 0x0B,
            Self::Abort => 0x0C,
        }
    }

    pub fn decode(data: &'m [u8]) -> Result<(DfuRequest<'m>, &'m [u8]), ()> {
        let mut data = ReadBuf::new(data);
        let op = data.decode_u8()?;
        let req = match op {
            0x00 => Ok(Self::ProtocolVersion),
            0x01 => {
                let obj_type = ObjectType::try_from(data.decode_u8()?)?;
                let obj_size = data.decode_u32()?;

                Ok(Self::Create { obj_type, obj_size })
            }
            0x02 => {
                let target = data.decode_u16()?;
                Ok(Self::SetReceiptNotification { target })
            }
            0x03 => Ok(Self::Crc),
            0x04 => Ok(Self::Execute),
            0x06 => {
                let obj_type = ObjectType::try_from(data.decode_u8()?)?;
                Ok(Self::Select { obj_type })
            }
            0x07 => Ok(Self::MtuGet),
            0x08 => Ok(Self::Write { data: data.slice() }),
            0x09 => {
                let id = data.decode_u8()?;
                Ok(Self::Ping { id })
            }
            0x0A => Ok(Self::HwVersion),
            0x0B => {
                let image_id = data.decode_u8()?;
                Ok(Self::FwVersion { image_id })
            }
            0x0C => Ok(Self::Abort),
            _ => Err(()),
        }?;

        Ok((req, data.release()))
    }
}

impl<'m> DfuResponse<'m> {
    pub fn new(request: DfuRequest<'m>, result: DfuResult) -> DfuResponse<'m> {
        DfuResponse {
            request,
            result,
            body: None,
        }
    }

    pub fn body(self, body: DfuResponseBody) -> Self {
        Self {
            request: self.request,
            result: self.result,
            body: Some(body),
        }
    }
    pub fn encode(&self, buf: &mut [u8]) -> Result<usize, ()> {
        let mut buf = WriteBuf::new(buf);
        const DFU_RESPONSE_OP_CODE: u8 = 0x60;
        buf.encode_u8(DFU_RESPONSE_OP_CODE)?;
        buf.encode_u8(self.request.code())?;

        let len = self.result.encode(buf.slice())?;
        buf.advance(len)?;

        if let Some(body) = self.body {
            let len = body.encode(buf.slice())?;
            buf.advance(len)?;
        }
        Ok(buf.release())
    }
}

impl DfuResult {
    fn encode(&self, buf: &mut [u8]) -> Result<usize, ()> {
        let mut buf = WriteBuf::new(buf);
        let code = match self {
            DfuResult::Invalid => 0x00,
            DfuResult::Success => 0x01,
            DfuResult::OpNotSupported => 0x02,
            DfuResult::InvalidParameter => 0x03,
            DfuResult::InsufficientResources => 0x04,
            DfuResult::InvalidObject => 0x05,
            DfuResult::UnsupportedType => 0x07,
            DfuResult::OpNotPermitted => 0x08,
            DfuResult::OpFailed => 0x0A,
            DfuResult::ExtError(_) => 0x0B,
        };

        buf.encode_u8(code)?;
        if let DfuResult::ExtError(code) = self {
            buf.encode_u8(*code)?;
        }

        Ok(buf.pos)
    }
}

impl DfuResponseBody {
    fn encode(&self, buf: &mut [u8]) -> Result<usize, ()> {
        let mut buf = WriteBuf::new(buf);
        match &self {
            DfuResponseBody::ProtocolVersion { version } => buf.encode_u8(*version)?,
            DfuResponseBody::Crc { offset, crc } => {
                buf.encode_u32(*offset)?;
                buf.encode_u32(*crc)?;
            }
            DfuResponseBody::Select { offset, crc, max_size } => {
                buf.encode_u32(*max_size)?;
                buf.encode_u32(*offset)?;
                buf.encode_u32(*crc)?;
            }
            DfuResponseBody::Mtu { mtu } => {
                buf.encode_u16(*mtu)?;
            }
            DfuResponseBody::Write { crc } => {
                buf.encode_u32(*crc)?;
            }
            DfuResponseBody::Ping { id } => {
                buf.encode_u8(*id)?;
            }
            DfuResponseBody::HwVersion {
                part,
                variant,
                rom_size,
                ram_size,
                rom_page_size,
            } => {
                buf.encode_u32(*part)?;
                buf.encode_u32(*variant)?;
                buf.encode_u32(*rom_size)?;
                buf.encode_u32(*ram_size)?;
                buf.encode_u32(*rom_page_size)?;
            }
            DfuResponseBody::FwVersion {
                ftype,
                version,
                addr,
                len,
            } => {
                buf.encode_u8((*ftype).into())?;
                buf.encode_u32(*version)?;
                buf.encode_u32(*addr)?;
                buf.encode_u32(*len)?;
            }
        }
        Ok(buf.release())
    }
}

impl TryFrom<u8> for ObjectType {
    type Error = ();
    fn try_from(t: u8) -> Result<Self, Self::Error> {
        match t {
            0 => Ok(Self::Invalid),
            1 => Ok(Self::Command),
            2 => Ok(Self::Data),
            _ => Err(()),
        }
    }
}

impl Into<u8> for ObjectType {
    fn into(self) -> u8 {
        match self {
            Self::Invalid => 0x00,
            Self::Command => 0x01,
            Self::Data => 0x02,
        }
    }
}

impl Into<u8> for FirmwareType {
    fn into(self) -> u8 {
        match self {
            Self::Softdevice => 0x00,
            Self::Application => 0x01,
            Self::Bootloader => 0x02,
            Self::Unknown => 0xFF,
        }
    }
}

struct ReadBuf<'m> {
    data: &'m [u8],
    pos: usize,
}

impl<'m> ReadBuf<'m> {
    fn new(data: &'m [u8]) -> ReadBuf<'m> {
        Self { data, pos: 0 }
    }

    fn decode_u8(&mut self) -> Result<u8, ()> {
        if self.data.len() - self.pos >= 1 {
            let b = self.data[self.pos];
            self.pos += 1;
            Ok(b)
        } else {
            Err(())
        }
    }

    fn decode_u16(&mut self) -> Result<u16, ()> {
        if self.data.len() - self.pos >= 2 {
            let b = u16::from_le_bytes([self.data[self.pos], self.data[self.pos + 1]]);
            self.pos += 2;
            Ok(b)
        } else {
            Err(())
        }
    }

    fn decode_u32(&mut self) -> Result<u32, ()> {
        if self.data.len() - self.pos >= 4 {
            let b = u32::from_le_bytes([
                self.data[self.pos],
                self.data[self.pos + 1],
                self.data[self.pos + 2],
                self.data[self.pos + 3],
            ]);
            self.pos += 4;
            Ok(b)
        } else {
            Err(())
        }
    }

    fn slice(&mut self) -> &'m [u8] {
        let s = &self.data[self.pos..];
        self.pos += s.len();
        s
    }

    fn release(self) -> &'m [u8] {
        &self.data[self.pos..]
    }
}

struct WriteBuf<'m> {
    data: &'m mut [u8],
    pos: usize,
}

impl<'m> WriteBuf<'m> {
    fn new(data: &'m mut [u8]) -> WriteBuf<'m> {
        Self { data, pos: 0 }
    }

    fn encode_u8(&mut self, value: u8) -> Result<(), ()> {
        if self.data.len() - self.pos >= 1 {
            self.data[self.pos] = value;
            self.pos += 1;
            Ok(())
        } else {
            Err(())
        }
    }

    fn encode_u16(&mut self, value: u16) -> Result<(), ()> {
        if self.data.len() - self.pos >= 2 {
            let d = value.to_le_bytes();
            self.data[self.pos] = d[0];
            self.data[self.pos + 1] = d[1];
            self.pos += 2;
            Ok(())
        } else {
            Err(())
        }
    }

    fn encode_u32(&mut self, value: u32) -> Result<(), ()> {
        if self.data.len() - self.pos >= 4 {
            let d = value.to_le_bytes();
            self.data[self.pos] = d[0];
            self.data[self.pos + 1] = d[1];
            self.data[self.pos + 2] = d[2];
            self.data[self.pos + 3] = d[3];
            self.pos += 4;
            Ok(())
        } else {
            Err(())
        }
    }

    fn advance(&mut self, amount: usize) -> Result<(), ()> {
        if self.data.len() - self.pos >= amount {
            self.pos += amount;
            Ok(())
        } else {
            Err(())
        }
    }

    fn release(self) -> usize {
        self.pos
    }

    fn slice(&mut self) -> &mut [u8] {
        &mut self.data[self.pos..]
    }
}
