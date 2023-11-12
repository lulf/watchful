use defmt::{info, warn};
use embedded_storage::nor_flash::NorFlash;
use heapless::Vec;
use nrf_dfu_target::prelude::*;
use nrf_softdevice::ble::gatt_server::NotifyValueError;
use nrf_softdevice::ble::{gatt_client, Connection};

pub const MTU: usize = 120;
// Aligned to 4 bytes + 3 bytes for header
pub const ATT_MTU: usize = MTU + 3;

type Target = DfuTarget<256>;

#[nrf_softdevice::gatt_service(uuid = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E")]
pub struct NrfUartService {
    #[characteristic(uuid = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E", write)]
    rx: Vec<u8, ATT_MTU>,

    #[characteristic(uuid = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E", notify)]
    tx: Vec<u8, ATT_MTU>,
}

impl NrfUartService {
    fn handle(&self, _connection: &mut ConnectionHandle, event: NrfUartServiceEvent) {
        match event {
            NrfUartServiceEvent::TxCccdWrite { notifications } => {
                info!("Enable logging: {}", notifications);
            }
            _ => {}
        }
    }
}

#[nrf_softdevice::gatt_service(uuid = "FE59")]
pub struct NrfDfuService {
    #[characteristic(uuid = "8EC90001-F315-4F60-9FB8-838830DAEA50", write, notify)]
    control: Vec<u8, ATT_MTU>,

    /// The maximum size of each packet is derived from the Att MTU size of the connection.
    /// The maximum Att MTU size of the DFU Service is 256 bytes (saved in NRF_SDH_BLE_GATT_MAX_MTU_SIZE),
    /// making the maximum size of the DFU Packet characteristic 253 bytes. (3 bytes are used for opcode and handle ID upon writing.)
    #[characteristic(uuid = "8EC90002-F315-4F60-9FB8-838830DAEA50", write_without_response, notify)]
    packet: Vec<u8, ATT_MTU>,
}

pub struct ConnectionHandle {
    pub connection: Connection,
    pub notify_control: bool,
    pub notify_packet: bool,
}

impl NrfDfuService {
    fn process<DFU: NorFlash, F: FnOnce(&ConnectionHandle, &[u8]) -> Result<(), NotifyValueError>>(
        &self,
        target: &mut Target,
        dfu: &mut DFU,
        conn: &mut ConnectionHandle,
        request: DfuRequest<'_>,
        notify: F,
    ) -> DfuStatus {
        let (response, status) = target.process(request, dfu);
        let mut buf: [u8; 32] = [0; 32];
        match response.encode(&mut buf[..]) {
            Ok(len) => match notify(&conn, &buf[..len]) {
                Ok(_) => {}
                Err(e) => {
                    warn!("Error sending notification: {:?}", e);
                }
            },
            Err(e) => {
                warn!("Error encoding DFU response: {:?}", e);
            }
        }
        status
    }

    fn handle<DFU: NorFlash>(
        &self,
        target: &mut Target,
        dfu: &mut DFU,
        connection: &mut ConnectionHandle,
        event: NrfDfuServiceEvent,
    ) -> Option<DfuStatus> {
        match event {
            NrfDfuServiceEvent::ControlWrite(data) => {
                if let Ok((request, _)) = DfuRequest::decode(&data) {
                    return Some(self.process(target, dfu, connection, request, |conn, response| {
                        if conn.notify_control {
                            self.control_notify(&conn.connection, &Vec::from_slice(response).unwrap())?;
                        }
                        Ok(())
                    }));
                }
            }
            NrfDfuServiceEvent::ControlCccdWrite { notifications } => {
                connection.notify_control = notifications;
            }
            NrfDfuServiceEvent::PacketWrite(data) => {
                let request = DfuRequest::Write { data: &data[..] };
                return Some(self.process(target, dfu, connection, request, |conn, response| {
                    if conn.notify_packet {
                        self.packet_notify(&conn.connection, &Vec::from_slice(response).unwrap())?;
                    }
                    Ok(())
                }));
            }
            NrfDfuServiceEvent::PacketCccdWrite { notifications } => {
                connection.notify_packet = notifications;
            }
        }
        None
    }
}

#[nrf_softdevice::gatt_server]
pub struct PineTimeServer {
    dfu: NrfDfuService,
    uart: NrfUartService,
}

#[nrf_softdevice::gatt_client(uuid = "1805")]
struct CurrentTimeServiceClient {
    #[characteristic(uuid = "2a2b", write, read, notify)]
    current_time: Vec<u8, 10>,
}

pub async fn sync_time(conn: &Connection, clock: &crate::clock::Clock) {
    if let Ok(time_client) = gatt_client::discover::<CurrentTimeServiceClient>(&conn).await {
        info!("Found time server on peer, synchronizing time");
        match time_client.get_time().await {
            Ok(time) => {
                // info!("Got time from peer: {:?}", defmt::Debug2Format(&time));
                clock.set(time);
            }
            Err(e) => {
                info!("Error retrieving time: {:?}", e);
            }
        }
    }
}

impl CurrentTimeServiceClient {
    pub async fn get_time(&self) -> Result<time::PrimitiveDateTime, gatt_client::ReadError> {
        let data = self.current_time_read().await?;
        if data.len() == 10 {
            let year = u16::from_le_bytes([data[0], data[1]]);
            let month = data[2];
            let day = data[3];
            let hour = data[4];
            let minute = data[5];
            let second = data[6];
            let _weekday = data[7];
            let secs_frac = data[8];

            if let Ok(month) = month.try_into() {
                let date = time::Date::from_calendar_date(year as i32, month, day);
                let micros = secs_frac as u32 * 1000000 / 256;
                let time = time::Time::from_hms_micro(hour, minute, second, micros);
                if let (Ok(time), Ok(date)) = (time, date) {
                    let dt = time::PrimitiveDateTime::new(date, time);
                    return Ok(dt);
                }
            }
        }
        Err(gatt_client::ReadError::Truncated)
    }
}

impl PineTimeServer {
    pub fn handle<DFU: NorFlash>(
        &self,
        target: &mut Target,
        dfu: &mut DFU,
        conn: &mut ConnectionHandle,
        event: PineTimeServerEvent,
    ) -> Option<DfuStatus> {
        match event {
            PineTimeServerEvent::Dfu(event) => self.dfu.handle(target, dfu, conn, event),
            PineTimeServerEvent::Uart(event) => {
                self.uart.handle(conn, event);
                None
            }
        }
    }
}
