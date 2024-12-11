use defmt::{info, unwrap, warn};
use embassy_executor::Spawner;
use embedded_storage::nor_flash::NorFlash;
use heapless::Vec;
use nrf_dfu_target::prelude::{DfuRequest, DfuStatus, DfuTarget, FirmwareInfo, FirmwareType, HardwareInfo};
use static_cell::StaticCell;
use trouble_host::attribute::Characteristic;
use trouble_host::gatt::GattEvent;
use trouble_host::prelude::*;

use crate::DfuConfig;

pub const MTU: usize = 120;
// Aligned to 4 bytes + 3 bytes for header
pub const ATT_MTU: usize = MTU + 3;

type Target = DfuTarget<256>;
type NrfController = nrf_sdc::SoftdeviceController<'static>;

#[gatt_service(uuid = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E")]
pub struct NrfUartService {
    #[characteristic(uuid = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E", write)]
    rx: Vec<u8, ATT_MTU>,

    #[characteristic(uuid = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E", notify)]
    tx: Vec<u8, ATT_MTU>,
}

/*
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
*/

#[gatt_service(uuid = "FE59")]
pub struct NrfDfuService {
    #[characteristic(uuid = "8EC90001-F315-4F60-9FB8-838830DAEA50", write, notify)]
    control: Vec<u8, ATT_MTU>,

    /// The maximum size of each packet is derived from the Att MTU size of the connection.
    /// The maximum Att MTU size of the DFU Service is 256 bytes (saved in NRF_SDH_BLE_GATT_MAX_MTU_SIZE),
    /// making the maximum size of the DFU Packet characteristic 253 bytes. (3 bytes are used for opcode and handle ID upon writing.)
    #[characteristic(uuid = "8EC90002-F315-4F60-9FB8-838830DAEA50", write_without_response, notify)]
    packet: Vec<u8, ATT_MTU>,
}

#[gatt_server]
pub struct PineTimeServer {
    dfu: NrfDfuService,
    // uart: NrfUartService,
}

/*
#[gatt_client(uuid = "1805")]
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
*/

impl PineTimeServer<'_, '_, NrfController> {
    pub async fn handle<DFU: NorFlash>(
        &self,
        target: &mut Target,
        dfu: &mut DFU,
        connection: Connection<'static>,
        handle: u16,
    ) -> Option<DfuStatus> {
        if handle == self.dfu.control.handle {
            let data = unwrap!(self.get(&self.dfu.control));
            if let Ok((request, _)) = DfuRequest::decode(&data) {
                let (response, status) = target.process(request, dfu);
                let mut buf: [u8; 32] = [0; 32];
                if let Ok(len) = response.encode(&mut buf[..]) {
                    let response = Vec::from_slice(&buf[..len]).unwrap();
                    if let Err(e) = self.notify(&self.dfu.control, &connection, &response).await {
                        warn!("Error notifying control: {:?}", e);
                    }
                }
                Some(status)
            } else {
                None
            }
        } else if handle == self.dfu.packet.handle {
            let data = unwrap!(self.get(&self.dfu.packet));
            let request = DfuRequest::Write { data: &data[..] };
            let (response, status) = target.process(request, dfu);
            let mut buf: [u8; 32] = [0; 32];
            if let Ok(len) = response.encode(&mut buf[..]) {
                let response = Vec::from_slice(&buf[..len]).unwrap();
                if let Err(e) = self.notify(&self.dfu.control, &connection, &response).await {
                    warn!("Error notifying control: {:?}", e);
                }

                if let Err(e) = self.notify(&self.dfu.packet, &connection, &response).await {
                    warn!("Error notifying packet: {:?}", e);
                }
            }
            Some(status)
        } else {
            None
        }
    }
}

/// Size of L2CAP packets (ATT MTU is this - 4)
const L2CAP_MTU: usize = 27;
const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 2; // Signal + att
type BleResources = HostResources<NrfController, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX, L2CAP_MTU>;
static RESOURCES: StaticCell<BleResources> = StaticCell::new();

fn ble_addr() -> Address {
    let ficr = embassy_nrf::pac::FICR;
    let high = u64::from((ficr.deviceaddr(1).read() & 0x0000ffff) | 0x0000c000);
    let addr = high << 32 | u64::from(ficr.deviceaddr(0).read());
    Address::random(unwrap!(addr.to_le_bytes()[..6].try_into()))
}

pub fn start(spawner: Spawner, controller: NrfController, dfu_config: DfuConfig<'static>) {
    let resources = RESOURCES.init(BleResources::new(PacketQos::None));
    let (stack, peripheral, _, runner) = trouble_host::new(controller, resources)
        .set_random_address(ble_addr())
        .build();

    let gatt = unwrap!(PineTimeServer::new_with_config(
        stack,
        GapConfig::Peripheral(PeripheralConfig {
            name: "Watchful",
            appearance: &appearance::power_device::GENERIC_POWER_DEVICE,
        }),
    ));
    static SERVER: StaticCell<PineTimeServer<'static, 'static, NrfController>> = StaticCell::new();
    let server = SERVER.init(gatt);

    spawner.must_spawn(ble_task(runner));
    spawner.must_spawn(gatt_task(server));
    spawner.must_spawn(advertise_task(stack, peripheral, server, dfu_config));
}

#[embassy_executor::task]
async fn ble_task(mut runner: Runner<'static, NrfController>) {
    unwrap!(runner.run().await);
}

#[embassy_executor::task]
async fn gatt_task(server: &'static PineTimeServer<'_, '_, NrfController>) {
    unwrap!(server.run().await);
}

#[embassy_executor::task]
async fn advertise_task(
    stack: Stack<'static, NrfController>,
    mut peripheral: Peripheral<'static, NrfController>,
    server: &'static PineTimeServer<'_, '_, NrfController>,
    mut dfu_config: DfuConfig<'static>,
) {
    let mut advertiser_data = [0; 31];
    unwrap!(AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::ServiceUuids16(&[Uuid::Uuid16([0x0f, 0x18])]),
            AdStructure::CompleteLocalName(b"Watchful"),
        ],
        &mut advertiser_data[..],
    ));
    let mut advertiser = unwrap!(
        peripheral
            .advertise(
                &Default::default(),
                Advertisement::ConnectableScannableUndirected {
                    adv_data: &advertiser_data[..],
                    scan_data: &[],
                },
            )
            .await
    );
    loop {
        match advertiser.accept().await {
            Ok(conn) => process(stack, conn, server, &mut dfu_config).await,
            Err(e) => {
                warn!("Error advertising: {:?}", e);
            }
        }
    }
}

async fn process(
    stack: Stack<'static, NrfController>,
    connection: Connection<'static>,
    server: &'static PineTimeServer<'_, '_, NrfController>,
    dfu_config: &mut DfuConfig<'static>,
) {
    let ficr = embassy_nrf::pac::FICR;
    let part = ficr.info().part().read().part().to_bits();
    let variant = ficr.info().variant().read().variant().to_bits();

    let hw_info = HardwareInfo {
        part,
        variant,
        rom_size: 0,
        ram_size: 0,
        rom_page_size: 0,
    };

    let fw_info = FirmwareInfo {
        ftype: FirmwareType::Application,
        version: 1,
        addr: 0,
        len: 0,
    };

    // Synchronize time
    let s = Spawner::for_current_executor().await;
    s.must_spawn(sync_time(stack, connection.clone()));

    //let mut dfu = dfu_config.dfu();
    //let mut target = DfuTarget::new(dfu.size(), fw_info, hw_info);

    loop {
        match connection.next().await {
            ConnectionEvent::Disconnected { reason: _ } => {
                break;
            }
            ConnectionEvent::Gatt { event, connection } => match event {
                GattEvent::Write { value_handle } => {
                    defmt::info!("Gatt write!");
                    /*
                    if let Some(DfuStatus::DoneReset) =
                        server.handle(&mut target, &mut dfu, connection, value_handle).await
                    {
                        warn!("Supposed to reset!");
                        //let mut magic = crate::AlignedBuffer([0; 4]);
                        //let mut state = embassy_boot::FirmwareState::new(dfu_config.state(), &mut magic.0);
                        //match state.mark_updated().await {
                        //    Ok(_) => {
                        //        info!("Firmware updated, resetting");
                        //        cortex_m::peripheral::SCB::sys_reset();
                        //    }
                        //    Err(e) => {
                        //        panic!("Error marking firmware updated: {:?}", e);
                        //    }
                        //}
                    }*/
                }
                _ => {}
            },
        }
    }
}

#[embassy_executor::task]
async fn sync_time(stack: Stack<'static, NrfController>, conn: Connection<'static>) {
    info!("Synchronizing time, creating gatt client");
    let client = unwrap!(GattClient::<_, 10, 24>::new(stack, &conn).await);

    info!("Discovering time service");
    let services = unwrap!(client.services_by_uuid(&Uuid::new_short(0x1805)).await);
    let service = services.first().unwrap().clone();

    info!("Looking for value handle");
    let c: Characteristic<u8> = unwrap!(client.characteristic_by_uuid(&service, &Uuid::new_short(0x2a2b)).await);

    info!("Reading characteristic");
    let mut data = [0; 10];
    unwrap!(client.read_characteristic(&c, &mut data[..]).await);

    if let Some(time) = parse_time(data) {
        crate::CLOCK.set(time);
    }
}

fn parse_time(data: [u8; 10]) -> Option<time::PrimitiveDateTime> {
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
            return Some(dt);
        }
    }
    None
}
