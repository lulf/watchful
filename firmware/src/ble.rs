use defmt::{info, unwrap, warn};
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_time::{with_timeout, Duration};
use embedded_storage_async::nor_flash::NorFlash;
use heapless::Vec;
use nrf_dfu_target::prelude::{DfuRequest, DfuStatus, DfuTarget, FirmwareInfo, FirmwareType, HardwareInfo};
use static_cell::StaticCell;
use trouble_host::attribute::Characteristic;
use trouble_host::gatt::GattEvent;
use trouble_host::prelude::*;

use crate::device::Battery;
use crate::DfuConfig;

pub const ATT_MTU: usize = L2CAP_MTU - 4 - 3;

type Target = DfuTarget<256>;
type NrfController = nrf_sdc::SoftdeviceController<'static>;

/*
#[gatt_service(uuid = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E")]
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
*/

// Battery service
#[gatt_service(uuid = service::BATTERY)]
struct BatteryService {
    /// Battery Level
    #[descriptor(uuid = descriptors::VALID_RANGE, read, value = [0, 100])]
    #[descriptor(uuid = descriptors::MEASUREMENT_DESCRIPTION, read, value = "Battery Level")]
    #[characteristic(uuid = characteristic::BATTERY_LEVEL, read, notify)]
    level: u8,
}

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

// NOTE: Disabled as it doesn't properly deal with init/start specific to infinitime protocol
//#[gatt_service(uuid = "23D1BCEA-5F78-2315-DEEF-121230150000")]
//pub struct InfinitimeDfuService {
//    #[characteristic(uuid = "23D1BCEA-5F78-2315-DEEF-121231150000", write, notify)]
//    control: Vec<u8, ATT_MTU>,
//
//    #[characteristic(uuid = "23D1BCEA-5F78-2315-DEEF-121234150000", read, value = 8)]
//    revision: u16,
//
//    /// The maximum size of each packet is derived from the Att MTU size of the connection.
//    /// The maximum Att MTU size of the DFU Service is 256 bytes (saved in NRF_SDH_BLE_GATT_MAX_MTU_SIZE),
//    /// making the maximum size of the DFU Packet characteristic 253 bytes. (3 bytes are used for opcode and handle ID upon writing.)
//    #[characteristic(uuid = "23D1BCEA-5F78-2315-DEEF-121232150000", write_without_response)]
//    packet: Vec<u8, ATT_MTU>,
//}

#[gatt_server]
pub struct PineTimeServer {
    nrfdfu: NrfDfuService,
    battery: BatteryService,
    //   infdfu: InfinitimeDfuService,
    // uart: NrfUartService,
}

impl PineTimeServer<'_> {
    pub async fn handle_dfu_control<DFU: NorFlash>(
        &self,
        target: &mut Target,
        dfu: &mut DFU,
        connection: &Connection<'static>,
        control: &Characteristic<Vec<u8, ATT_MTU>>,
    ) -> Option<DfuStatus> {
        let data: Vec<u8, ATT_MTU> = unwrap!(control.get(self));
        if let Ok((request, _)) = DfuRequest::decode(&data) {
            // info!("[ble] request: {:?}", request);
            let (response, status) = target.process(request, dfu).await;
            let mut buf: [u8; 32] = [0; 32];
            if let Ok(len) = response.encode(&mut buf[..]) {
                let response = Vec::from_slice(&buf[..len]).unwrap();
                if let Err(e) = control.notify(self, &connection, &response).await {
                    warn!("Error notifying control: {:?}", e);
                }
            }
            Some(status)
        } else {
            warn!("unable to decode");
            None
        }
    }

    pub async fn handle_dfu_packet<DFU: NorFlash>(
        &self,
        target: &mut Target,
        dfu: &mut DFU,
        connection: &Connection<'static>,
        control: &Characteristic<Vec<u8, ATT_MTU>>,
        packet: &Characteristic<Vec<u8, ATT_MTU>>,
    ) -> Option<DfuStatus> {
        let data = unwrap!(packet.get(self));
        let request = DfuRequest::Write { data: &data[..] };
        // info!("[ble] write request: {:?}", request);
        let (response, status) = target.process(request, dfu).await;
        let mut buf: [u8; 32] = [0; 32];
        if let Ok(len) = response.encode(&mut buf[..]) {
            let response = Vec::from_slice(&buf[..len]).unwrap();
            if let Err(e) = control.notify(self, &connection, &response).await {
                warn!("Error notifying control: {:?}", e);
            }

            if let Err(e) = packet.notify(self, &connection, &response).await {
                warn!("Error notifying packet: {:?}", e);
            }
        }
        Some(status)
    }

    pub async fn handle<DFU: NorFlash>(
        &self,
        target: &mut Target,
        dfu: &mut DFU,
        connection: &Connection<'static>,
        handle: u16,
    ) -> Option<DfuStatus> {
        if handle == self.nrfdfu.control.handle {
            self.handle_dfu_control(target, dfu, connection, &self.nrfdfu.control)
                .await
        //        } else if handle == self.infdfu.control.handle {
        //            self.handle_dfu_control(target, dfu, connection, &self.infdfu.control)
        //                .await
        } else if handle == self.nrfdfu.packet.handle {
            self.handle_dfu_packet(target, dfu, connection, &self.nrfdfu.control, &self.nrfdfu.packet)
                .await
        //        } else if handle == self.infdfu.packet.handle {
        //            self.handle_dfu_packet(target, dfu, connection, &self.infdfu.control, &self.infdfu.packet)
        //                .await
        } else {
            // Ignore, no need to handle
            None
        }
    }
}

/// Size of L2CAP packets
pub const L2CAP_MTU: usize = 27;
pub const L2CAP_TXQ: u8 = 10;
pub const L2CAP_RXQ: u8 = 10;

const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 2; // Signal + att
type BleResources = HostResources<CONNECTIONS_MAX, L2CAP_CHANNELS_MAX, L2CAP_MTU>;
static RESOURCES: StaticCell<BleResources> = StaticCell::new();
static STACK: StaticCell<Stack<'static, NrfController>> = StaticCell::new();

fn ble_addr() -> Address {
    let ficr = embassy_nrf::pac::FICR;
    let high = u64::from((ficr.deviceaddr(1).read() & 0x0000ffff) | 0x0000c000);
    let addr = high << 32 | u64::from(ficr.deviceaddr(0).read());
    Address::random(unwrap!(addr.to_le_bytes()[..6].try_into()))
}

const NAME: &str = "Watchful";

pub fn start(
    spawner: Spawner,
    controller: NrfController,
    dfu_config: DfuConfig<'static>,
    battery: &'static Battery<'static>,
) {
    let resources = RESOURCES.init(BleResources::new());
    let stack = STACK.init(trouble_host::new(controller, resources).set_random_address(ble_addr()));

    let Host { peripheral, runner, .. } = stack.build();

    let gatt = unwrap!(PineTimeServer::new_with_config(GapConfig::Peripheral(
        PeripheralConfig {
            name: NAME,
            appearance: &appearance::watch::SMARTWATCH,
        }
    ),));
    static SERVER: StaticCell<PineTimeServer<'static>> = StaticCell::new();
    let server = SERVER.init(gatt);

    spawner.must_spawn(ble_task(runner));
    spawner.must_spawn(advertise_task(stack, peripheral, server, dfu_config, battery));
}

#[embassy_executor::task]
async fn ble_task(mut runner: Runner<'static, NrfController>) {
    unwrap!(runner.run().await);
}

#[embassy_executor::task]
async fn advertise_task(
    stack: &'static Stack<'static, NrfController>,
    mut peripheral: Peripheral<'static, NrfController>,
    server: &'static PineTimeServer<'static>,
    mut dfu_config: DfuConfig<'static>,
    battery: &'static Battery<'static>,
) {
    const BAS: [u8; 2] = [0x0F, 0x18];
    const DFU: [u8; 2] = [0x59, 0xFE];
    let mut advertiser_data = [0; 31];
    unwrap!(AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::ServiceUuids16(&[Uuid::Uuid16(BAS), Uuid::Uuid16(DFU)]),
            AdStructure::CompleteLocalName(NAME.as_bytes()),
        ],
        &mut advertiser_data[..],
    ));
    loop {
        info!("[ble] advertising");
        let advertiser = unwrap!(
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
        match advertiser.accept().await {
            Ok(conn) => process(stack, conn, server, &mut dfu_config, battery).await,
            Err(e) => {
                warn!("Error advertising: {:?}", e);
            }
        }
    }
}

async fn process(
    stack: &'static Stack<'static, NrfController>,
    connection: Connection<'static>,
    server: &'static PineTimeServer<'_>,
    dfu_config: &mut DfuConfig<'static>,
    battery: &'static Battery<'static>,
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

    let mut dfu = dfu_config.dfu();
    let mut target = DfuTarget::new(dfu.size(), fw_info, hw_info);

    loop {
        let event = connection.next().await;
        match event {
            ConnectionEvent::Disconnected { reason } => {
                defmt::info!("[ble] disconnected: {:?}", reason);
                break;
            }
            ConnectionEvent::Gatt { data } => match data.process(server).await {
                Ok(Some(GattEvent::Read(event))) => {
                    let handle = event.handle();
                    if handle == server.battery.level.handle {
                        let value = battery.measure().await.max(100) as u8;
                        if let Err(_) = server.battery.level.set(server, &value) {
                            warn!("error updating battery level");
                        }
                    }
                    let reply = unwrap!(event.accept());
                    reply.send().await;
                }
                Ok(Some(GattEvent::Write(event))) => {
                    let handle = event.handle();
                    let reply = unwrap!(event.accept());
                    let result = server.handle(&mut target, &mut dfu, &connection, handle).await;
                    reply.send().await;

                    if let Some(DfuStatus::DoneReset) = result {
                        warn!("DFU done! Supposed to reset!");
                        cortex_m::peripheral::SCB::sys_reset();
                    }
                }
                _ => {}
            },
        }
    }
}

#[embassy_executor::task]
async fn sync_time(stack: &'static Stack<'static, NrfController>, conn: Connection<'static>) {
    info!("[ble] synchronizing time");
    let client = unwrap!(GattClient::<_, 10, ATT_MTU>::new(stack, &conn).await);
    match select(
        client.task(),
        with_timeout(Duration::from_secs(8), async {
            let services = client.services_by_uuid(&Uuid::new_short(0x1805)).await?;
            if let Some(service) = services.first() {
                let c: Characteristic<u8> = client
                    .characteristic_by_uuid(&service, &Uuid::new_short(0x2a2b))
                    .await?;

                let mut data = [0; 10];
                client.read_characteristic(&c, &mut data[..]).await?;

                if let Some(time) = parse_time(data) {
                    crate::CLOCK.set(time);
                }
            }
            Ok::<(), BleHostError<nrf_sdc::Error>>(())
        }),
    )
    .await
    {
        Either::First(_) => panic!("[ble] gatt client exited prematurely"),
        Either::Second(Ok(_)) => {
            info!("[ble] time sync completed");
        }
        Either::Second(Err(e)) => {
            warn!("[ble] time sync error: {:?}", e);
        }
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
