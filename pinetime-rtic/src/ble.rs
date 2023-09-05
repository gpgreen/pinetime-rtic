//! ble.rs code for interfacing with the BLE stack

use crate::app;
use nrf52832_hal as hal;
use rtic::Mutex;
use rtt_target::rprintln;
use rubble::{
    config::Config,
    gatt::BatteryServiceAttrs,
    l2cap::{BleChannelMap, L2CAPState},
    link::ad_structure::AdStructure,
    link::queue::{PacketQueue, SimpleQueue},
    link::{LinkLayer, Responder},
    security::NoSecurity,
    time::{Duration as RubbleDuration, Timer},
};
use rubble_nrf5x::radio::{BleRadio, PacketBuffer};
use rubble_nrf5x::timer::BleTimer;
use rubble_nrf5x::utils::get_device_address;

/// Configuration type for BLE stack
pub struct AppConfig {}

impl Config for AppConfig {
    type Timer = BleTimer<hal::pac::TIMER2>;
    type Transmitter = BleRadio;
    type ChannelMapper = BleChannelMap<BatteryServiceAttrs, NoSecurity>;
    type PacketQueue = &'static mut SimpleQueue;
}

/// Component container used in tasks for BLE
pub struct BleStack {
    /// The BleRadio component of the stack
    pub radio: BleRadio,
    /// The LinkLayer component of the stack
    pub ble_ll: LinkLayer<AppConfig>,
}

/// initialize the BLE stack
pub fn ble_initialize(
    radio_pac: hal::pac::RADIO,
    ficr: hal::pac::FICR,
    tx_buf: &'static mut PacketBuffer,
    rx_buf: &'static mut PacketBuffer,
    tx_queue: &'static mut SimpleQueue,
    rx_queue: &'static mut SimpleQueue,
    ble_timer: BleTimer<hal::pac::TIMER2>,
) -> (BleStack, Responder<AppConfig>) {
    // Get bluetooth device address
    let device_address = get_device_address();
    rprintln!("Bluetooth device address: {:?}", device_address);

    // Initialize radio
    let mut radio = BleRadio::new(radio_pac, &ficr, tx_buf, rx_buf);

    // Create bluetooth TX/RX queues
    let (tx, tx_cons) = tx_queue.split();
    let (rx_prod, rx) = rx_queue.split();

    // Create the actual BLE stack objects
    let mut ble_ll = LinkLayer::<AppConfig>::new(device_address, ble_timer);
    let ble_r = Responder::<AppConfig>::new(
        tx,
        rx,
        L2CAPState::new(BleChannelMap::with_attributes(BatteryServiceAttrs::new())),
    );

    // Send advertisement and set up regular interrupt
    let next_update = ble_ll
        .start_advertise(
            RubbleDuration::from_millis(200),
            &[AdStructure::CompleteLocalName("Rusty PineTime")],
            &mut radio,
            tx_cons,
            rx_prod,
        )
        .unwrap();
    ble_ll.timer().configure_interrupt(next_update);
    (BleStack { radio, ble_ll }, ble_r)
}

/// Hook up the RADIO interrupt to the Rubble BLE stack.
pub fn radio_interrupt(mut cx: app::radio_interrupt::Context) {
    cx.shared.blestack.lock(|blestack| {
        let n = blestack.ble_ll.timer().now();
        if let Some(cmd) = blestack.radio.recv_interrupt(n, &mut blestack.ble_ll) {
            blestack.radio.configure_receiver(cmd.radio);
            blestack.ble_ll.timer().configure_interrupt(cmd.next_update);

            if cmd.queued_work {
                // If there's any lower-priority work to be done, ensure that happens.
                // If we fail to spawn the task, it's already scheduled.
                crate::app::ble_worker::spawn().ok();
            }
        }
    });
}

/// Hook up the TIMER2 interrupt to the Rubble BLE stack.
pub fn timer2_interrupt(mut cx: app::timer2_interrupt::Context) {
    cx.shared.blestack.lock(|blestack| {
        let timer = blestack.ble_ll.timer();
        if timer.is_interrupt_pending() {
            timer.clear_interrupt();

            let cmd = blestack.ble_ll.update_timer(&mut blestack.radio);
            blestack.radio.configure_receiver(cmd.radio);

            blestack.ble_ll.timer().configure_interrupt(cmd.next_update);

            if cmd.queued_work {
                // If there's any lower-priority work to be done, ensure that happens.
                // If we fail to spawn the task, it's already scheduled.
                crate::app::ble_worker::spawn().ok();
            }
        }
    });
}

/// low priority task to drain the queue
pub fn ble_worker(cx: app::ble_worker::Context) {
    // Fully drain the packet queue
    while cx.local.ble_r.has_work() {
        cx.local.ble_r.process_one().unwrap();
    }
}
