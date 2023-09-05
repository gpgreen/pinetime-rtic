//! pinetime-rtic main.rs

#![deny(unsafe_code)]
#![deny(warnings)]
#![deny(missing_docs)]
#![no_main]
#![cfg_attr(not(test), no_std)]

// Panic handler
#[cfg(not(test))]
use panic_rtt_target as _;

// device hal
use nrf52832_hal as hal;

mod backlight;
mod battery;
mod ble;
mod button;
mod delay;
mod monotonic_nrf52;

#[rtic::app(device = crate::hal::pac, peripherals = true, dispatchers = [SWI0_EGU0, SWI1_EGU1, SWI2_EGU2, SWI3_EGU3, SWI4_EGU4, SWI5_EGU5])]
mod app {

    use super::backlight::Backlight;
    use super::battery::BatteryStatus;
    use super::button::Button;
    use crate::ble::{
        ble_initialize, ble_worker, radio_interrupt, timer2_interrupt, AppConfig, BleStack,
    };
    use crate::hal::{
        gpio::{p0, Level, Output, PushPull},
        pac::SPIM1,
        prelude::*,
        spim, Spim,
    };
    use display_interface_spi::SPIInterface;
    use embedded_graphics::{
        image::{Image, ImageRawLE},
        mono_font::{ascii::FONT_10X20, MonoTextStyle},
        pixelcolor::Rgb565,
        prelude::*,
        primitives::{rectangle::Rectangle, PrimitiveStyleBuilder},
        text::Text,
    };
    use fugit::ExtU32;
    use mipidsi::{models::ST7789, Builder};
    use numtoa::NumToA;
    use rtt_target::{rprintln, rtt_init_print};
    use rubble::link::{queue::SimpleQueue, Responder, MIN_PDU_BUF};
    use rubble_nrf5x::{radio::PacketBuffer, timer::BleTimer};

    #[monotonic(binds = TIMER1, default = true)]
    type MyMono = crate::monotonic_nrf52::MonoTimer<crate::hal::pac::TIMER1>;

    const LCD_W: u16 = 240;
    const LCD_H: u16 = 240;

    const FERRIS_W: u16 = 86;
    const FERRIS_H: u16 = 64;

    const MARGIN: u16 = 10;

    #[shared]
    struct Shared {
        // LCD
        lcd: mipidsi::Display<
            display_interface_spi::SPIInterface<
                Spim<SPIM1>,
                p0::P0_18<Output<PushPull>>,
                p0::P0_25<Output<PushPull>>,
            >,
            ST7789,
            p0::P0_26<Output<PushPull>>,
        >,

        backlight: Backlight,

        // Battery
        battery: BatteryStatus,

        // BLE
        blestack: BleStack,
    }

    #[local]
    struct Local {
        // BLE
        ble_r: Responder<AppConfig>,

        // Button
        button: Button,

        // Counter resources
        counter: usize,

        // Ferris resources
        ferris: ImageRawLE<'static, Rgb565>,
        ferris_x_offset: i32,
        ferris_y_offset: i32,
        ferris_step_size: i32,
    }

    /// initialization task
    #[init(local = [ble_tx_buf: PacketBuffer = [0; MIN_PDU_BUF],
                    ble_rx_buf: PacketBuffer = [0; MIN_PDU_BUF],
                    tx_queue: SimpleQueue = SimpleQueue::new(),
    rx_queue: SimpleQueue = SimpleQueue::new()])]
    fn init(mut cx: init::Context) -> (Shared, Local, init::Monotonics) {
        // Destructure device peripherals
        let crate::hal::pac::Peripherals {
            CLOCK,
            FICR,
            P0,
            RADIO,
            SAADC,
            SPIM1,
            TIMER0,
            TIMER1,
            TIMER2,
            ..
        } = cx.device;

        // Init RTT
        rtt_init_print!();
        rprintln!("Initializing…");

        // Set up clocks. On reset, the high frequency clock is already used,
        // but we also need to switch to the external HF oscillator. This is
        // needed for Bluetooth to work.
        let _clocks = crate::hal::clocks::Clocks::new(CLOCK).enable_ext_hfosc();

        // Set up delay provider on TIMER0
        let mut delay = crate::delay::TimerDelay::new(TIMER0);

        // Initialize monotonic timer on TIMER1 (for RTIC)
        let mono = crate::monotonic_nrf52::MonoTimer::new(TIMER1);

        // Set up GPIO peripheral
        let gpio = p0::Parts::new(P0);

        // Enable backlight
        let backlight = Backlight::init(
            gpio.p0_14.into_push_pull_output(Level::High).degrade(),
            gpio.p0_22.into_push_pull_output(Level::High).degrade(),
            gpio.p0_23.into_push_pull_output(Level::High).degrade(),
            1,
        );

        // Battery status
        let battery = BatteryStatus::init(
            gpio.p0_12.into_floating_input(),
            gpio.p0_31.into_floating_input(),
            SAADC,
        );

        // Enable button
        let button = Button::init(
            gpio.p0_15.into_push_pull_output(Level::High).degrade(),
            gpio.p0_13.into_floating_input().degrade(),
        );

        // fire up the BLE Stack
        let (blestack, ble_r) = ble_initialize(
            RADIO,
            FICR,
            cx.local.ble_tx_buf,
            cx.local.ble_rx_buf,
            cx.local.tx_queue,
            cx.local.rx_queue,
            BleTimer::init(TIMER2), // Initialize BLE timer on TIMER2
        );

        // Set up SPI pins
        let spi_clk = gpio.p0_02.into_push_pull_output(Level::Low).degrade();
        let spi_mosi = gpio.p0_03.into_push_pull_output(Level::Low).degrade();
        let spi_miso = gpio.p0_04.into_floating_input().degrade();
        let spi_pins = spim::Pins {
            sck: spi_clk,
            miso: Some(spi_miso),
            mosi: Some(spi_mosi),
        };

        // Set up LCD pins
        // LCD_CS (P0.25): Chip select
        let mut lcd_cs = gpio.p0_25.into_push_pull_output(Level::Low);
        // LCD_RS (P0.18): Data/clock pin
        let lcd_dc = gpio.p0_18.into_push_pull_output(Level::Low);
        // LCD_RESET (P0.26): Display reset
        let lcd_rst = gpio.p0_26.into_push_pull_output(Level::Low);

        // Initialize SPI
        let spi = Spim::new(
            SPIM1,
            spi_pins,
            // Use SPI at 8MHz (the fastest clock available on the nRF52832)
            // because otherwise refreshing will be super slow.
            spim::Frequency::M8,
            // SPI must be used in mode 3. Mode 0 (the default) won't work.
            spim::MODE_3,
            0,
        );

        // Chip select must be held low while driving the display. It must be high
        // when using other SPI devices on the same bus (such as external flash
        // storage) so that the display controller won't respond to the wrong
        // commands.
        lcd_cs.set_low().unwrap();

        // display interface
        let di = SPIInterface::new(spi, lcd_dc, lcd_cs);
        // Initialize LCD
        let mut lcd = Builder::st7789(di)
            .with_display_size(LCD_W, LCD_H)
            .init(&mut delay, Some(lcd_rst))
            .unwrap();

        // clear the display
        lcd.clear(Rgb565::BLACK).ok();

        // Choose text style
        let text_style = MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE);

        // Draw text
        Text::new("PineTime", Point::new(10, 10), text_style)
            .draw(&mut lcd)
            .unwrap();

        // Load ferris image data
        let ferris = ImageRawLE::<Rgb565>::new(include_bytes!("../ferris.raw"), FERRIS_W.into());

        // Schedule tasks immediately
        write_counter::spawn().unwrap();
        write_ferris::spawn().unwrap();
        poll_button::spawn().unwrap();
        show_battery_status::spawn().unwrap();
        update_battery_status::spawn().unwrap();

        // set the ARM SLEEPONEXIT bit to go to sleep after handling interrupts
        cx.core.SCB.set_sleepdeep();

        (
            Shared {
                lcd,
                backlight,
                battery,
                blestack,
            },
            Local {
                ble_r,
                button,
                counter: 0,
                ferris,
                ferris_x_offset: 10,
                ferris_y_offset: 80,
                ferris_step_size: 2,
            },
            init::Monotonics(mono), // give the monotonic to RTIC
        )
    }

    /// idle task, allows cpu to sleep when this task is reached, runs at priority 0
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            // wait for interrupt is used instead of a busy-wait loop
            // to allow MCU to sleep between interrupts
            rtic::export::wfi()
        }
    }

    extern "Rust" {
        #[task(binds = RADIO, shared = [blestack], priority = 3)]
        fn radio_interrupt(_: radio_interrupt::Context);
        #[task(binds = TIMER2, shared = [blestack], priority = 3)]
        fn timer2_interrupt(_: timer2_interrupt::Context);
        #[task(local = [ble_r], priority = 2)]
        fn ble_worker(_: ble_worker::Context);
    }

    /// task to display ferris on the lcd
    #[task(shared = [lcd], local = [ferris, ferris_x_offset, ferris_y_offset, ferris_step_size])]
    fn write_ferris(mut cx: write_ferris::Context) {
        // Draw ferris
        let img = Image::new(
            cx.local.ferris,
            Point::new(*cx.local.ferris_x_offset, *cx.local.ferris_y_offset),
        );
        // Clean up behind ferris
        let backdrop_style = PrimitiveStyleBuilder::new()
            .fill_color(Rgb565::BLACK)
            .build();
        let (p1, p2) = if *cx.local.ferris_step_size > 0 {
            // Clean up to the left
            (
                Point::new(
                    *cx.local.ferris_x_offset - *cx.local.ferris_step_size,
                    *cx.local.ferris_y_offset,
                ),
                Size::new(*cx.local.ferris_step_size as u32, FERRIS_H as u32),
            )
        } else {
            // Clean up to the right
            (
                Point::new(
                    *cx.local.ferris_x_offset + FERRIS_W as i32,
                    *cx.local.ferris_y_offset,
                ),
                Size::new(*cx.local.ferris_step_size as u32, FERRIS_H as u32),
            )
        };
        // Reset step size
        if *cx.local.ferris_x_offset as u16 > LCD_W - FERRIS_W - MARGIN
            || (*cx.local.ferris_x_offset as u16) < MARGIN
        {
            *cx.local.ferris_step_size = -*cx.local.ferris_step_size;
        }
        *cx.local.ferris_x_offset += *cx.local.ferris_step_size;

        cx.shared.lcd.lock(|lcd| {
            img.draw(lcd).unwrap();
            Rectangle::new(p1, p2)
                .into_styled(backdrop_style)
                .draw(lcd)
                .unwrap();
        });

        // Re-schedule the timer interrupt
        write_ferris::spawn_after(40.millis()).unwrap();
    }

    /// task to increment and display counter
    #[task(shared = [lcd], local = [counter])]
    fn write_counter(mut cx: write_counter::Context) {
        rprintln!("Counter is {}", cx.local.counter);

        // Write counter to the display
        let mut buf = [0u8; 20];
        let text = cx.local.counter.numtoa_str(10, &mut buf);
        // Choose text style
        let text_style = MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE);

        cx.shared.lcd.lock(|lcd| {
            Text::new(text, Point::new(10, LCD_H as i32 - 10 - 16), text_style)
                .draw(lcd)
                .unwrap();
        });
        // Increment counter
        *cx.local.counter += 1;
        // Re-schedule the timer interrupt
        write_counter::spawn_after(1.secs()).unwrap();
    }

    /// task to poll the button, and dispatch button_pressed events
    #[task(local = [button])]
    fn poll_button(cx: poll_button::Context) {
        if cx.local.button.poll() {
            button_pressed::spawn().unwrap();
        }
        // Re-schedule the timer interrupt in 2ms
        poll_button::spawn_after(2.millis()).unwrap();
    }

    /// Called when button is pressed without bouncing for 12 (6 * 2) ms.
    #[task(shared = [backlight])]
    fn button_pressed(mut cx: button_pressed::Context) {
        cx.shared.backlight.lock(|backlight| {
            if backlight.get_brightness() < 7 {
                backlight.brighter();
            } else {
                backlight.off();
            }
        });
    }

    /// Fetch the battery status from the hardware. Update the text if
    /// something changed.
    #[task(shared = [battery])]
    fn update_battery_status(mut cx: update_battery_status::Context) {
        rprintln!("Update battery status");

        cx.shared.battery.lock(|battery| {
            let changed = battery.update();
            if changed {
                rprintln!("Battery status changed");
                show_battery_status::spawn().unwrap();
            }
        });
        // Re-schedule the timer interrupt in 1s
        update_battery_status::spawn_after(1.secs()).unwrap();
    }

    /// Show the battery status on the LCD.
    #[task(shared = [battery, lcd])]
    fn show_battery_status(mut cx: show_battery_status::Context) {
        let (voltage, charging) = cx
            .shared
            .battery
            .lock(|battery| (battery.voltage(), battery.is_charging()));

        rprintln!(
            "Battery status: {} ({})",
            voltage,
            if charging { "charging" } else { "discharging" },
        );

        // Show battery status in top right corner
        let mut buf = [0u8; 6];
        (voltage / 10).numtoa(10, &mut buf[0..1]);
        buf[1] = b'.';
        (voltage % 10).numtoa(10, &mut buf[2..3]);
        buf[3] = b'V';
        buf[4] = b'/';
        buf[5] = if charging { b'C' } else { b'D' };
        let status = core::str::from_utf8(&buf).unwrap();
        // Choose text style
        let text_style = MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE);
        cx.shared.lcd.lock(|lcd| {
            let text = Text::new(status, Point::zero(), text_style);
            let translation = Point::new(
                LCD_W as i32 - text.character_style.font.image.size().width as i32 - MARGIN as i32,
                MARGIN as i32,
            );
            text.translate(translation).draw(lcd).unwrap();
        });
    }
}
