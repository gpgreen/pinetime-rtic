use debouncr::{debounce_6, Debouncer, Edge, Repeat6};
use embedded_hal::digital::v2::{InputPin, OutputPin, StatefulOutputPin};
use nrf52832_hal::gpio::{Floating, Input, Output, Pin, PushPull};
use rtt_target::rprintln;

/// Control the pushbutton.
///
/// The button is initially disabled. It can be enabled/disabled with a
/// pin. The button itself is monitored on another pin. The input is
/// debounced with a callback function.
pub struct Button {
    enable: Pin<Output<PushPull>>,
    button: Pin<Input<Floating>>,
    debouncer: Debouncer<u8, Repeat6>,
}

impl Button {
    /// Initialize the button
    pub fn init(enable: Pin<Output<PushPull>>, button: Pin<Input<Floating>>) -> Self {
        rprintln!("Initializing button");
        Self {
            enable,
            button,
            debouncer: debounce_6(),
        }
    }

    /// Enable or disable the button
    pub fn enable(&mut self, enabled: bool) {
        rprintln!("Button enable set to: {}", enabled);
        if enabled {
            // per the pinetime wiki, the button needs a short time to give good outputs
            // setting the enable button high 4 times in a row results in enough delay
            // that the input is stable
            for _i in 0..4 {
                self.enable.set_high().unwrap();
            }
        } else {
            self.enable.set_low().unwrap();
        }
    }

    /// Poll the button, when a button press is detected, return true, otherwise false
    /// returns false if button is not enabled
    pub fn poll(&mut self) -> bool {
        if self.enable.is_set_low().unwrap() {
            false
        } else {
            let pressed = self.button.is_high().unwrap();
            let edge = self.debouncer.update(pressed);
            if edge == Some(Edge::Rising) {
                true
            } else {
                false
            }
        }
    }
}
