//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]
#![allow(unused_imports)]
#![allow(dead_code)]

// use core::pin::Pin;
use core::ops::AddAssign;
use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use display_interface_spi::SPIInterface;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::{image::*, prelude::*};
use embedded_hal::digital::v2::{InputPin, OutputPin, ToggleableOutputPin};
use embedded_time::fixed_point::FixedPoint;
use embedded_time::rate::Extensions;
use panic_probe as _;
use st7789::{Orientation, TearingEffect, ST7789};
// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};
// use rp_pico::hal::gpio::Pin;
use bsp::hal::gpio::pin::bank0::{Gpio12, Gpio13};
use bsp::hal::gpio::OutputDriveStrength::TwoMilliAmps;
use bsp::hal::gpio::{FunctionSpi, Pin, PullUpInput};
use bsp::hal::spi::Spi;
use bsp::{Gp6Pwm3A, Gp7Pwm3B, Gp8Pwm4A};

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // region Setup Outputs
    //High == On
    // let mut led_pin = pins.led.into_push_pull_output();

    // High == off
    let mut r_pin = pins.gpio6.into_push_pull_output();
    let mut g_pin = pins.gpio7.into_push_pull_output();
    let mut b_pin = pins.gpio8.into_push_pull_output();

    r_pin.set_drive_strength(TwoMilliAmps);
    g_pin.set_drive_strength(TwoMilliAmps);
    b_pin.set_drive_strength(TwoMilliAmps);

    // These are implicitly used by the spi driver if they are in the correct mode
    let _spi_sclk = pins.gpio18.into_mode::<FunctionSpi>();
    let _spi_tx = pins.gpio19.into_mode::<FunctionSpi>();
    let spi = Spi::<_, _, 8>::new(pac.SPI0);

    let dc = pins.gpio16.into_push_pull_output();
    let cs = pins.gpio17.into_push_pull_output();
    let mut bl = pins.gpio21.into_push_pull_output();

    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        16_000_000u32.Hz(),
        &embedded_hal::spi::MODE_3,
    );

    // create driver
    let di = SPIInterface::new(spi, dc, cs);
    let mut display = ST7789::new(di, pins.gpio0.into_push_pull_output(), 240, 135);

    // initialize
    display.init(&mut delay).unwrap();
    // set default orientation
    display.set_orientation(Orientation::Landscape).unwrap();
    display.set_tearing_effect(TearingEffect::Off).unwrap();

    let raw_image_data = ImageRawLE::new(include_bytes!("../ferris.raw"), 86);
    let mut origin = Point::new(80, 80);
    let ferris = Image::new(&raw_image_data, origin);

    // draw image on black background
    display.clear(Rgb565::BLACK).unwrap();
    ferris.draw(&mut display).unwrap();

    // endregion
    // region Setup Inputs

    // Low == pressed
    let a_btn: Pin<Gpio12, PullUpInput> = pins.gpio12.into_pull_up_input();
    let b_btn: Pin<Gpio13, PullUpInput> = pins.gpio13.into_mode::<PullUpInput>();
    let x_btn = pins.gpio14.into_mode::<PullUpInput>();
    let y_btn = pins.gpio15.into_mode::<PullUpInput>();
    // endregion

    let mut state = InputState::new();

    // Handle Input Loop
    loop {
        for _ in 0..200 {
            if a_btn.is_low().unwrap() && !state.a_active {
                state.a_active = !state.a_active;
                r_pin.toggle().unwrap();
            } else if a_btn.is_high().unwrap() && state.a_active {
                state.a_active = !state.a_active;
            }
            if b_btn.is_low().unwrap() && !state.b_active {
                state.b_active = !state.b_active;
                g_pin.toggle().unwrap();
            } else if b_btn.is_high().unwrap() && state.b_active {
                state.b_active = !state.b_active;
            }
            if x_btn.is_low().unwrap() && !state.x_active {
                state.x_active = !state.x_active;
                b_pin.toggle().unwrap();
            } else if x_btn.is_high().unwrap() && state.x_active {
                state.x_active = !state.x_active;
            }
            if y_btn.is_low().unwrap() && !state.y_active {
                state.y_active = !state.y_active;
                r_pin.toggle().unwrap();
                g_pin.toggle().unwrap();
                b_pin.toggle().unwrap();
                bl.toggle().unwrap();
            } else if y_btn.is_high().unwrap() && state.y_active {
                state.y_active = !state.y_active;
            }
            // the following delay acts as a lazy man's debounce
            delay.delay_ms(1);
        }

        origin.add_assign(Point::new(1, 0));
        if origin.y > 135 {
            origin.y = 0;
        }

        if origin.x > 240 {
            origin.x = 0
        }

        let ferris = Image::new(&raw_image_data, origin);

        ferris.draw(&mut display).unwrap();
    }
}

struct InputState {
    a_active: bool,
    b_active: bool,
    x_active: bool,
    y_active: bool,
}

impl InputState {
    fn new() -> InputState {
        InputState {
            a_active: false,
            b_active: false,
            x_active: false,
            y_active: false,
        }
    }
}

// struct StatefulInput {
//     active: bool,
//     pin: Pin<Gpio12, PullUpInput>
// }

// struct InputState {
//     a_state: StatefulInput,
//     // b_active: bool,
//     // x_active: bool,
//     // y_active: bool,
// }

// impl InputState {
//     fn a_new_active(&self, current_state: bool) -> bool {
//         if self.a_state.pin.is_low().unwrap() && !self.a_state.active {
//             self.a_state.active = !self.a_state.active;
//             self.a_state.active
//         } else if a_btn.is_high().unwrap() && state.a_active {
//             state.a_active = !state.a_active;
//         }
//         self.a_state
//     }
// }

// End of file
