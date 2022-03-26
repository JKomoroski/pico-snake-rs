//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::{OutputPin, StatefulOutputPin};
use embedded_time::fixed_point::FixedPoint;
use panic_probe as _;

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
//use rp_pico::hal::gpio::Pin;
use rp_pico::Gp9Pwm4B;

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

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.led.into_push_pull_output();
    //let mut r_pin: Gp9Pwm4B = pins.gpio9.into_mode();
    let mut r_pin = pins.gpio6.into_push_pull_output();
    let mut g_pin = pins.gpio7.into_push_pull_output();
    let mut b_pin = pins.gpio8.into_push_pull_output();

    r_pin.set_low().unwrap();
    g_pin.set_high().unwrap();
    b_pin.set_high().unwrap();

    loop {
        if r_pin.is_set_low().unwrap() {

            g_pin.set_low().unwrap();
            r_pin.set_high().unwrap();

        } else if g_pin.is_set_low().unwrap() {

            b_pin.set_low().unwrap();
            g_pin.set_high().unwrap();

        } else if b_pin.is_set_low().unwrap() {

            r_pin.set_low().unwrap();
            b_pin.set_high().unwrap();

        }

        info!("on!");
        led_pin.set_high().unwrap();
        delay.delay_ms(250);
        info!("off!");
        led_pin.set_low().unwrap();
        delay.delay_ms(750);
    }
}

// End of file
