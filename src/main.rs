//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]
#![allow(unused_imports)]
#![allow(dead_code)]

// use core::pin::Pin;
use core::convert::Infallible;
use bsp::hal::clocks::ClockSource;
use bsp::hal::timer::CountDown;
use cortex_m_rt::entry;
use cortex_m::prelude::*;
use cortex_m::delay::Delay;
use defmt::*;
use defmt_rtt as _;
use display_interface_spi::SPIInterface;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::{image::*, prelude::*};
use embedded_hal::digital::v2::{InputPin, OutputPin, ToggleableOutputPin};
use embedded_time::fixed_point::FixedPoint;
use embedded_time::duration::Milliseconds;
use embedded_time::duration::Extensions;
use embedded_time::rate::Extensions as RateExtensions;
use embedded_time::clock::Clock;
use embedded_time::Timer as EmbeddedTimer;
use panic_probe as _;
use st7789::{Orientation, TearingEffect, ST7789};
use heapless::spsc::Queue;
// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock as HalClock},
    timer::Timer,
    pac,
    sio::Sio,
    watchdog::Watchdog,
};
// use rp_pico::hal::gpio::Pin;
use bsp::hal::gpio::pin::bank0::{Gpio12, Gpio13, Gpio14, Gpio15};
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

    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut delay = Delay::new(core.SYST, clocks.system_clock.freq().integer());

    // These are implicitly used by the spi driver if they are in the correct mode
    let _spi_sclk = pins.gpio18.into_mode::<FunctionSpi>();
    let _spi_tx = pins.gpio19.into_mode::<FunctionSpi>();
    let spi = Spi::<_, _, 8>::new(pac.SPI0);

    let dc = pins.gpio16.into_push_pull_output();
    let cs = pins.gpio17.into_push_pull_output();
    // let mut bl = pins.gpio21.into_push_pull_output();

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
    display.clear(Rgb565::BLACK).unwrap();

    // Draw Loading Screen -- Ferris
    Image::new(&ImageRawLE::new(include_bytes!("../ferris.raw"), 86), Point::new(120, 67))
        .draw(&mut display)
        .unwrap();
    
    delay.delay_ms(250);

    // endregion

    let game_timer = bsp::hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    
    // region Setup Outputs
    //High == On
    let mut led_pin = pins.led.into_push_pull_output();

    // High == off
    let mut r_pin = pins.gpio6.into_push_pull_output();
    // let mut g_pin = pins.gpio7.into_push_pull_output();
    // let mut b_pin = pins.gpio8.into_push_pull_output();

    // endregion

    // region Setup Inputs

    // Low == pressed
    let a_btn: Pin<Gpio12, PullUpInput> = pins.gpio12.into_mode();
    let b_btn: Pin<Gpio13, PullUpInput> = pins.gpio13.into_mode();
    let x_btn: Pin<Gpio14, PullUpInput> = pins.gpio14.into_mode();
    let y_btn: Pin<Gpio15, PullUpInput> = pins.gpio15.into_mode();
    // endregion


    let mut state = Inputs {
        input_state: InputState {
                a_active: false,
                b_active: false,
                x_active: false,
                y_active: false,
            },
        a_btn: a_btn,
        b_btn: b_btn,
        x_btn: x_btn,
        y_btn: y_btn,
        delay: delay,
    };
    // Handle Input Loop

    loop {
        let mut input_timer = game_timer.count_down();
        input_timer.start(500.milliseconds());

        let mut game_tick = game_timer.count_down();
        game_tick.start(500.milliseconds());
        
        match accept_input(&mut state, &mut input_timer).unwrap() {
            InputEvent::DownA => r_pin.toggle().unwrap(), 
            _ => {},
        }
            
        let _ = nb::block!(game_tick.wait());
        led_pin.toggle().unwrap();
    }
}

struct Inputs {
    input_state: InputState,
    a_btn: Pin<Gpio12, PullUpInput>,
    b_btn: Pin<Gpio13, PullUpInput>,
    x_btn: Pin<Gpio14, PullUpInput>,
    y_btn: Pin<Gpio15, PullUpInput>,
    delay: Delay,
}

#[derive(Clone,PartialEq, Eq)]
struct InputState {
    a_active: bool,
    b_active: bool,
    x_active: bool,
    y_active: bool,
}

fn accept_input(state: &mut Inputs, input_timer: &mut CountDown) -> Result<InputEvent, GameError> {
    while input_timer.wait().is_err() {
        
        if state.a_btn.is_low()? && !state.input_state.a_active {
            state.input_state.a_active = true;
            // input_buffer.enqueue(InputEvent::DownA)?
            return Ok(InputEvent::DownA);
        } else if state.a_btn.is_high()? && state.input_state.a_active {
            state.input_state.a_active = !state.input_state.a_active;
            // input_buffer.enqueue(InputEvent::UpA)?
            return Ok(InputEvent::UpA);
        }
        
        if state.b_btn.is_low()? && !state.input_state.b_active {
            state.input_state.b_active = !state.input_state.b_active;
            // input_buffer.enqueue(InputEvent::DownB)?
            return Ok(InputEvent::DownB);

        } else if state.b_btn.is_high()? && state.input_state.b_active {
            state.input_state.b_active = !state.input_state.b_active;
            // input_buffer.enqueue(InputEvent::UpB)?
            return Ok(InputEvent::UpB);
        }
        
        if state.x_btn.is_low()? && !state.input_state.x_active {
            state.input_state.x_active = !state.input_state.x_active;
            // input_buffer.enqueue(InputEvent::DownX)?
            return Ok(InputEvent::DownX);
        } else if state.x_btn.is_high()? && state.input_state.x_active {
            state.input_state.x_active = !state.input_state.x_active;
            // input_buffer.enqueue(InputEvent::UpX)?
            return Ok(InputEvent::UpX);
        }
        
        if state.y_btn.is_low()? && !state.input_state.y_active {
            state.input_state.y_active = !state.input_state.y_active;
            // input_buffer.enqueue(InputEvent::DownY)?
            return Ok(InputEvent::DownY);

        } else if state.y_btn.is_high()? && state.input_state.y_active {
            state.input_state.y_active = !state.input_state.y_active;
            // input_buffer.enqueue(InputEvent::UpY)?
            return Ok(InputEvent::UpY);
        }
        state.delay.delay_ms(1);
    }
    Ok(InputEvent::Noop)
}

#[derive(Debug)]
enum InputEvent {
    UpA,
    DownA,
    UpB,
    DownB,
    UpX,
    DownX,
    UpY,
    DownY,
    Noop,
} 

#[derive(Debug)]
enum GameError {   
    InputPin,
    Infallible, // God is dead
    UnableToQueue(InputEvent)
}

impl From<Infallible> for GameError {
    fn from(_: Infallible) -> GameError {
        GameError::Infallible
    }
}

impl From<InputEvent> for GameError {
    fn from(err: InputEvent) -> GameError {
        GameError::UnableToQueue(err)
    }
}
