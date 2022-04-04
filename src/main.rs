//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

mod direction;
mod game;
mod input;
mod point;
mod snake;

use crate::direction::Direction;
use crate::game::*;
use crate::input::*;
use bsp::hal::gpio::bank0::{Gpio0, Gpio16, Gpio17};
use bsp::pac::SPI0;
use core::convert::Infallible;
use cortex_m::delay::Delay;
use cortex_m::prelude::*;
use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use display_interface_spi::SPIInterface;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::primitives::{Circle, PrimitiveStyle, Rectangle};
use embedded_graphics::{image::*, prelude::*};
use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};
use embedded_time::duration::Extensions;
use embedded_time::fixed_point::FixedPoint;
use embedded_time::rate::Extensions as RateExtensions;
use panic_probe as _;
use st7789::{Orientation, ST7789};
// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock as HalClock},
    gpio::{
        pin::bank0::{Gpio12, Gpio13, Gpio14, Gpio15},
        {FunctionSpi, Pin, PullUpInput},
    },
    pac,
    sio::Sio,
    spi::Spi,
    watchdog::Watchdog,
};
// use rp_pico::hal::gpio::Pin;

const ORIGIN: embedded_graphics::geometry::Point = Point::new(40, 52);
const BLOCK_SIZE: embedded_graphics::geometry::Point = Point::new(10, 10);

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
    Image::new(
        &ImageRawLE::new(include_bytes!("../ferris.raw"), 86),
        Point::new(120, 67),
    )
    .draw(&mut display)
    .unwrap();

    // endregion

    let game_timer = bsp::hal::Timer::new(pac.TIMER, &mut pac.RESETS);

    // region Setup Outputs
    //High == On
    let mut led_pin = pins.led.into_push_pull_output();

    // High == off
    let mut r_pin = pins.gpio6.into_push_pull_output();
    let mut g_pin = pins.gpio7.into_push_pull_output();
    let mut b_pin = pins.gpio8.into_push_pull_output();

    r_pin.set_high().unwrap();
    g_pin.set_high().unwrap();
    b_pin.set_high().unwrap();
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

    let mut game = Game::new(23, 13, 0);

    state.delay.delay_ms(1500);
    display.clear(Rgb565::BLACK).unwrap();

    //draw boarder
    Rectangle::new(ORIGIN + Point::new(2, 2), Size::new(23 * 10, 13 * 10))
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 2))
        .draw(&mut display)
        .unwrap();

    loop {
        let mut input_timer = game_timer.count_down();
        input_timer.start(750.milliseconds());

        let mut game_tick = game_timer.count_down();
        game_tick.start(750.milliseconds());

        let command = match accept_input(&mut state, &mut input_timer).unwrap() {
            InputEvent::DownA => Command::Turn(Direction::Left),
            InputEvent::DownB => Command::Turn(Direction::Down),
            InputEvent::DownX => Command::Turn(Direction::Up),
            InputEvent::DownY => Command::Turn(Direction::Right),
            _ => Command::Turn(game.snake.get_direction()),
        };

        game.tick(command);

        let _ = nb::block!(game_tick.wait());

        if game.done {
            loop {
                r_pin.toggle().unwrap();
                g_pin.toggle().unwrap();
                b_pin.toggle().unwrap();
                state.delay.delay_ms(500);
            }
        } else {
            render_game(&game, &mut display).unwrap();
            led_pin.toggle().unwrap();
        }
    }
}

fn render_game(
    game: &Game,
    display: &mut ST7789<
        SPIInterface<
            bsp::hal::Spi<bsp::hal::spi::Enabled, SPI0, 8_u8>,
            bsp::hal::gpio::Pin<Gpio16, bsp::hal::gpio::Output<bsp::hal::gpio::PushPull>>,
            bsp::hal::gpio::Pin<Gpio17, bsp::hal::gpio::Output<bsp::hal::gpio::PushPull>>,
        >,
        bsp::hal::gpio::Pin<Gpio0, bsp::hal::gpio::Output<bsp::hal::gpio::PushPull>>,
    >,
) -> Result<(), GameError> {

    // display.clear(Rgb565::BLACK)?;
    let first = ORIGIN + (*game.snake.get_body_points().iter().next().unwrap() * BLOCK_SIZE);
    Circle::new(first, 10)
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::BLACK, 1))
        .draw(display)?;

    

    let last = ORIGIN + (*game.snake.get_body_points().iter().last().unwrap() * BLOCK_SIZE);
    

    Rectangle::new(Point::new(last.x + 10, last.y), Size::new(10, 10))
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::BLACK, 1))
        .draw(display)
        .unwrap();
    Rectangle::new(Point::new(last.x - 10, last.y), Size::new(10, 10))
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::BLACK, 1))
        .draw(display)
        .unwrap();
    Rectangle::new(Point::new(last.x, last.y + 10), Size::new(10, 10))
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::BLACK, 1))
        .draw(display)
        .unwrap();
    Rectangle::new(Point::new(last.x, last.y - 10), Size::new(10, 10))
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::BLACK, 1))
        .draw(display)
        .unwrap();
    
    match game.food {
        Some(f) => Circle::new( ORIGIN + (f * BLOCK_SIZE), 10)
                                .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1))
                                .draw(display)?,
        None => {}
    }

    game.snake.get_body_points()
        .iter()
        .map(|p| ORIGIN + (*p * BLOCK_SIZE))
        .map(|p| Rectangle::new(p, Size::new(10, 10)))
        .for_each(|c| c.into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1)).draw(display).unwrap());


    // Image::new(&ImageRawLE::new(include_bytes!("../ferris.raw"), 86), Point::new(120, 67))
    //     .draw(display)?;

    Ok(())
}

#[derive(Debug)]
pub enum GameError {
    InputPin,
    Infallible, // God is dead
    UnableToQueue(InputEvent),
    DisplayError,
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

impl<PinE> From<st7789::Error<PinE>> for GameError {
    fn from(err: st7789::Error<PinE>) -> GameError {
        GameError::DisplayError
    }
}
