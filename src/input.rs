use crate::GameError;
use cortex_m::{delay::Delay, prelude::_embedded_hal_timer_CountDown};
use embedded_hal::digital::v2::InputPin;
use rp_pico::hal::{
    gpio::{bank0::*, Pin, PullUpInput},
    timer::CountDown,
};

pub struct Inputs {
    pub input_state: InputState,
    pub a_btn: Pin<Gpio12, PullUpInput>,
    pub b_btn: Pin<Gpio13, PullUpInput>,
    pub x_btn: Pin<Gpio14, PullUpInput>,
    pub y_btn: Pin<Gpio15, PullUpInput>,
    pub delay: Delay,
}

#[derive(Clone, PartialEq, Eq)]
pub struct InputState {
    pub a_active: bool,
    pub b_active: bool,
    pub x_active: bool,
    pub y_active: bool,
}

pub fn accept_input(
    state: &mut Inputs,
    input_timer: &mut CountDown,
) -> Result<InputEvent, GameError> {
    while input_timer.wait().is_err() {
        if state.a_btn.is_low()? && !state.input_state.a_active {
            state.input_state.a_active = !state.input_state.a_active;
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
pub enum InputEvent {
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
