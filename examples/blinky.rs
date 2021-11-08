//! Simple blinky example
//!
//! Additional note on LEDs when using the REB1 development board:
//! Be not afraid: Pulling the GPIOs low makes the LEDs blink. See REB1
//! schematic for more details.
#![no_main]
#![no_std]

use cortex_m_rt::entry;
use embedded_hal::digital::v2::ToggleableOutputPin;
use panic_halt as _;
use va108xx_hal::{pac, prelude::*};

#[entry]
fn main() -> ! {
    let mut dp = pac::Peripherals::take().unwrap();
    let porta = dp.PORTA.split(&mut dp.SYSCONFIG).unwrap();
    let mut led1 = porta
        .pa10
        .into_push_pull_output(&mut dp.IOCONFIG, &mut dp.PORTA);
    let mut led2 = porta
        .pa7
        .into_push_pull_output(&mut dp.IOCONFIG, &mut dp.PORTA);
    let mut led3 = porta
        .pa6
        .into_push_pull_output(&mut dp.IOCONFIG, &mut dp.PORTA);
    for _ in 0..10 {
        led1.set_low().ok();
        led2.set_low().ok();
        led3.set_low().ok();
        cortex_m::asm::delay(5_000_000);
        led1.set_high().ok();
        led2.set_high().ok();
        led3.set_high().ok();
        cortex_m::asm::delay(5_000_000);
    }
    loop {
        led1.toggle().ok();
        cortex_m::asm::delay(5_000_000);
        led2.toggle().ok();
        cortex_m::asm::delay(5_000_000);
        led3.toggle().ok();
        cortex_m::asm::delay(5_000_000);
    }
}
