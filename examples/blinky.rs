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
use va108xx_hal::{gpio::PinsA, pac, prelude::*, timer::set_up_ms_timer};

#[entry]
fn main() -> ! {
    let mut dp = pac::Peripherals::take().unwrap();
    let porta = PinsA::new(&mut dp.SYSCONFIG, Some(dp.IOCONFIG), dp.PORTA);
    let mut led1 = porta.pa10.into_push_pull_output();
    let mut led2 = porta.pa7.into_push_pull_output();
    let mut led3 = porta.pa6.into_push_pull_output();
    let mut delay = set_up_ms_timer(
        &mut dp.SYSCONFIG,
        &mut dp.IRQSEL,
        50.mhz().into(),
        dp.TIM0,
        pac::Interrupt::OC0,
    );
    for _ in 0..10 {
        led1.set_low().ok();
        led2.set_low().ok();
        led3.set_low().ok();
        delay.delay_ms(200_u16);
        led1.set_high().ok();
        led2.set_high().ok();
        led3.set_high().ok();
        delay.delay_ms(200_u16);
    }
    loop {
        led1.toggle().ok();
        delay.delay_ms(200_u16);
        led2.toggle().ok();
        delay.delay_ms(200_u16);
        led3.toggle().ok();
        delay.delay_ms(200_u16);
    }
}
