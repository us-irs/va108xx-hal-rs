//! UART example application. Sends a test string over a UART and then enters
//! echo mode
#![no_main]
#![no_std]

use core::fmt::Write;
use cortex_m_rt::entry;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use va108xx_hal::{pac, prelude::*, uart};

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("-- VA108xx UART test application--");

    let mut dp = pac::Peripherals::take().unwrap();

    let gpiob = dp.PORTB.split(&mut dp.SYSCONFIG).unwrap();
    let tx = gpiob.pb21.into_funsel_1(&mut dp.IOCONFIG);
    let rx = gpiob.pb20.into_funsel_1(&mut dp.IOCONFIG);

    let uartb = uart::Uart::uartb(
        dp.UARTB,
        (tx, rx),
        115200.bps(),
        &mut dp.SYSCONFIG,
        50.mhz().into(),
    );
    let (mut tx, mut rx) = uartb.split();
    writeln!(tx, "Hello World\r").unwrap();
    loop {
        // Echo what is received on the serial link.
        match rx.read() {
            Ok(recv) => {
                nb::block!(tx.write(recv)).expect("TX send error");
            }
            Err(nb::Error::WouldBlock) => (),
            Err(nb::Error::Other(uart_error)) => {
                rprintln!("UART receive error {:?}", uart_error);
            }
        }
    }
}
