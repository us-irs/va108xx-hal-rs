//! UART example application. Sends a test string over a UART and then enters
//! echo mode
#![no_main]
#![no_std]

#[rtic::app(device = pac)]
mod app {
    use core::fmt::Write;
    use panic_rtt_target as _;
    use rtt_target::{rprintln, rtt_init_print};
    use va108xx_hal::{gpio::PinsB, pac, prelude::*, uart};

    #[local]
    struct Local {}

    #[shared]
    struct Shared {}

    #[init]
    fn init(_ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        rtt_init_print!();
        rprintln!("-- VA108xx UART example application--");
    
        let mut dp = pac::Peripherals::take().unwrap();
    
        let gpiob = PinsB::new(&mut dp.SYSCONFIG, Some(dp.IOCONFIG), dp.PORTB);
        let tx = gpiob.pb21.into_funsel_1();
        let rx = gpiob.pb20.into_funsel_1();
    
        let uartb = uart::Uart::uartb(
            dp.UARTB,
            (tx, rx),
            115200.bps(),
            &mut dp.SYSCONFIG,
            50.mhz(),
        );
        let (mut tx, mut _rx) = uartb.split();
        writeln!(tx, "Hello World\r").unwrap();
        (Shared {}, Local {}, init::Monotonics())
    }

    // `shared` cannot be accessed from this context
    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {}
    }
}

// #[entry]
// fn main() -> ! {

//     loop {
//         // Echo what is received on the serial link.
//         match rx.read() {
//             Ok(recv) => {
//                 nb::block!(tx.write(recv)).expect("TX send error");
//             }
//             Err(nb::Error::WouldBlock) => (),
//             Err(nb::Error::Other(uart_error)) => {
//                 rprintln!("UART receive error {:?}", uart_error);
//             }
//         }
//     }
// }
