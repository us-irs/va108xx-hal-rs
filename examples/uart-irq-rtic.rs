//! More complex UART application
//!
//! Uses the IRQ capabilities of the VA10820 peripheral and the RTIC framework to poll the UART in
//! a non-blocking way. You can send variably sized strings to the VA10820 which will be echoed
//! back to the sender.
//!
//! This script was tested with an Arduino Due. You can find the test script in the
//! [`/test/DueSerialTest`](https://egit.irs.uni-stuttgart.de/rust/va108xx-hal/src/branch/main/test/DueSerialTest)
//! folder.
#![no_main]
#![no_std]

#[rtic::app(device = pac, dispatchers = [OC4])]
mod app {
    use core::fmt::Write;
    use panic_rtt_target as _;
    use rtt_target::{rprintln, rtt_init_default, set_print_channel};
    use va108xx_hal::{
        gpio::PinsB,
        pac,
        prelude::*,
        uart::{self, IrqCfg, IrqResult, UartWithIrqBase},
    };

    #[local]
    struct Local {}

    #[shared]
    struct Shared {
        irq_uart: UartWithIrqBase<pac::UARTB>,
        rx_buf: [u8; 64],
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let channels = rtt_init_default!();
        set_print_channel(channels.up.0);
        rprintln!("-- VA108xx UART IRQ example application--");
        let mut dp = ctx.device;
        let gpiob = PinsB::new(&mut dp.SYSCONFIG, Some(dp.IOCONFIG), dp.PORTB);
        let tx = gpiob.pb21.into_funsel_1();
        let rx = gpiob.pb20.into_funsel_1();

        let irq_cfg = IrqCfg::new(pac::interrupt::OC3, true, true);
        let (mut irq_uart, _) = uart::Uart::uartb(
            dp.UARTB,
            (tx, rx),
            115200.bps(),
            &mut dp.SYSCONFIG,
            50.mhz(),
        )
        .into_uart_with_irq(irq_cfg, Some(&mut dp.SYSCONFIG), Some(&mut dp.IRQSEL))
        .downgrade();
        irq_uart
            .read_fixed_len_using_irq(64, true)
            .expect("Read initialization failed");
        let rx_buf: [u8; 64] = [0; 64];
        (Shared { irq_uart, rx_buf }, Local {}, init::Monotonics())
    }

    // `shared` cannot be accessed from this context
    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {}
    }

    #[task(
        binds = OC3,
        shared = [irq_uart, rx_buf],
        local = [cnt: u32 = 0, result: IrqResult = IrqResult::new()],
        priority = 4
    )]
    fn reception_task(cx: reception_task::Context) {
        let result = cx.local.result;
        let cnt: &mut u32 = cx.local.cnt;
        let irq_uart = cx.shared.irq_uart;
        let rx_buf = cx.shared.rx_buf;
        let (completed, end_idx) = (irq_uart, rx_buf).lock(|irq_uart, rx_buf| {
            match irq_uart.irq_handler(result, rx_buf) {
                Ok(_) => {
                    if result.complete() {
                        // Initiate next transfer immediately
                        irq_uart
                            .read_fixed_len_using_irq(64, true)
                            .expect("Read operation init failed");

                        let mut end_idx = 0;
                        for idx in 0..rx_buf.len() {
                            if (rx_buf[idx] as char) == '\n' {
                                end_idx = idx;
                                break;
                            }
                        }
                        (true, end_idx)
                    } else {
                        (false, 0)
                    }
                }
                Err(e) => {
                    rprintln!("Reception Error {:?}", e);
                    (false, 0)
                }
            }
        });
        if completed {
            rprintln!("Counter: {}", cnt);
            reply_handler::spawn(result.bytes_read, end_idx, result.timeout()).unwrap();
        }
        *cnt += 1;
    }

    #[task(shared = [irq_uart, rx_buf], priority = 3)]
    fn reply_handler(cx: reply_handler::Context, bytes_read: usize, end_idx: usize, timeout: bool) {
        let irq_uart = cx.shared.irq_uart;
        let rx_buf = cx.shared.rx_buf;
        (irq_uart, rx_buf).lock(|irq_uart, rx_buf| {
            rprintln!("Reception success, {} bytes read", bytes_read);
            if timeout {
                rprintln!("Timeout occured");
            }
            let string = core::str::from_utf8(&rx_buf[0..end_idx]).expect("Invalid string format");
            rprintln!("Read string: {}", string);
            writeln!(irq_uart.uart, "{}", string).expect("Sending reply failed");
        });
    }
}
