//! SPI example application
#![no_main]
#![no_std]

use core::cell::RefCell;

use cortex_m_rt::entry;
use embedded_hal::spi::{Mode, MODE_0};
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use va108xx_hal::{
    gpio::{PinsA, PinsB},
    pac::{self, SPIA, SPIB},
    prelude::*,
    spi::{self, NoneT, Spi, SpiBase, TransferConfig},
    timer::CountDownTimer,
};

#[derive(PartialEq, Debug)]
pub enum ExampleSelect {
    // Enter loopback mode. It is not necessary to tie MOSI/MISO together for this
    Loopback,
    // Send a test buffer and print everything received
    TestBuffer,
}

#[derive(PartialEq, Debug)]
pub enum SpiBusSelect {
    SpiAPortA,
    SpiAPortB,
    SpiBPortB,
}

const EXAMPLE_SEL: ExampleSelect = ExampleSelect::Loopback;
const SPI_BUS_SEL: SpiBusSelect = SpiBusSelect::SpiBPortB;
const SPI_SPEED_KHZ: u32 = 1000;
const SPI_MODE: Mode = MODE_0;
const BLOCKMODE: bool = false;

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("-- VA108xx SPI example application--");
    let mut dp = pac::Peripherals::take().unwrap();

    let spia_ref: RefCell<Option<SpiBase<SPIA, u8>>> = RefCell::new(None);
    let spib_ref: RefCell<Option<SpiBase<SPIB, u8>>> = RefCell::new(None);
    let mut spi_cfg = spi::SpiConfig::default();
    if EXAMPLE_SEL == ExampleSelect::Loopback {
        spi_cfg = spi_cfg.loopback(true)
    }

    match SPI_BUS_SEL {
        SpiBusSelect::SpiAPortA => {
            let pinsa = PinsA::new(&mut dp.SYSCONFIG, None, dp.PORTA);
            let (sck, mosi, miso) = (
                pinsa.pa31.into_funsel_1(),
                pinsa.pa30.into_funsel_1(),
                pinsa.pa29.into_funsel_1(),
            );
            spia_ref.borrow_mut().replace(
                Spi::spia::<NoneT>(
                    dp.SPIA,
                    (sck, miso, mosi),
                    50.mhz().into(),
                    spi_cfg,
                    Some(&mut dp.SYSCONFIG),
                    None,
                )
                .downgrade(),
            );
        }
        SpiBusSelect::SpiAPortB => {
            let pinsb = PinsB::new(&mut dp.SYSCONFIG, Some(dp.IOCONFIG), dp.PORTB);
            let (sck, mosi, miso) = (
                pinsb.pb9.into_funsel_2(),
                pinsb.pb8.into_funsel_2(),
                pinsb.pb7.into_funsel_2(),
            );
            spia_ref.borrow_mut().replace(
                Spi::spia::<NoneT>(
                    dp.SPIA,
                    (sck, miso, mosi),
                    50.mhz().into(),
                    spi_cfg,
                    Some(&mut dp.SYSCONFIG),
                    None,
                )
                .downgrade(),
            );
        }
        SpiBusSelect::SpiBPortB => {
            let pinsb = PinsB::new(&mut dp.SYSCONFIG, Some(dp.IOCONFIG), dp.PORTB);
            let (sck, mosi, miso) = (
                pinsb.pb5.into_funsel_1(),
                pinsb.pb4.into_funsel_1(),
                pinsb.pb3.into_funsel_1(),
            );
            spib_ref.borrow_mut().replace(
                Spi::spib::<NoneT>(
                    dp.SPIB,
                    (sck, miso, mosi),
                    50.mhz().into(),
                    spi_cfg,
                    Some(&mut dp.SYSCONFIG),
                    None,
                )
                .downgrade(),
            );
        }
    }
    let mut delay_tim = CountDownTimer::tim1(&mut dp.SYSCONFIG, 50.mhz().into(), dp.TIM1);
    loop {
        match SPI_BUS_SEL {
            SpiBusSelect::SpiAPortA | SpiBusSelect::SpiAPortB => {
                if let Some(ref mut spi) = *spia_ref.borrow_mut() {
                    let transfer_cfg: TransferConfig<NoneT> = TransferConfig {
                        spi_clk: SPI_SPEED_KHZ.khz().into(),
                        blockmode: BLOCKMODE,
                        hw_cs: None,
                        mode: SPI_MODE,
                        sod: false,
                    };
                    spi.cfg_transfer(&transfer_cfg);
                    if EXAMPLE_SEL == ExampleSelect::Loopback {
                        nb::block!(spi.send(0x42_u8)).unwrap();
                        let word = nb::block!(spi.read()).unwrap();
                        assert_eq!(word, 0x42);
                        delay_tim.delay_ms(500_u32);

                        let mut send_buf: [u8; 3] = [0x03, 0x02, 0x01];
                        let reply = spi.transfer(&mut send_buf).unwrap();
                        rprintln!("Received reply: {}, {}, {}", reply[0], reply[1], reply[2]);
                        assert_eq!(reply, &[0x03, 0x02, 0x01]);
                        delay_tim.delay_ms(500_u32);
                    } else {
                        let mut send_buf: [u8; 3] = [0x00, 0x01, 0x02];
                        let reply = spi.transfer(&mut send_buf).unwrap();
                        rprintln!("Received reply: {}, {}, {}", reply[0], reply[1], reply[2]);
                        delay_tim.delay_ms(1000_u32);
                    }
                }
            }
            SpiBusSelect::SpiBPortB => {
                if let Some(ref mut spi) = *spib_ref.borrow_mut() {
                    let transfer_cfg: TransferConfig<NoneT> = TransferConfig {
                        spi_clk: SPI_SPEED_KHZ.khz().into(),
                        blockmode: BLOCKMODE,
                        hw_cs: None,
                        mode: SPI_MODE,
                        sod: false,
                    };
                    spi.cfg_transfer(&transfer_cfg);
                    if EXAMPLE_SEL == ExampleSelect::Loopback {
                        nb::block!(spi.send(0x42_u8)).unwrap();
                        let word = nb::block!(spi.read()).unwrap();
                        assert_eq!(word, 0x42);
                        delay_tim.delay_ms(500_u32);

                        let mut send_buf: [u8; 3] = [0x03, 0x02, 0x01];
                        let reply = spi.transfer(&mut send_buf).unwrap();
                        rprintln!("Received reply: {}, {}, {}", reply[0], reply[1], reply[2]);
                        assert_eq!(reply, &[0x03, 0x02, 0x01]);
                        delay_tim.delay_ms(500_u32);
                    } else {
                        let mut send_buf: [u8; 3] = [0x00, 0x01, 0x02];
                        let reply = spi.transfer(&mut send_buf).unwrap();
                        rprintln!("Received reply: {}, {}, {}", reply[0], reply[1], reply[2]);
                        delay_tim.delay_ms(1000_u32);
                    }
                }
            }
        }
    }
}
