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
    spi::{self, Spi, SpiBase, TransferConfig},
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

const EXAMPLE_SEL: ExampleSelect = ExampleSelect::TestBuffer;
const SPI_BUS_SEL: SpiBusSelect = SpiBusSelect::SpiBPortB;
const SPI_SPEED_KHZ: u32 = 1000;
const SPI_MODE: Mode = MODE_0;
const BLOCKMODE: bool = true;

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("-- VA108xx SPI example application--");
    let mut dp = pac::Peripherals::take().unwrap();

    let spia_ref: RefCell<Option<SpiBase<SPIA, u8>>> = RefCell::new(None);
    let spib_ref: RefCell<Option<SpiBase<SPIB, u8>>> = RefCell::new(None);
    let pinsa = PinsA::new(&mut dp.SYSCONFIG, None, dp.PORTA);
    let pinsb = PinsB::new(&mut dp.SYSCONFIG, Some(dp.IOCONFIG), dp.PORTB);

    let mut spi_cfg = spi::SpiConfig::default();
    if EXAMPLE_SEL == ExampleSelect::Loopback {
        spi_cfg = spi_cfg.loopback(true)
    }

    // Set up the SPI peripheral
    match SPI_BUS_SEL {
        SpiBusSelect::SpiAPortA => {
            let (sck, mosi, miso) = (
                pinsa.pa31.into_funsel_1(),
                pinsa.pa30.into_funsel_1(),
                pinsa.pa29.into_funsel_1(),
            );
            spia_ref.borrow_mut().replace(
                Spi::spia(
                    dp.SPIA,
                    (sck, miso, mosi),
                    50.mhz(),
                    spi_cfg,
                    Some(&mut dp.SYSCONFIG),
                    None,
                )
                .downgrade(),
            );
        }
        SpiBusSelect::SpiAPortB => {
            let (sck, mosi, miso) = (
                pinsb.pb9.into_funsel_2(),
                pinsb.pb8.into_funsel_2(),
                pinsb.pb7.into_funsel_2(),
            );
            spia_ref.borrow_mut().replace(
                Spi::spia(
                    dp.SPIA,
                    (sck, miso, mosi),
                    50.mhz(),
                    spi_cfg,
                    Some(&mut dp.SYSCONFIG),
                    None,
                )
                .downgrade(),
            );
        }
        SpiBusSelect::SpiBPortB => {
            let (sck, mosi, miso) = (
                pinsb.pb5.into_funsel_1(),
                pinsb.pb4.into_funsel_1(),
                pinsb.pb3.into_funsel_1(),
            );
            spib_ref.borrow_mut().replace(
                Spi::spib(
                    dp.SPIB,
                    (sck, miso, mosi),
                    50.mhz(),
                    spi_cfg,
                    Some(&mut dp.SYSCONFIG),
                    None,
                )
                .downgrade(),
            );
        }
    }
    // Configure transfer specific properties here
    match SPI_BUS_SEL {
        SpiBusSelect::SpiAPortA | SpiBusSelect::SpiAPortB => {
            if let Some(ref mut spi) = *spia_ref.borrow_mut() {
                let transfer_cfg = TransferConfig::new_no_hw_cs(
                    SPI_SPEED_KHZ.khz().into(),
                    SPI_MODE,
                    BLOCKMODE,
                    false,
                );
                spi.cfg_transfer(&transfer_cfg);
            }
        }
        SpiBusSelect::SpiBPortB => {
            if let Some(ref mut spi) = *spib_ref.borrow_mut() {
                let hw_cs_pin = pinsb.pb2.into_funsel_1();
                let transfer_cfg = TransferConfig::new(
                    SPI_SPEED_KHZ.khz().into(),
                    SPI_MODE,
                    Some(hw_cs_pin),
                    BLOCKMODE,
                    false,
                );
                spi.cfg_transfer(&transfer_cfg);
            }
        }
    }

    // Application logic
    let mut delay_tim = CountDownTimer::new(&mut dp.SYSCONFIG, 50.mhz(), dp.TIM1);
    loop {
        match SPI_BUS_SEL {
            SpiBusSelect::SpiAPortA | SpiBusSelect::SpiAPortB => {
                if let Some(ref mut spi) = *spia_ref.borrow_mut() {
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
                        let mut send_buf: [u8; 3] = [0x01, 0x02, 0x03];
                        let reply = spi.transfer(&mut send_buf).unwrap();
                        rprintln!("Received reply: {}, {}, {}", reply[0], reply[1], reply[2]);
                        delay_tim.delay_ms(1000_u32);
                    }
                }
            }
            SpiBusSelect::SpiBPortB => {
                if let Some(ref mut spi) = *spib_ref.borrow_mut() {
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
                        let mut send_buf: [u8; 3] = [0x01, 0x02, 0x03];
                        let reply = spi.transfer(&mut send_buf).unwrap();
                        rprintln!("Received reply: {}, {}, {}", reply[0], reply[1], reply[2]);
                        delay_tim.delay_ms(1000_u32);
                    }
                }
            }
        }
    }
}
