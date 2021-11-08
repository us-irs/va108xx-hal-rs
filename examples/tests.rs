//! Test image
//!
//! It would be nice to use a test framework like defmt-test, but I have issues
//! with probe run and it would be better to make the RTT work first
#![no_main]
#![no_std]

use cortex_m_rt::entry;
use embedded_hal::digital::v2::{InputPin, OutputPin, StatefulOutputPin, ToggleableOutputPin};
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use va108xx_hal::gpio::{porta, portb, PinState};
use va108xx_hal::prelude::*;

#[allow(dead_code)]
#[derive(Debug)]
enum TestCase {
    // Tie PORTA[0] to PORTA[1] for these tests!
    TestBasic,
    TestPullup,
    TestPulldown,
    TestMask,
    Perid,
    // Tie PA0 to an oscilloscope and configure pulse detection
    Pulse,
    // Tie PA0, PA1 and PA3 to an oscilloscope
    Delay,
}

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("-- VA108xx Test Application --");
    let mut dp = va108xx::Peripherals::take().unwrap();
    let porta = dp.PORTA.split(&mut dp.SYSCONFIG).unwrap();
    let _portb = dp.PORTB.split(&mut dp.SYSCONFIG).unwrap();
    let mut led1 = porta
        .pa10
        .into_push_pull_output(&mut dp.IOCONFIG, &mut dp.PORTA);
    let test_case = TestCase::Delay;

    match test_case {
        TestCase::TestBasic
        | TestCase::TestPulldown
        | TestCase::TestPullup
        | TestCase::TestMask => {
            rprintln!(
                "Test case {:?}. Make sure to tie PORTA[0] to PORTA[1]",
                test_case
            );
        }
        _ => {
            rprintln!("Test case {:?}", test_case);
        }
    }
    match test_case {
        TestCase::TestBasic => {
            // Tie PORTA[0] to PORTA[1] for these tests!
            let mut out = porta
                .pa0
                .into_push_pull_output(&mut dp.IOCONFIG, &mut dp.PORTA)
                .enable_input(&mut dp.IOCONFIG, true);
            let input = porta
                .pa1
                .into_floating_input(&mut dp.IOCONFIG, &mut dp.PORTA);
            out.set_high().unwrap();
            assert!(out.is_set_high().unwrap());
            assert!(input.is_high().unwrap());
            out.set_low().unwrap();
            assert!(out.is_set_low().unwrap());
            assert!(input.is_low().unwrap());
        }
        TestCase::TestPullup => {
            // Tie PORTA[0] to PORTA[1] for these tests!
            let input = porta.pa1.into_pull_up_input(&mut dp.IOCONFIG);
            assert!(input.is_high().unwrap());
            let mut out = porta
                .pa0
                .into_push_pull_output(&mut dp.IOCONFIG, &mut dp.PORTA);
            out.set_low().unwrap();
            assert!(input.is_low().unwrap());
            out.set_high().unwrap();
            assert!(input.is_high().unwrap());
            out.into_floating_input(&mut dp.IOCONFIG, &mut dp.PORTA);
            assert!(input.is_high().unwrap());
        }
        TestCase::TestPulldown => {
            // Tie PORTA[0] to PORTA[1] for these tests!
            let input = porta
                .pa1
                .into_pull_down_input(&mut dp.IOCONFIG, &mut dp.PORTA);
            assert!(input.is_low().unwrap());
            let mut out = porta
                .pa0
                .into_push_pull_output(&mut dp.IOCONFIG, &mut dp.PORTA);
            out.set_low().unwrap();
            assert!(input.is_low().unwrap());
            out.set_high().unwrap();
            assert!(input.is_high().unwrap());
            out.into_floating_input(&mut dp.IOCONFIG, &mut dp.PORTA);
            assert!(input.is_low().unwrap());
        }
        TestCase::TestMask => {
            // Tie PORTA[0] to PORTA[1] for these tests!
            let input = porta
                .pa1
                .into_pull_down_input(&mut dp.IOCONFIG, &mut dp.PORTA)
                .clear_datamask(&mut dp.PORTA);
            assert!(!input.datamask(&dp.PORTA));
            let out = porta
                .pa0
                .into_push_pull_output(&mut dp.IOCONFIG, &mut dp.PORTA)
                .clear_datamask(&mut dp.PORTA);
            assert!(input.is_low_masked(&mut dp.PORTA).is_err());
            assert!(out.set_high_masked(&mut dp.PORTA).is_err());
        }
        TestCase::Perid => {
            assert_eq!(porta::get_perid(&dp.PORTA), 0x004007e1);
            assert_eq!(portb::get_perid(&dp.PORTB), 0x004007e1);
        }
        TestCase::Pulse => {
            let mut output_pulsed = porta
                .pa0
                .into_push_pull_output(&mut dp.IOCONFIG, &mut dp.PORTA)
                .pulse_mode(&mut dp.PORTA, true, PinState::Low);
            rprintln!("Pulsing high 10 times..");
            output_pulsed.set_low().unwrap();
            for _ in 0..10 {
                output_pulsed.set_high().unwrap();
                cortex_m::asm::delay(25_000_000);
            }
            let mut output_pulsed = output_pulsed.pulse_mode(&mut dp.PORTA, true, PinState::High);
            rprintln!("Pulsing low 10 times..");
            for _ in 0..10 {
                output_pulsed.set_low().unwrap();
                cortex_m::asm::delay(25_000_000);
            }
        }
        TestCase::Delay => {
            let mut out_0 = porta
                .pa0
                .into_push_pull_output(&mut dp.IOCONFIG, &mut dp.PORTA)
                .delay(&mut dp.PORTA, true, false);
            let mut out_1 = porta
                .pa1
                .into_push_pull_output(&mut dp.IOCONFIG, &mut dp.PORTA)
                .delay(&mut dp.PORTA, false, true);
            let mut out_2 = porta
                .pa3
                .into_push_pull_output(&mut dp.IOCONFIG, &mut dp.PORTA)
                .delay(&mut dp.PORTA, true, true);
            for _ in 0..20 {
                out_0.toggle().unwrap();
                out_1.toggle().unwrap();
                out_2.toggle().unwrap();
                cortex_m::asm::delay(25_000_000);
            }
        }
    }

    rprintln!("Test success");
    loop {
        led1.toggle().ok();
        cortex_m::asm::delay(25_000_000);
    }
}
