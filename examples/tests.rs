//! Test image
//!
//! It would be nice to use a test framework like defmt-test, but I have issues
//! with probe run and it would be better to make the RTT work first
#![no_main]
#![no_std]

use cortex_m_rt::entry;
use embedded_hal::digital::v2::{InputPin, OutputPin, ToggleableOutputPin};
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use va108xx_hal::{
    gpio::{PinState, PinsA, PinsB},
    pac::{self, interrupt},
    prelude::*,
    time::Hertz,
    timer::{default_ms_irq_handler, set_up_ms_timer, CountDownTimer, Delay},
};

#[allow(dead_code)]
#[derive(Debug)]
enum TestCase {
    // Tie PORTA[0] to PORTA[1] for these tests!
    TestBasic,
    TestPullup,
    TestPulldown,
    TestMask,
    // Tie PORTB[22] to PORTB[23] for this test
    PortB,
    Perid,
    // Tie PA0 to an oscilloscope and configure pulse detection
    Pulse,
    // Tie PA0, PA1 and PA3 to an oscilloscope
    DelayGpio,
    DelayMs,
}

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("-- VA108xx Test Application --");
    let mut dp = va108xx::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    let pinsa = PinsA::new(&mut dp.SYSCONFIG, None, dp.PORTA);
    let pinsb = PinsB::new(&mut dp.SYSCONFIG, Some(dp.IOCONFIG), dp.PORTB);
    let mut led1 = pinsa.pa10.into_push_pull_output();
    let test_case = TestCase::DelayGpio;

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
            let mut out = pinsa.pa0.into_readable_push_pull_output();
            let input = pinsa.pa1.into_floating_input();
            out.set_high().unwrap();
            assert!(input.is_high().unwrap());
            out.set_low().unwrap();
            assert!(input.is_low().unwrap());
        }
        TestCase::TestPullup => {
            // Tie PORTA[0] to PORTA[1] for these tests!
            let input = pinsa.pa1.into_pull_up_input();
            assert!(input.is_high().unwrap());
            let mut out = pinsa.pa0.into_readable_push_pull_output();
            out.set_low().unwrap();
            assert!(input.is_low().unwrap());
            out.set_high().unwrap();
            assert!(input.is_high().unwrap());
            out.into_floating_input();
            assert!(input.is_high().unwrap());
        }
        TestCase::TestPulldown => {
            // Tie PORTA[0] to PORTA[1] for these tests!
            let input = pinsa.pa1.into_pull_down_input();
            assert!(input.is_low().unwrap());
            let mut out = pinsa.pa0.into_push_pull_output();
            out.set_low().unwrap();
            assert!(input.is_low().unwrap());
            out.set_high().unwrap();
            assert!(input.is_high().unwrap());
            out.into_floating_input();
            assert!(input.is_low().unwrap());
        }
        TestCase::TestMask => {
            // Tie PORTA[0] to PORTA[1] for these tests!
            let input = pinsa.pa1.into_pull_down_input().clear_datamask();
            assert!(!input.datamask());
            let mut out = pinsa.pa0.into_push_pull_output().clear_datamask();
            assert!(input.is_low_masked().is_err());
            assert!(out.set_high_masked().is_err());
        }
        TestCase::PortB => {
            // Tie PORTB[22] to PORTB[23] for these tests!
            let mut out = pinsb.pb22.into_readable_push_pull_output();
            let input = pinsb.pb23.into_floating_input();
            out.set_high().unwrap();
            assert!(input.is_high().unwrap());
            out.set_low().unwrap();
            assert!(input.is_low().unwrap());
        }
        TestCase::Perid => {
            assert_eq!(PinsA::get_perid(), 0x004007e1);
            assert_eq!(PinsB::get_perid(), 0x004007e1);
        }
        TestCase::Pulse => {
            let mut output_pulsed = pinsa
                .pa0
                .into_push_pull_output()
                .pulse_mode(true, PinState::Low);
            rprintln!("Pulsing high 10 times..");
            output_pulsed.set_low().unwrap();
            for _ in 0..10 {
                output_pulsed.set_high().unwrap();
                cortex_m::asm::delay(25_000_000);
            }
            let mut output_pulsed = output_pulsed.pulse_mode(true, PinState::High);
            rprintln!("Pulsing low 10 times..");
            for _ in 0..10 {
                output_pulsed.set_low().unwrap();
                cortex_m::asm::delay(25_000_000);
            }
        }
        TestCase::DelayGpio => {
            let mut out_0 = pinsa.pa0.into_push_pull_output().delay(true, false);
            let mut out_1 = pinsa.pa1.into_push_pull_output().delay(false, true);
            let mut out_2 = pinsa.pa3.into_push_pull_output().delay(true, true);
            for _ in 0..20 {
                out_0.toggle().unwrap();
                out_1.toggle().unwrap();
                out_2.toggle().unwrap();
                cortex_m::asm::delay(25_000_000);
            }
        }
        TestCase::DelayMs => {
            let ms_timer = set_up_ms_timer(
                &mut dp.SYSCONFIG,
                &mut dp.IRQSEL,
                50.mhz().into(),
                dp.TIM0,
                pac::Interrupt::OC0,
            );
            unsafe {
                cortex_m::peripheral::NVIC::unmask(pac::Interrupt::OC0);
            }
            let mut delay = Delay::new(ms_timer);
            for _ in 0..5 {
                led1.toggle().ok();
                delay.delay_ms(500);
                led1.toggle().ok();
                delay.delay_ms(500);
            }

            let mut delay_timer = CountDownTimer::new(&mut dp.SYSCONFIG, 50.mhz().into(), dp.TIM1);
            let mut pa0 = pinsa.pa0.into_push_pull_output();
            for _ in 0..5 {
                led1.toggle().ok();
                delay_timer.delay_ms(200_u32);
                led1.toggle().ok();
                delay_timer.delay_ms(200_u32);
            }
            let ahb_freq: Hertz = 50.mhz().into();
            let mut syst_delay = cortex_m::delay::Delay::new(cp.SYST, ahb_freq.0);
            // Test usecond delay using both TIM peripheral and SYST
            loop {
                pa0.toggle().ok();
                delay_timer.delay_us(50_u32);
                pa0.toggle().ok();
                delay_timer.delay_us(50_u32);
                pa0.toggle().ok();
                syst_delay.delay_us(50);
                pa0.toggle().ok();
                syst_delay.delay_us(50);
            }
        }
    }

    rprintln!("Test success");
    loop {
        led1.toggle().ok();
        cortex_m::asm::delay(25_000_000);
    }
}

#[interrupt]
fn OC0() {
    default_ms_irq_handler()
}
