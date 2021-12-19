//! MS and Second counter implemented using the TIM0 and TIM1 peripheral
#![no_main]
#![no_std]

use core::cell::Cell;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use va108xx_hal::{
    clock::{get_sys_clock, set_sys_clock},
    pac::{self, interrupt},
    prelude::*,
    time::Hertz,
    timer::{default_ms_irq_handler, set_up_ms_timer, CountDownTimer, Event, IrqCfg, MS_COUNTER},
};

#[allow(dead_code)]
enum LibType {
    Pac,
    Hal,
}

static SEC_COUNTER: Mutex<Cell<u32>> = Mutex::new(Cell::new(0));

#[entry]
fn main() -> ! {
    rtt_init_print!();
    let mut dp = pac::Peripherals::take().unwrap();
    let mut last_ms = 0;
    rprintln!("-- Vorago system ticks using timers --");
    set_sys_clock(50.mhz());
    let lib_type = LibType::Hal;
    match lib_type {
        LibType::Pac => {
            unsafe {
                dp.SYSCONFIG
                    .peripheral_clk_enable
                    .modify(|_, w| w.irqsel().set_bit());
                dp.SYSCONFIG
                    .tim_clk_enable
                    .modify(|r, w| w.bits(r.bits() | (1 << 0) | (1 << 1)));
                dp.IRQSEL.tim[0].write(|w| w.bits(0x00));
                dp.IRQSEL.tim[1].write(|w| w.bits(0x01));
            }

            let sys_clk: Hertz = 50.mhz().into();
            let cnt_ms = sys_clk.0 / 1000 - 1;
            let cnt_sec = sys_clk.0 - 1;
            unsafe {
                dp.TIM0.cnt_value.write(|w| w.bits(cnt_ms));
                dp.TIM0.rst_value.write(|w| w.bits(cnt_ms));
                dp.TIM0.ctrl.write(|w| {
                    w.enable().set_bit();
                    w.irq_enb().set_bit()
                });
                dp.TIM1.cnt_value.write(|w| w.bits(cnt_sec));
                dp.TIM1.rst_value.write(|w| w.bits(cnt_sec));
                dp.TIM1.ctrl.write(|w| {
                    w.enable().set_bit();
                    w.irq_enb().set_bit()
                });
                unmask_irqs();
            }
        }
        LibType::Hal => {
            set_up_ms_timer(
                IrqCfg::new(interrupt::OC0, true, true),
                &mut dp.SYSCONFIG,
                Some(&mut dp.IRQSEL),
                50.mhz(),
                dp.TIM0,
            );
            let mut second_timer =
                CountDownTimer::new(&mut dp.SYSCONFIG, get_sys_clock().unwrap(), dp.TIM1);
            second_timer.listen(
                Event::TimeOut,
                IrqCfg::new(interrupt::OC1, true, true),
                Some(&mut dp.IRQSEL),
                Some(&mut dp.SYSCONFIG),
            );
            second_timer.start(1.hz());
        }
    }
    loop {
        let current_ms = cortex_m::interrupt::free(|cs| MS_COUNTER.borrow(cs).get());
        if current_ms - last_ms >= 1000 {
            last_ms = current_ms;
            rprintln!("MS counter: {}", current_ms);
            let second = cortex_m::interrupt::free(|cs| SEC_COUNTER.borrow(cs).get());
            rprintln!("Second counter: {}", second);
        }
        cortex_m::asm::delay(10000);
    }
}

fn unmask_irqs() {
    unsafe {
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::OC0);
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::OC1);
    }
}

#[interrupt]
fn OC0() {
    default_ms_irq_handler()
}

#[interrupt]
fn OC1() {
    cortex_m::interrupt::free(|cs| {
        let mut sec = SEC_COUNTER.borrow(cs).get();
        sec += 1;
        SEC_COUNTER.borrow(cs).set(sec);
    });
}
