//! API for the TIM peripherals
//!
//! ## Examples
//!
//! - [MS and second tick implementation](https://github.com/robamu-org/va108xx-hal-rs/blob/main/examples/timer-ticks.rs)
use crate::{
    clock::{enable_peripheral_clock, PeripheralClocks},
    pac,
    prelude::*,
    time::Hertz,
    timer,
};
use core::cell::Cell;
use cortex_m::interrupt::Mutex;
use embedded_hal::{
    blocking::delay,
    timer::{Cancel, CountDown, Periodic},
};
use va108xx::{Interrupt, IRQSEL, SYSCONFIG};
use void::Void;

const IRQ_DST_NONE: u32 = 0xffffffff;
pub static MS_COUNTER: Mutex<Cell<u32>> = Mutex::new(Cell::new(0));

/// Hardware timers
pub struct CountDownTimer<TIM> {
    tim: TIM,
    curr_freq: Hertz,
    sys_clk: Hertz,
    rst_val: u32,
    last_cnt: u32,
    listening: bool,
}

/// Interrupt events
pub enum Event {
    /// Timer timed out / count down ended
    TimeOut,
}

pub enum TimerErrors {
    Canceled,
}

fn enable_tim_clk(syscfg: &mut SYSCONFIG, idx: u8) {
    syscfg
        .tim_clk_enable
        .modify(|r, w| unsafe { w.bits(r.bits() | (1 << idx)) });
}

macro_rules! timers {
    ($($TIM:ident: ($tim:ident, $i:expr),)+) => {
        $(
            use crate::pac::$TIM;

            impl CountDownTimer<$TIM> {
                // XXX(why not name this `new`?) bummer: constructors need to have different names
                // even if the `$TIM` are non overlapping (compare to the `free` function below
                // which just works)
                /// Configures a TIM peripheral as a periodic count down timer
                pub fn $tim(
                    syscfg: &mut SYSCONFIG, sys_clk: Hertz, tim: $TIM
                ) -> Self {
                    enable_tim_clk(syscfg, $i);
                    tim.ctrl.modify(|_, w| w.enable().set_bit());
                    CountDownTimer {
                        tim,
                        sys_clk,
                        rst_val: 0,
                        curr_freq: 0.hz(),
                        listening: false,
                        last_cnt: 0,
                    }
                }

                /// Listen for events. This also actives the IRQ in the IRQSEL register
                /// for the provided interrupt. It also actives the peripheral clock for
                /// IRQSEL
                pub fn listen(
                    &mut self,
                    event: Event,
                    syscfg: &mut SYSCONFIG,
                    irqsel: &mut IRQSEL,
                    interrupt: Interrupt,
                ) {
                    match event {
                        Event::TimeOut => {
                            enable_peripheral_clock(syscfg, PeripheralClocks::Irqsel);
                            irqsel.tim[$i].write(|w| unsafe { w.bits(interrupt as u32) });
                            self.tim.ctrl.modify(|_, w| w.irq_enb().set_bit());
                            self.listening = true;
                        }
                    }
                }

                pub fn unlisten(
                    &mut self, event: Event, syscfg: &mut SYSCONFIG, irqsel: &mut IRQSEL
                ) {
                    match event {
                        Event::TimeOut => {
                            enable_peripheral_clock(syscfg, PeripheralClocks::Irqsel);
                            irqsel.tim[$i].write(|w| unsafe { w.bits(IRQ_DST_NONE) });
                            self.tim.ctrl.modify(|_, w| w.irq_enb().clear_bit());
                            self.listening = false;
                        }
                    }
                }

                pub fn release(self, syscfg: &mut SYSCONFIG) -> $TIM {
                    self.tim.ctrl.write(|w| w.enable().clear_bit());
                    syscfg
                        .tim_clk_enable
                        .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << $i)) });
                    self.tim
                }

                pub fn auto_disable(self, enable: bool) -> Self {
                    if enable {
                        self.tim.ctrl.modify(|_, w| w.auto_disable().set_bit());
                    } else {
                        self.tim.ctrl.modify(|_, w| w.auto_disable().clear_bit());
                    }
                    self
                }

                pub fn auto_deactivate(self, enable: bool) -> Self {
                    if enable {
                        self.tim.ctrl.modify(|_, w| w.auto_deactivate().set_bit());
                    } else {
                        self.tim.ctrl.modify(|_, w| w.auto_deactivate().clear_bit());
                    }
                    self
                }

                pub fn curr_freq(&self) -> Hertz {
                    self.curr_freq
                }

                pub fn listening(&self) -> bool {
                    self.listening
                }
            }

            /// CountDown implementation for TIMx
            impl CountDown for CountDownTimer<$TIM> {
                type Time = Hertz;

                fn start<T>(&mut self, timeout: T)
                where
                    T: Into<Hertz>,
                {
                    self.tim.ctrl.modify(|_, w| w.enable().clear_bit());
                    self.curr_freq = timeout.into();
                    self.rst_val = self.sys_clk.0 / self.curr_freq.0;
                    unsafe {
                        self.tim.rst_value.write(|w| w.bits(self.rst_val));
                        self.tim.cnt_value.write(|w| w.bits(self.rst_val));
                    }
                    self.tim.ctrl.modify(|_, w| w.enable().set_bit());
                }

                /// Return `Ok` if the timer has wrapped. Peripheral will automatically clear the
                /// flag and restart the time if configured correctly
                fn wait(&mut self) -> nb::Result<(), Void> {
                    let cnt = self.tim.cnt_value.read().bits();
                    if cnt > self.last_cnt {
                        self.last_cnt = self.rst_val;
                        Ok(())
                    } else if cnt == 0 {
                        self.last_cnt = self.rst_val;
                        Ok(())
                    } else {
                        self.last_cnt = cnt;
                        Err(nb::Error::WouldBlock)
                    }
                }
            }

            impl Periodic for CountDownTimer<$TIM> {}

            impl Cancel for CountDownTimer<$TIM> {
                type Error = TimerErrors;
                fn cancel(&mut self) -> Result<(), Self::Error> {
                    if !self.tim.ctrl.read().enable().bit_is_set() {
                        return Err(TimerErrors::Canceled);
                    }
                    self.tim.ctrl.write(|w| w.enable().clear_bit());
                    Ok(())
                }
            }

            /// Delay for microseconds.
            ///
            /// For delays less than 100 us, an assembly delay will be used.
            /// For larger delays, the timer peripheral will be used.
            /// Please note that the delay using the peripheral might not
            /// work properly in debug mode.
            impl delay::DelayUs<u32> for CountDownTimer<$TIM> {
                fn delay_us(&mut self, us: u32) {
                    if(us < 100) {
                       cortex_m::asm::delay(us * (self.sys_clk.0 / 2_000_000));
                    } else {
                        // Configuring the peripheral for higher frequencies is unstable
                        self.start(1000.khz());
                        // The subtracted value is an empirical value measures by using tests with
                        // an oscilloscope.
                        for _ in 0..us - 7 {
                            nb::block!(self.wait()).unwrap();
                        }
                    }
                }
            }
            /// Forwards call to u32 variant of delay
            impl delay::DelayUs<u16> for CountDownTimer<$TIM> {
                fn delay_us(&mut self, us: u16) {
                    self.delay_us(u32::from(us));
                }
            }
            /// Forwards call to u32 variant of delay
            impl delay::DelayUs<u8> for CountDownTimer<$TIM> {
                fn delay_us(&mut self, us: u8) {
                    self.delay_us(u32::from(us));
                }
            }

            impl delay::DelayMs<u32> for CountDownTimer<$TIM> {
                fn delay_ms(&mut self, ms: u32) {
                    self.start(1000.hz());
                    for _ in 0..ms {
                        nb::block!(self.wait()).unwrap();
                    }
                }
            }
            impl delay::DelayMs<u16> for CountDownTimer<$TIM> {
                fn delay_ms(&mut self, ms: u16) {
                    self.delay_ms(u32::from(ms));
                }
            }
            impl embedded_hal::blocking::delay::DelayMs<u8> for CountDownTimer<$TIM> {
                fn delay_ms(&mut self, ms: u8) {
                    self.delay_ms(u32::from(ms));
                }
            }

        )+
    }
}

// Set up a millisecond timer on TIM0. Please note that you still need to unmask the related IRQ
// and provide an IRQ handler yourself
pub fn set_up_ms_timer(
    syscfg: &mut pac::SYSCONFIG,
    irqsel: &mut pac::IRQSEL,
    sys_clk: Hertz,
    tim0: TIM0,
    irq: pac::Interrupt,
) -> CountDownTimer<TIM0> {
    let mut ms_timer = CountDownTimer::tim0(syscfg, sys_clk, tim0);
    ms_timer.listen(timer::Event::TimeOut, syscfg, irqsel, irq);
    ms_timer.start(1000.hz());
    ms_timer
}

/// This function can be called in a specified interrupt handler to increment
/// the MS counter
pub fn default_ms_irq_handler() {
    cortex_m::interrupt::free(|cs| {
        let mut ms = MS_COUNTER.borrow(cs).get();
        ms += 1;
        MS_COUNTER.borrow(cs).set(ms);
    });
}

/// Get the current MS tick count
pub fn get_ms_ticks() -> u32 {
    cortex_m::interrupt::free(|cs| MS_COUNTER.borrow(cs).get())
}

timers! {
    TIM0: (tim0, 0),
    TIM1: (tim1, 1),
    TIM2: (tim2, 2),
    TIM3: (tim3, 3),
    TIM4: (tim4, 4),
    TIM5: (tim5, 5),
    TIM6: (tim6, 6),
    TIM7: (tim7, 7),
    TIM8: (tim8, 8),
    TIM9: (tim9, 9),
    TIM10: (tim10, 10),
    TIM11: (tim11, 11),
    TIM12: (tim12, 12),
    TIM13: (tim13, 13),
    TIM14: (tim14, 14),
    TIM15: (tim15, 15),
    TIM16: (tim16, 16),
    TIM17: (tim17, 17),
    TIM18: (tim18, 18),
    TIM19: (tim19, 19),
    TIM20: (tim20, 20),
    TIM21: (tim21, 21),
    TIM22: (tim22, 22),
    TIM23: (tim23, 23),
}

//==================================================================================================
// Delay implementations
//==================================================================================================

pub struct Delay {
    cd_tim: CountDownTimer<TIM0>,
}

impl Delay {
    pub fn new(tim0: CountDownTimer<TIM0>) -> Self {
        Delay { cd_tim: tim0 }
    }
}

/// This assumes that the user has already set up a MS tick timer in TIM0 as a system tick
impl embedded_hal::blocking::delay::DelayMs<u32> for Delay {
    fn delay_ms(&mut self, ms: u32) {
        if self.cd_tim.curr_freq() != 1000.hz() || !self.cd_tim.listening() {
            return;
        }
        let start_time = get_ms_ticks();
        while get_ms_ticks() - start_time < ms {
            cortex_m::asm::nop();
        }
    }
}
