//! API for the TIM peripherals
//!
//! ## Examples
//!
//! - [MS and second tick implementation](https://github.com/robamu-org/va108xx-hal-rs/blob/main/examples/timer-ticks.rs)
use crate::{
    clock::{enable_peripheral_clock, PeripheralClocks},
    time::Hertz,
};
use embedded_hal::timer::{Cancel, CountDown, Periodic};
use va108xx::{Interrupt, IRQSEL, SYSCONFIG};
use void::Void;

const IRQ_DST_NONE: u32 = 0xffffffff;

/// Hardware timers
pub struct CountDownTimer<TIM> {
    tim: TIM,
    sys_clk: Hertz,
    last_cnt: u32,
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
            }

            /// CountDown implementation for TIMx
            impl CountDown for CountDownTimer<$TIM> {
                type Time = Hertz;

                fn start<T>(&mut self, timeout: T)
                where
                    T: Into<Hertz>,
                {
                    self.last_cnt = self.sys_clk.0 / timeout.into().0 - 1;
                    unsafe {
                        self.tim.rst_value.write(|w| w.bits(self.last_cnt));
                        self.tim.cnt_value.write(|w| w.bits(self.last_cnt));
                    }
                }

                /// Return `Ok` if the timer has wrapped
                /// Automatically clears the flag and restarts the time
                fn wait(&mut self) -> nb::Result<(), Void> {
                    let cnt = self.tim.cnt_value.read().bits();
                    if cnt == 0 || cnt < self.last_cnt {
                        self.last_cnt = cnt;
                        Ok(())
                    } else {
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
        )+
    }
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
