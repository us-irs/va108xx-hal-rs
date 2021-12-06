//! API for Pulse-Width Modulation (PWM)
//!
//! The Vorago VA108xx devices use the TIM peripherals to perform PWM related tasks
//!
//! ## Examples
//!
//! - [PWM example](https://egit.irs.uni-stuttgart.de/rust/va108xx-hal/src/branch/main/examples/pwm.rs)
use core::marker::PhantomData;

use crate::{clock::enable_peripheral_clock, gpio::DynPinId};
pub use crate::{gpio::PinId, prelude::*, time::Hertz, timer::*};

use va108xx::SYSCONFIG;

const DUTY_MAX: u16 = u16::MAX;

pub struct PwmBase {
    sys_clk: Hertz,
    /// For PWMB, this is the upper limit
    current_duty: u16,
    /// For PWMA, this value will not be used
    current_lower_limit: u16,
    current_period: Hertz,
    current_rst_val: u32,
}

enum StatusSelPwm {
    PwmA = 3,
    PwmB = 4,
}

pub struct PWMA {}
pub struct PWMB {}

//==================================================================================================
// Common
//==================================================================================================

macro_rules! pwm_common_func {
    () => {
        #[inline]
        fn enable_pwm_a(&mut self) {
            self.reg
                .reg()
                .ctrl
                .modify(|_, w| unsafe { w.status_sel().bits(StatusSelPwm::PwmA as u8) });
        }

        #[inline]
        fn enable_pwm_b(&mut self) {
            self.reg
                .reg()
                .ctrl
                .modify(|_, w| unsafe { w.status_sel().bits(StatusSelPwm::PwmB as u8) });
        }

        #[inline]
        pub fn get_period(&self) -> Hertz {
            self.pwm_base.current_period
        }

        #[inline]
        pub fn set_period(&mut self, period: impl Into<Hertz>) {
            self.pwm_base.current_period = period.into();
            // Avoid division by 0
            if self.pwm_base.current_period.0 == 0 {
                return;
            }
            self.pwm_base.current_rst_val =
                self.pwm_base.sys_clk.0 / self.pwm_base.current_period.0;
            self.reg
                .reg()
                .rst_value
                .write(|w| unsafe { w.bits(self.pwm_base.current_rst_val) });
        }
    };
}

macro_rules! pwmb_func {
    () => {
        pub fn pwmb_lower_limit(&self) -> u16 {
            self.pwm_base.current_lower_limit
        }

        pub fn pwmb_upper_limit(&self) -> u16 {
            self.pwm_base.current_duty
        }

        /// Set the lower limit for PWMB
        ///
        /// The PWM signal will be 1 as long as the current RST counter is larger than
        /// the lower limit. For example, with a lower limit of 0.5 and and an upper limit
        /// of 0.7, Only a fixed period between 0.5 * period and 0.7 * period will be in a high
        /// state
        pub fn set_pwmb_lower_limit(&mut self, duty: u16) {
            self.pwm_base.current_lower_limit = duty;
            let pwmb_val: u64 = (self.pwm_base.current_rst_val as u64
                * self.pwm_base.current_lower_limit as u64)
                / DUTY_MAX as u64;
            self.reg
                .reg()
                .pwmb_value
                .write(|w| unsafe { w.bits(pwmb_val as u32) });
        }

        /// Set the higher limit for PWMB
        ///
        /// The PWM signal will be 1 as long as the current RST counter is smaller than
        /// the higher limit. For example, with a lower limit of 0.5 and and an upper limit
        /// of 0.7, Only a fixed period between 0.5 * period and 0.7 * period will be in a high
        /// state
        pub fn set_pwmb_upper_limit(&mut self, duty: u16) {
            self.pwm_base.current_duty = duty;
            let pwma_val: u64 = (self.pwm_base.current_rst_val as u64
                * self.pwm_base.current_duty as u64)
                / DUTY_MAX as u64;
            self.reg
                .reg()
                .pwma_value()
                .write(|w| unsafe { w.bits(pwma_val as u32) });
        }
    };
}

//==================================================================================================
// Strongly typed PWM pin
//==================================================================================================

pub struct PwmPin<PIN: TimPin, TIM: ValidTim, MODE = PWMA> {
    reg: TimAndPinRegister<PIN, TIM>,
    pwm_base: PwmBase,
    _mode: PhantomData<MODE>,
}

impl<PIN: TimPin, TIM: ValidTim, MODE> PwmPin<PIN, TIM, MODE>
where
    (PIN, TIM): ValidTimAndPin<PIN, TIM>,
{
    /// Create a new stronlgy typed PWM pin
    pub fn new(
        vtp: (PIN, TIM),
        sys_clk: impl Into<Hertz> + Copy,
        sys_cfg: &mut SYSCONFIG,
        initial_period: impl Into<Hertz> + Copy,
    ) -> Self {
        let mut pin = PwmPin {
            pwm_base: PwmBase {
                current_duty: 0,
                current_lower_limit: 0,
                current_period: initial_period.into(),
                current_rst_val: 0,
                sys_clk: sys_clk.into(),
            },
            reg: unsafe { TimAndPinRegister::new(vtp.0, vtp.1) },
            _mode: PhantomData,
        };
        enable_peripheral_clock(sys_cfg, crate::clock::PeripheralClocks::Gpio);
        enable_peripheral_clock(sys_cfg, crate::clock::PeripheralClocks::Ioconfig);
        sys_cfg
            .tim_clk_enable
            .modify(|r, w| unsafe { w.bits(r.bits() | pin.reg.mask_32()) });
        pin.enable_pwm_a();
        pin.set_period(initial_period);
        pin
    }
    pub fn release(self) -> (PIN, TIM) {
        self.reg.release()
    }

    pwm_common_func!();
}

impl<PIN: TimPin, TIM: ValidTim> From<PwmPin<PIN, TIM, PWMA>> for PwmPin<PIN, TIM, PWMB>
where
    (PIN, TIM): ValidTimAndPin<PIN, TIM>,
{
    fn from(other: PwmPin<PIN, TIM, PWMA>) -> Self {
        let mut pwmb = Self {
            reg: other.reg,
            pwm_base: other.pwm_base,
            _mode: PhantomData,
        };
        pwmb.enable_pwm_b();
        pwmb
    }
}

impl<PIN: TimPin, TIM: ValidTim> From<PwmPin<PIN, TIM, PWMB>> for PwmPin<PIN, TIM, PWMA>
where
    (PIN, TIM): ValidTimAndPin<PIN, TIM>,
{
    fn from(other: PwmPin<PIN, TIM, PWMB>) -> Self {
        let mut pwmb = Self {
            reg: other.reg,
            pwm_base: other.pwm_base,
            _mode: PhantomData,
        };
        pwmb.enable_pwm_a();
        pwmb
    }
}

impl<PIN: TimPin, TIM: ValidTim> PwmPin<PIN, TIM, PWMA>
where
    (PIN, TIM): ValidTimAndPin<PIN, TIM>,
{
    pub fn pwma(
        vtp: (PIN, TIM),
        sys_clk: impl Into<Hertz> + Copy,
        sys_cfg: &mut SYSCONFIG,
        initial_period: impl Into<Hertz> + Copy,
    ) -> Self {
        let mut pin: PwmPin<PIN, TIM, PWMA> = Self::new(vtp, sys_clk, sys_cfg, initial_period);
        pin.enable_pwm_a();
        pin
    }
}

impl<PIN: TimPin, TIM: ValidTim> PwmPin<PIN, TIM, PWMB>
where
    (PIN, TIM): ValidTimAndPin<PIN, TIM>,
{
    pub fn pwmb(
        vtp: (PIN, TIM),
        sys_clk: impl Into<Hertz> + Copy,
        sys_cfg: &mut SYSCONFIG,
        initial_period: impl Into<Hertz> + Copy,
    ) -> Self {
        let mut pin: PwmPin<PIN, TIM, PWMB> = Self::new(vtp, sys_clk, sys_cfg, initial_period);
        pin.enable_pwm_b();
        pin
    }
}

//==================================================================================================
// Reduced PWM pin
//==================================================================================================

/// Reduced version where type information is deleted
pub struct ReducedPwmPin<MODE = PWMA> {
    reg: TimDynRegister,
    pwm_base: PwmBase,
    _pin_id: DynPinId,
    _mode: PhantomData<MODE>,
}

impl<PIN: TimPin, TIM: ValidTim> From<PwmPin<PIN, TIM>> for ReducedPwmPin<PWMA> {
    fn from(pwm_pin: PwmPin<PIN, TIM>) -> Self {
        ReducedPwmPin {
            reg: TimDynRegister::from(pwm_pin.reg),
            pwm_base: pwm_pin.pwm_base,
            _pin_id: PIN::DYN,
            _mode: PhantomData,
        }
    }
}

impl<MODE> ReducedPwmPin<MODE> {
    pwm_common_func!();
}

impl From<ReducedPwmPin<PWMA>> for ReducedPwmPin<PWMB> {
    fn from(other: ReducedPwmPin<PWMA>) -> Self {
        let mut pwmb = Self {
            reg: other.reg,
            pwm_base: other.pwm_base,
            _pin_id: other._pin_id,
            _mode: PhantomData,
        };
        pwmb.enable_pwm_b();
        pwmb
    }
}

impl From<ReducedPwmPin<PWMB>> for ReducedPwmPin<PWMA> {
    fn from(other: ReducedPwmPin<PWMB>) -> Self {
        let mut pwmb = Self {
            reg: other.reg,
            pwm_base: other.pwm_base,
            _pin_id: other._pin_id,
            _mode: PhantomData,
        };
        pwmb.enable_pwm_a();
        pwmb
    }
}

//==================================================================================================
// PWMB implementations
//==================================================================================================

impl<PIN: TimPin, TIM: ValidTim> PwmPin<PIN, TIM, PWMB>
where
    (PIN, TIM): ValidTimAndPin<PIN, TIM>,
{
    pwmb_func!();
}

impl ReducedPwmPin<PWMB> {
    pwmb_func!();
}

//==================================================================================================
// Embedded HAL implementation: PWMA only
//==================================================================================================

macro_rules! pwm_pin_impl {
    () => {
        #[inline]
        fn disable(&mut self) {
            self.reg.reg().ctrl.modify(|_, w| w.enable().clear_bit());
        }

        #[inline]
        fn enable(&mut self) {
            self.reg.reg().ctrl.modify(|_, w| w.enable().set_bit());
        }

        #[inline]
        fn set_duty(&mut self, duty: Self::Duty) {
            self.pwm_base.current_duty = duty;
            let pwma_val: u64 = (self.pwm_base.current_rst_val as u64
                * (DUTY_MAX as u64 - self.pwm_base.current_duty as u64))
                / DUTY_MAX as u64;
            self.reg
                .reg()
                .pwma_value()
                .write(|w| unsafe { w.bits(pwma_val as u32) });
        }

        #[inline]
        fn get_duty(&self) -> Self::Duty {
            self.pwm_base.current_duty
        }

        #[inline]
        fn get_max_duty(&self) -> Self::Duty {
            DUTY_MAX
        }
    };
}

macro_rules! pwm_impl {
    () => {
        #[inline]
        fn disable(&mut self, _channel: Self::Channel) {
            self.reg.reg().ctrl.modify(|_, w| w.enable().clear_bit());
        }

        #[inline]
        fn enable(&mut self, _channel: Self::Channel) {
            self.reg.reg().ctrl.modify(|_, w| w.enable().set_bit());
        }

        #[inline]
        fn get_period(&self) -> Self::Time {
            self.pwm_base.current_period
        }

        #[inline]
        fn set_duty(&mut self, _channel: Self::Channel, duty: Self::Duty) {
            self.pwm_base.current_duty = duty;
            let pwma_val: u64 = (self.pwm_base.current_rst_val as u64
                * (DUTY_MAX as u64 - self.pwm_base.current_duty as u64))
                / DUTY_MAX as u64;
            self.reg
                .reg()
                .pwma_value()
                .write(|w| unsafe { w.bits(pwma_val as u32) });
        }

        #[inline]
        fn set_period<P>(&mut self, period: P)
        where
            P: Into<Self::Time>,
        {
            self.pwm_base.current_period = period.into();
            // Avoid division by 0
            if self.pwm_base.current_period.0 == 0 {
                return;
            }
            self.pwm_base.current_rst_val =
                self.pwm_base.sys_clk.0 / self.pwm_base.current_period.0;
            let reg_block = self.reg.reg();
            reg_block
                .rst_value
                .write(|w| unsafe { w.bits(self.pwm_base.current_rst_val) });
            reg_block
                .cnt_value
                .write(|w| unsafe { w.bits(self.pwm_base.current_rst_val) });
        }

        #[inline(always)]
        fn get_duty(&self, _channel: Self::Channel) -> Self::Duty {
            self.pwm_base.current_duty
        }

        #[inline(always)]
        fn get_max_duty(&self) -> Self::Duty {
            DUTY_MAX
        }
    };
}

impl<PIN: TimPin, TIM: ValidTim> embedded_hal::Pwm for PwmPin<PIN, TIM> {
    type Channel = ();
    type Duty = u16;
    type Time = Hertz;

    pwm_impl!();
}

impl embedded_hal::Pwm for ReducedPwmPin<PWMA> {
    type Channel = ();
    type Duty = u16;
    type Time = Hertz;

    pwm_impl!();
}

impl<PIN: TimPin, TIM: ValidTim> embedded_hal::PwmPin for PwmPin<PIN, TIM> {
    type Duty = u16;

    pwm_pin_impl!();
}

impl embedded_hal::PwmPin for ReducedPwmPin<PWMA> {
    type Duty = u16;

    pwm_pin_impl!();
}

/// Get the corresponding u16 duty cycle from a percent value ranging between 0.0 and 1.0.
///
/// Please note that this might load a lot of floating point code because this processor does not
/// have a FPU
pub fn get_duty_from_percent(percent: f32) -> u16 {
    if percent > 1.0 {
        DUTY_MAX
    } else if percent <= 0.0 {
        0
    } else {
        (percent * DUTY_MAX as f32) as u16
    }
}
