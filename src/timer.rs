//! API for the TIM peripherals
//!
//! ## Examples
//!
//! - [MS and second tick implementation](https://github.com/robamu-org/va108xx-hal-rs/blob/main/examples/timer-ticks.rs)
use crate::{
    clock::{enable_peripheral_clock, PeripheralClocks},
    gpio::{
        AltFunc1, AltFunc2, AltFunc3, DynPinId, Pin, PinId, PA0, PA1, PA10, PA11, PA12, PA13, PA14,
        PA15, PA2, PA24, PA25, PA26, PA27, PA28, PA29, PA3, PA30, PA31, PA4, PA5, PA6, PA7, PA8,
        PA9, PB0, PB1, PB10, PB11, PB12, PB13, PB14, PB15, PB16, PB17, PB18, PB19, PB2, PB20, PB21,
        PB22, PB23, PB3, PB4, PB5, PB6,
    },
    pac::{
        self, tim0, TIM0, TIM1, TIM10, TIM11, TIM12, TIM13, TIM14, TIM15, TIM16, TIM17, TIM18,
        TIM19, TIM2, TIM20, TIM21, TIM22, TIM23, TIM3, TIM4, TIM5, TIM6, TIM7, TIM8, TIM9,
    },
    prelude::*,
    private::Sealed,
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

//==================================================================================================
// Defintions
//==================================================================================================

/// Interrupt events
pub enum Event {
    /// Timer timed out / count down ended
    TimeOut,
}

#[derive(Default, Debug, PartialEq, Copy, Clone)]
pub struct CascadeCtrl {
    /// Enable Cascade 0 signal active as a requirement for counting
    pub enb_start_src_csd0: bool,
    /// Invert Cascade 0, making it active low
    pub inv_csd0: bool,
    /// Enable Cascade 1 signal active as a requirement for counting
    pub enb_start_src_csd1: bool,
    /// Invert Cascade 1, making it active low
    pub inv_csd1: bool,
    /// Specify required operation if both Cascade 0 and Cascade 1 are active.
    /// 0 is a logical AND of both cascade signals, 1 is a logical OR
    pub dual_csd_op: bool,
    /// Enable trigger mode for Cascade 0. In trigger mode, couting will start with the selected
    /// cascade signal active, but once the counter is active, cascade control will be ignored
    pub trg_csd0: bool,
    /// Trigger mode, identical to [`trg_csd0`](CascadeCtrl) but for Cascade 1
    pub trg_csd1: bool,
    /// Enable Cascade 2 signal active as a requirement to stop counting. This mode is similar
    /// to the REQ_STOP control bit, but signalled by a Cascade source
    pub enb_stop_src_csd2: bool,
    /// Invert Cascade 2, making it active low
    pub inv_csd2: bool,
    /// The counter is automatically disabled if the corresponding Cascade 2 level-sensitive input
    /// souce is active when the count reaches 0. If the counter is not 0, the cascade control is
    /// ignored
    pub trg_csd2: bool,
}

#[derive(Debug, PartialEq)]
pub enum CascadeSel {
    Csd0 = 0,
    Csd1 = 1,
    Csd2 = 2,
}

/// The numbers are the base numbers for bundles like PORTA, PORTB or TIM
#[derive(Debug, PartialEq)]
pub enum CascadeSource {
    PortABase = 0,
    PortBBase = 32,
    TimBase = 64,
    RamSbe = 96,
    RamMbe = 97,
    RomSbe = 98,
    RomMbe = 99,
    Txev = 100,
    ClockDividerBase = 120,
}

#[derive(Debug, PartialEq)]
pub enum TimerErrors {
    Canceled,
    /// Invalid input for Cascade source
    InvalidCsdSourceInput,
}

//==================================================================================================
// Valid TIM and PIN combinations
//==================================================================================================

pub trait TimPin {
    const DYN: DynPinId;
}

pub trait ValidTim {
    // TIM ID ranging from 0 to 23 for 24 TIM peripherals
    const TIM_ID: u8;
}

macro_rules! tim_marker {
    ($TIMX:ident, $ID:expr) => {
        impl ValidTim for $TIMX {
            const TIM_ID: u8 = $ID;
        }
    };
}

tim_marker!(TIM0, 0);
tim_marker!(TIM1, 1);
tim_marker!(TIM2, 2);
tim_marker!(TIM3, 3);
tim_marker!(TIM4, 4);
tim_marker!(TIM5, 5);
tim_marker!(TIM6, 6);
tim_marker!(TIM7, 7);
tim_marker!(TIM8, 8);
tim_marker!(TIM9, 9);
tim_marker!(TIM10, 10);
tim_marker!(TIM11, 11);
tim_marker!(TIM12, 12);
tim_marker!(TIM13, 13);
tim_marker!(TIM14, 14);
tim_marker!(TIM15, 15);
tim_marker!(TIM16, 16);
tim_marker!(TIM17, 17);
tim_marker!(TIM18, 18);
tim_marker!(TIM19, 19);
tim_marker!(TIM20, 20);
tim_marker!(TIM21, 21);
tim_marker!(TIM22, 22);
tim_marker!(TIM23, 23);

pub trait ValidTimAndPin<PIN: TimPin, TIM: ValidTim>: Sealed {}

macro_rules! pin_and_tim {
    ($PAX:ident, $ALTFUNC:ident, $ID:expr, $TIMX:ident) => {
        impl TimPin for Pin<$PAX, $ALTFUNC>
        where
            $PAX: PinId,
        {
            const DYN: DynPinId = $PAX::DYN;
        }

        impl<PIN: TimPin, TIM: ValidTim> ValidTimAndPin<PIN, TIM> for (Pin<$PAX, $ALTFUNC>, $TIMX)
        where
            Pin<$PAX, $ALTFUNC>: TimPin,
            $PAX: PinId,
        {
        }

        impl Sealed for (Pin<$PAX, $ALTFUNC>, $TIMX) {}
    };
}

pin_and_tim!(PA31, AltFunc2, 23, TIM23);
pin_and_tim!(PA30, AltFunc2, 22, TIM22);
pin_and_tim!(PA29, AltFunc2, 21, TIM21);
pin_and_tim!(PA28, AltFunc2, 20, TIM20);
pin_and_tim!(PA27, AltFunc2, 19, TIM19);
pin_and_tim!(PA26, AltFunc2, 18, TIM18);
pin_and_tim!(PA25, AltFunc2, 17, TIM17);
pin_and_tim!(PA24, AltFunc2, 16, TIM16);

pin_and_tim!(PA15, AltFunc1, 15, TIM15);
pin_and_tim!(PA14, AltFunc1, 14, TIM14);
pin_and_tim!(PA13, AltFunc1, 13, TIM13);
pin_and_tim!(PA12, AltFunc1, 12, TIM12);
pin_and_tim!(PA11, AltFunc1, 11, TIM11);
pin_and_tim!(PA10, AltFunc1, 10, TIM10);
pin_and_tim!(PA9, AltFunc1, 9, TIM9);
pin_and_tim!(PA8, AltFunc1, 8, TIM8);
pin_and_tim!(PA7, AltFunc1, 7, TIM7);
pin_and_tim!(PA6, AltFunc1, 6, TIM6);
pin_and_tim!(PA5, AltFunc1, 5, TIM5);
pin_and_tim!(PA4, AltFunc1, 4, TIM4);
pin_and_tim!(PA3, AltFunc1, 3, TIM3);
pin_and_tim!(PA2, AltFunc1, 2, TIM2);
pin_and_tim!(PA1, AltFunc1, 1, TIM1);
pin_and_tim!(PA0, AltFunc1, 0, TIM0);

pin_and_tim!(PB23, AltFunc3, 23, TIM23);
pin_and_tim!(PB22, AltFunc3, 22, TIM22);
pin_and_tim!(PB21, AltFunc3, 21, TIM21);
pin_and_tim!(PB20, AltFunc3, 20, TIM20);
pin_and_tim!(PB19, AltFunc3, 19, TIM19);
pin_and_tim!(PB18, AltFunc3, 18, TIM18);
pin_and_tim!(PB17, AltFunc3, 17, TIM17);
pin_and_tim!(PB16, AltFunc3, 16, TIM16);
pin_and_tim!(PB15, AltFunc3, 15, TIM15);
pin_and_tim!(PB14, AltFunc3, 14, TIM14);
pin_and_tim!(PB13, AltFunc3, 13, TIM13);
pin_and_tim!(PB12, AltFunc3, 12, TIM12);
pin_and_tim!(PB11, AltFunc3, 11, TIM11);
pin_and_tim!(PB10, AltFunc3, 10, TIM10);

pin_and_tim!(PB6, AltFunc3, 6, TIM6);
pin_and_tim!(PB5, AltFunc3, 5, TIM5);
pin_and_tim!(PB4, AltFunc3, 4, TIM4);
pin_and_tim!(PB3, AltFunc3, 3, TIM3);
pin_and_tim!(PB2, AltFunc3, 2, TIM2);
pin_and_tim!(PB1, AltFunc3, 1, TIM1);
pin_and_tim!(PB0, AltFunc3, 0, TIM0);

//==================================================================================================
// Register Interface for TIM registers and TIM pins
//==================================================================================================

pub type TimRegBlock = tim0::RegisterBlock;

/// Register interface.
///
/// This interface provides valid TIM pins a way to access their corresponding TIM
/// registers
///
/// # Safety
///
/// Users should only implement the [`tim_id`] function. No default function
/// implementations should be overridden. The implementing type must also have
/// "control" over the corresponding pin ID, i.e. it must guarantee that a each
/// pin ID is a singleton.
pub(super) unsafe trait TimRegInterface {
    fn tim_id(&self) -> u8;

    const PORT_BASE: *const tim0::RegisterBlock = TIM0::ptr() as *const _;

    /// All 24 TIM blocks are identical. This helper functions returns the correct
    /// memory mapped peripheral depending on the TIM ID.
    #[inline(always)]
    fn reg(&self) -> &TimRegBlock {
        unsafe { &*Self::PORT_BASE.offset(self.tim_id() as isize) }
    }

    #[inline(always)]
    fn mask_32(&self) -> u32 {
        1 << self.tim_id()
    }

    /// Clear the reset bit of the TIM, holding it in reset
    ///
    /// # Safety
    ///
    /// Only the bit related to the corresponding TIM peripheral is modified
    #[inline]
    fn clear_tim_reset_bit(&self) {
        unsafe {
            va108xx::Peripherals::steal()
                .SYSCONFIG
                .tim_reset
                .modify(|r, w| w.bits(r.bits() & !self.mask_32()))
        }
    }

    #[inline]
    fn set_tim_reset_bit(&self) {
        unsafe {
            va108xx::Peripherals::steal()
                .SYSCONFIG
                .tim_reset
                .modify(|r, w| w.bits(r.bits() | self.mask_32()))
        }
    }
}

/// Register interface.
///
/// This interface provides an interface for TIM pins to access their corresponding
/// configuration
///
/// # Safety
///
/// Users should only implement the [`pin_id`] function. No default function
/// implementations should be overridden. The implementing type must also have
/// "control" over the corresponding pin ID, i.e. it must guarantee that a each
/// pin ID is a singleton.
pub(super) unsafe trait TimPinInterface {
    fn pin_id(&self) -> DynPinId;
}

/// Provide a safe register interface for [`ValidTimAndPin`]s
///
/// This `struct` takes ownership of a [`ValidTimAndPin`] and provides an API to
/// access the corresponding registers.
pub(super) struct TimAndPinRegister<PIN: TimPin, TIM: ValidTim> {
    pin: PIN,
    tim: TIM,
}

pub(super) struct TimRegister<TIM: ValidTim> {
    tim: TIM,
}

impl<TIM: ValidTim> TimRegister<TIM> {
    #[inline]
    pub(super) unsafe fn new(tim: TIM) -> Self {
        TimRegister { tim }
    }

    pub(super) fn release(self) -> TIM {
        self.tim
    }
}

unsafe impl<TIM: ValidTim> TimRegInterface for TimRegister<TIM> {
    fn tim_id(&self) -> u8 {
        TIM::TIM_ID
    }
}

impl<PIN: TimPin, TIM: ValidTim> TimAndPinRegister<PIN, TIM>
where
    (PIN, TIM): ValidTimAndPin<PIN, TIM>,
{
    #[inline]
    pub(super) unsafe fn new(pin: PIN, tim: TIM) -> Self {
        TimAndPinRegister { pin, tim }
    }

    pub(super) fn release(self) -> (PIN, TIM) {
        (self.pin, self.tim)
    }
}

unsafe impl<PIN: TimPin, TIM: ValidTim> TimRegInterface for TimAndPinRegister<PIN, TIM> {
    #[inline(always)]
    fn tim_id(&self) -> u8 {
        TIM::TIM_ID
    }
}

unsafe impl<PIN: TimPin, TIM: ValidTim> TimPinInterface for TimAndPinRegister<PIN, TIM> {
    #[inline(always)]
    fn pin_id(&self) -> DynPinId {
        PIN::DYN
    }
}

pub(super) struct TimDynRegister {
    tim_id: u8,
    pin_id: DynPinId,
}

impl<PIN: TimPin, TIM: ValidTim> From<TimAndPinRegister<PIN, TIM>> for TimDynRegister {
    fn from(_reg: TimAndPinRegister<PIN, TIM>) -> Self {
        Self {
            tim_id: TIM::TIM_ID,
            pin_id: PIN::DYN,
        }
    }
}

unsafe impl TimRegInterface for TimDynRegister {
    #[inline(always)]
    fn tim_id(&self) -> u8 {
        self.tim_id
    }
}

unsafe impl TimPinInterface for TimDynRegister {
    #[inline(always)]
    fn pin_id(&self) -> DynPinId {
        self.pin_id
    }
}

//==================================================================================================
// Timers
//==================================================================================================

/// Hardware timers
pub struct CountDownTimer<TIM: ValidTim> {
    tim: TimRegister<TIM>,
    curr_freq: Hertz,
    sys_clk: Hertz,
    rst_val: u32,
    last_cnt: u32,
    listening: bool,
}

fn enable_tim_clk(syscfg: &mut SYSCONFIG, idx: u8) {
    syscfg
        .tim_clk_enable
        .modify(|r, w| unsafe { w.bits(r.bits() | (1 << idx)) });
}

unsafe impl<TIM: ValidTim> TimRegInterface for CountDownTimer<TIM> {
    fn tim_id(&self) -> u8 {
        TIM::TIM_ID
    }
}

macro_rules! csd_sel {
    ($func_name:ident, $csd_reg:ident) => {
        /// Configure the Cascade sources
        pub fn $func_name(
            &mut self,
            src: CascadeSource,
            id: Option<u8>,
        ) -> Result<(), TimerErrors> {
            let mut id_num = 0;
            if let CascadeSource::PortABase
            | CascadeSource::PortBBase
            | CascadeSource::ClockDividerBase
            | CascadeSource::TimBase = src
            {
                if id.is_none() {
                    return Err(TimerErrors::InvalidCsdSourceInput);
                }
            }
            if id.is_some() {
                id_num = id.unwrap();
            }
            match src {
                CascadeSource::PortABase => {
                    if id_num > 55 {
                        return Err(TimerErrors::InvalidCsdSourceInput);
                    }
                    self.tim.reg().$csd_reg.write(|w| unsafe {
                        w.cassel().bits(CascadeSource::PortABase as u8 + id_num)
                    });
                    Ok(())
                }
                CascadeSource::PortBBase => {
                    if id_num > 23 {
                        return Err(TimerErrors::InvalidCsdSourceInput);
                    }
                    self.tim.reg().$csd_reg.write(|w| unsafe {
                        w.cassel().bits(CascadeSource::PortBBase as u8 + id_num)
                    });
                    Ok(())
                }
                CascadeSource::TimBase => {
                    if id_num > 23 {
                        return Err(TimerErrors::InvalidCsdSourceInput);
                    }
                    self.tim.reg().$csd_reg.write(|w| unsafe {
                        w.cassel().bits(CascadeSource::TimBase as u8 + id_num)
                    });
                    Ok(())
                }
                CascadeSource::ClockDividerBase => {
                    if id_num > 7 {
                        return Err(TimerErrors::InvalidCsdSourceInput);
                    }
                    self.tim.reg().cascade0.write(|w| unsafe {
                        w.cassel()
                            .bits(CascadeSource::ClockDividerBase as u8 + id_num)
                    });
                    Ok(())
                }
                _ => {
                    self.tim
                        .reg()
                        .$csd_reg
                        .write(|w| unsafe { w.cassel().bits(src as u8) });
                    Ok(())
                }
            }
        }
    };
}

impl<TIM: ValidTim> CountDownTimer<TIM> {
    /// Configures a TIM peripheral as a periodic count down timer
    pub fn new(syscfg: &mut SYSCONFIG, sys_clk: impl Into<Hertz>, tim: TIM) -> Self {
        enable_tim_clk(syscfg, TIM::TIM_ID);
        let cd_timer = CountDownTimer {
            tim: unsafe { TimRegister::new(tim) },
            sys_clk: sys_clk.into(),
            rst_val: 0,
            curr_freq: 0.hz(),
            listening: false,
            last_cnt: 0,
        };
        cd_timer.tim.reg().ctrl.modify(|_, w| w.enable().set_bit());
        cd_timer
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
                irqsel.tim[TIM::TIM_ID as usize].write(|w| unsafe { w.bits(interrupt as u32) });
                self.enable_interrupt();
                self.listening = true;
            }
        }
    }

    pub fn unlisten(&mut self, event: Event, syscfg: &mut SYSCONFIG, irqsel: &mut IRQSEL) {
        match event {
            Event::TimeOut => {
                enable_peripheral_clock(syscfg, PeripheralClocks::Irqsel);
                irqsel.tim[TIM::TIM_ID as usize].write(|w| unsafe { w.bits(IRQ_DST_NONE) });
                self.disable_interrupt();
                self.listening = false;
            }
        }
    }

    #[inline(always)]
    pub fn enable_interrupt(&mut self) {
        self.tim.reg().ctrl.modify(|_, w| w.irq_enb().set_bit());
    }

    #[inline(always)]
    pub fn disable_interrupt(&mut self) {
        self.tim.reg().ctrl.modify(|_, w| w.irq_enb().clear_bit());
    }

    pub fn release(self, syscfg: &mut SYSCONFIG) -> TIM {
        self.tim.reg().ctrl.write(|w| w.enable().clear_bit());
        syscfg
            .tim_clk_enable
            .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << TIM::TIM_ID)) });
        self.tim.release()
    }

    /// Load the count down timer with a timeout but do not start it.
    pub fn load(&mut self, timeout: impl Into<Hertz>) {
        self.tim.reg().ctrl.modify(|_, w| w.enable().clear_bit());
        self.curr_freq = timeout.into();
        self.rst_val = self.sys_clk.0 / self.curr_freq.0;
        unsafe {
            self.tim.reg().rst_value.write(|w| w.bits(self.rst_val));
            self.tim.reg().cnt_value.write(|w| w.bits(self.rst_val));
        }
    }

    #[inline(always)]
    pub fn enable(&mut self) {
        self.tim.reg().ctrl.modify(|_, w| w.enable().set_bit());
    }

    #[inline(always)]
    pub fn disable(&mut self) {
        self.tim.reg().ctrl.modify(|_, w| w.enable().clear_bit());
    }

    /// Disable the counter, setting both enable and active bit to 0
    pub fn auto_disable(self, enable: bool) -> Self {
        if enable {
            self.tim
                .reg()
                .ctrl
                .modify(|_, w| w.auto_disable().set_bit());
        } else {
            self.tim
                .reg()
                .ctrl
                .modify(|_, w| w.auto_disable().clear_bit());
        }
        self
    }

    /// This option only applies when the Auto-Disable functionality is 0.
    ///
    /// The active bit is changed to 0 when count reaches 0, but the counter stays
    /// enabled. When Auto-Disable is 1, Auto-Deactivate is implied
    pub fn auto_deactivate(self, enable: bool) -> Self {
        if enable {
            self.tim
                .reg()
                .ctrl
                .modify(|_, w| w.auto_deactivate().set_bit());
        } else {
            self.tim
                .reg()
                .ctrl
                .modify(|_, w| w.auto_deactivate().clear_bit());
        }
        self
    }

    /// Configure the cascade parameters
    pub fn cascade_control(&mut self, ctrl: CascadeCtrl) {
        self.tim.reg().csd_ctrl.write(|w| {
            w.csden0().bit(ctrl.enb_start_src_csd0);
            w.csdinv0().bit(ctrl.inv_csd0);
            w.csden1().bit(ctrl.enb_start_src_csd1);
            w.csdinv1().bit(ctrl.inv_csd1);
            w.dcasop().bit(ctrl.dual_csd_op);
            w.csdtrg0().bit(ctrl.trg_csd0);
            w.csdtrg1().bit(ctrl.trg_csd1);
            w.csden2().bit(ctrl.enb_stop_src_csd2);
            w.csdinv2().bit(ctrl.inv_csd2);
            w.csdtrg2().bit(ctrl.trg_csd2)
        });
    }

    csd_sel!(cascade_0_source, cascade0);
    csd_sel!(cascade_1_source, cascade1);
    csd_sel!(cascade_2_source, cascade2);

    pub fn curr_freq(&self) -> Hertz {
        self.curr_freq
    }

    pub fn listening(&self) -> bool {
        self.listening
    }
}

/// CountDown implementation for TIMx
impl<TIM: ValidTim> CountDown for CountDownTimer<TIM> {
    type Time = Hertz;

    #[inline]
    fn start<T>(&mut self, timeout: T)
    where
        T: Into<Hertz>,
    {
        self.load(timeout);
        self.enable();
    }

    /// Return `Ok` if the timer has wrapped. Peripheral will automatically clear the
    /// flag and restart the time if configured correctly
    fn wait(&mut self) -> nb::Result<(), Void> {
        let cnt = self.tim.reg().cnt_value.read().bits();
        if (cnt > self.last_cnt) || cnt == 0 {
            self.last_cnt = self.rst_val;
            Ok(())
        } else {
            self.last_cnt = cnt;
            Err(nb::Error::WouldBlock)
        }
    }
}

impl<TIM: ValidTim> Periodic for CountDownTimer<TIM> {}

impl<TIM: ValidTim> Cancel for CountDownTimer<TIM> {
    type Error = TimerErrors;
    fn cancel(&mut self) -> Result<(), Self::Error> {
        if !self.tim.reg().ctrl.read().enable().bit_is_set() {
            return Err(TimerErrors::Canceled);
        }
        self.tim.reg().ctrl.write(|w| w.enable().clear_bit());
        Ok(())
    }
}

/// Delay for microseconds.
///
/// For delays less than 100 us, an assembly delay will be used.
/// For larger delays, the timer peripheral will be used.
/// Please note that the delay using the peripheral might not
/// work properly in debug mode.
impl<TIM: ValidTim> delay::DelayUs<u32> for CountDownTimer<TIM> {
    fn delay_us(&mut self, us: u32) {
        if us < 100 {
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
impl<TIM: ValidTim> delay::DelayUs<u16> for CountDownTimer<TIM> {
    fn delay_us(&mut self, us: u16) {
        self.delay_us(u32::from(us));
    }
}
/// Forwards call to u32 variant of delay
impl<TIM: ValidTim> delay::DelayUs<u8> for CountDownTimer<TIM> {
    fn delay_us(&mut self, us: u8) {
        self.delay_us(u32::from(us));
    }
}

impl<TIM: ValidTim> delay::DelayMs<u32> for CountDownTimer<TIM> {
    fn delay_ms(&mut self, ms: u32) {
        self.start(1000.hz());
        for _ in 0..ms {
            nb::block!(self.wait()).unwrap();
        }
    }
}
impl<TIM: ValidTim> delay::DelayMs<u16> for CountDownTimer<TIM> {
    fn delay_ms(&mut self, ms: u16) {
        self.delay_ms(u32::from(ms));
    }
}
impl<TIM: ValidTim> embedded_hal::blocking::delay::DelayMs<u8> for CountDownTimer<TIM> {
    fn delay_ms(&mut self, ms: u8) {
        self.delay_ms(u32::from(ms));
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
    let mut ms_timer = CountDownTimer::new(syscfg, sys_clk, tim0);
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
