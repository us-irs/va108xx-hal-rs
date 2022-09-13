//! # API for clock related functionality
//!
//! This also includes functionality to enable the peripheral clocks
use crate::time::Hertz;
use crate::utility::PeripheralSelect;
use cortex_m::interrupt::{self, Mutex};
use once_cell::unsync::OnceCell;
use va108xx::SYSCONFIG;

static SYS_CLOCK: Mutex<OnceCell<Hertz>> = Mutex::new(OnceCell::new());

pub type PeripheralClocks = PeripheralSelect;

#[derive(Debug, PartialEq, Eq)]
pub enum FilterClkSel {
    SysClk = 0,
    Clk1 = 1,
    Clk2 = 2,
    Clk3 = 3,
    Clk4 = 4,
    Clk5 = 5,
    Clk6 = 6,
    Clk7 = 7,
}

/// The Vorago in powered by an external clock which might have different frequencies.
/// The clock can be set here so it can be used by other software components as well.
/// The clock can be set exactly once
pub fn set_sys_clock(freq: impl Into<Hertz>) {
    interrupt::free(|cs| {
        SYS_CLOCK.borrow(cs).set(freq.into()).ok();
    })
}

/// Returns the configured system clock
pub fn get_sys_clock() -> Option<Hertz> {
    interrupt::free(|cs| SYS_CLOCK.borrow(cs).get().copied())
}

pub fn set_clk_div_register(syscfg: &mut SYSCONFIG, clk_sel: FilterClkSel, div: u32) {
    match clk_sel {
        FilterClkSel::SysClk => (),
        FilterClkSel::Clk1 => syscfg.ioconfig_clkdiv1.write(|w| unsafe { w.bits(div) }),
        FilterClkSel::Clk2 => syscfg.ioconfig_clkdiv2.write(|w| unsafe { w.bits(div) }),
        FilterClkSel::Clk3 => syscfg.ioconfig_clkdiv3.write(|w| unsafe { w.bits(div) }),
        FilterClkSel::Clk4 => syscfg.ioconfig_clkdiv4.write(|w| unsafe { w.bits(div) }),
        FilterClkSel::Clk5 => syscfg.ioconfig_clkdiv5.write(|w| unsafe { w.bits(div) }),
        FilterClkSel::Clk6 => syscfg.ioconfig_clkdiv6.write(|w| unsafe { w.bits(div) }),
        FilterClkSel::Clk7 => syscfg.ioconfig_clkdiv7.write(|w| unsafe { w.bits(div) }),
    }
}

#[inline]
pub fn enable_peripheral_clock(syscfg: &mut SYSCONFIG, clock: PeripheralClocks) {
    syscfg
        .peripheral_clk_enable
        .modify(|r, w| unsafe { w.bits(r.bits() | (1 << clock as u8)) });
}

#[inline]
pub fn disable_peripheral_clock(syscfg: &mut SYSCONFIG, clock: PeripheralClocks) {
    syscfg
        .peripheral_clk_enable
        .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << clock as u8)) });
}
