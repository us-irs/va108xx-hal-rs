use crate::time::Hertz;
use cortex_m::interrupt::{self, Mutex};
use once_cell::unsync::OnceCell;
use va108xx::SYSCONFIG;

static SYS_CLOCK: Mutex<OnceCell<Hertz>> = Mutex::new(OnceCell::new());

pub enum PeripheralClocks {
    PortA = 0,
    PortB = 1,
    Spi0 = 4,
    Spi1 = 5,
    Spi2 = 6,
    UArt0 = 8,
    Uart1 = 9,
    I2c0 = 16,
    I2c1 = 17,
    Irqsel = 21,
    Ioconfig = 22,
    Utility = 23,
    Gpio = 24,
}

/// The Vorago in powered by an external clock which might have different frequencies.
/// The clock can be set here so it can be used by other software components as well.
/// The clock can be set exactly once
pub fn set_sys_clock(freq: Hertz) {
    interrupt::free(|cs| {
        SYS_CLOCK.borrow(cs).set(freq).ok();
    })
}

/// Returns the configured system clock
pub fn get_sys_clock() -> Option<Hertz> {
    interrupt::free(|cs| SYS_CLOCK.borrow(cs).get().copied())
}

pub fn enable_peripheral_clock(syscfg: &mut SYSCONFIG, clock: PeripheralClocks) {
    syscfg
        .peripheral_clk_enable
        .modify(|r, w| unsafe { w.bits(r.bits() | (1 << clock as u8)) });
}

pub fn disable_peripheral_clock(syscfg: &mut SYSCONFIG, clock: PeripheralClocks) {
    syscfg
        .peripheral_clk_enable
        .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << clock as u8)) });
}
