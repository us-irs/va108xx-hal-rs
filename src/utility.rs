//! # API for utility functions like the Error Detection and Correction (EDAC) block
//!
//! Some more information about the recommended scrub rates can be found on the
//! [Vorago White Paper website](https://www.voragotech.com/resources) in the
//! application note AN1212
use crate::pac;
use va108xx::{IOCONFIG, SYSCONFIG};

#[derive(PartialEq, Debug)]
pub enum UtilityError {
    InvalidCounterResetVal,
    InvalidPin,
}

#[derive(Debug, Eq, Copy, Clone, PartialEq)]
pub enum Funsel {
    Funsel1 = 0b01,
    Funsel2 = 0b10,
    Funsel3 = 0b11,
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum PortSel {
    PortA,
    PortB,
}

#[derive(Copy, Clone, PartialEq)]
pub enum PeripheralSelect {
    PortA = 0,
    PortB = 1,
    Spi0 = 4,
    Spi1 = 5,
    Spi2 = 6,
    Uart0 = 8,
    Uart1 = 9,
    I2c0 = 16,
    I2c1 = 17,
    Irqsel = 21,
    Ioconfig = 22,
    Utility = 23,
    Gpio = 24,
}

/// Generic IRQ config which can be used to specify whether the HAL driver will
/// use the IRQSEL register to route an interrupt, and whether the IRQ will be unmasked in the
/// Cortex-M0 NVIC. Both are generally necessary for IRQs to work, but the user might perform
/// this steps themselves
#[derive(Debug, PartialEq, Clone, Copy)]
pub struct IrqCfg {
    /// Interrupt target vector. Should always be set, might be required for disabling IRQs
    pub irq: pac::Interrupt,
    /// Specfiy whether IRQ should be routed to an IRQ vector using the IRQSEL peripheral
    pub route: bool,
    /// Specify whether the IRQ is unmasked in the Cortex-M NVIC
    pub enable: bool,
}

impl IrqCfg {
    pub fn new(irq: pac::Interrupt, route: bool, enable: bool) -> Self {
        IrqCfg { irq, route, enable }
    }
}

/// Enable scrubbing for the ROM
///
/// Returns [`UtilityError::InvalidCounterResetVal`] if the scrub rate is 0
/// (equivalent to disabling) or larger than 24 bits
pub fn enable_rom_scrubbing(syscfg: &mut SYSCONFIG, scrub_rate: u32) -> Result<(), UtilityError> {
    if scrub_rate == 0 || scrub_rate > u32::pow(2, 24) {
        return Err(UtilityError::InvalidCounterResetVal);
    }
    syscfg.rom_scrub.write(|w| unsafe { w.bits(scrub_rate) });
    Ok(())
}

pub fn disable_rom_scrubbing(syscfg: &mut SYSCONFIG) {
    syscfg.rom_scrub.write(|w| unsafe { w.bits(0) })
}

/// Enable scrubbing for the RAM
///
/// Returns [`UtilityError::InvalidCounterResetVal`] if the scrub rate is 0
/// (equivalent to disabling) or larger than 24 bits
pub fn enable_ram_scrubbing(syscfg: &mut SYSCONFIG, scrub_rate: u32) -> Result<(), UtilityError> {
    if scrub_rate == 0 || scrub_rate > u32::pow(2, 24) {
        return Err(UtilityError::InvalidCounterResetVal);
    }
    syscfg.ram_scrub.write(|w| unsafe { w.bits(scrub_rate) });
    Ok(())
}

pub fn disable_ram_scrubbing(syscfg: &mut SYSCONFIG) {
    syscfg.ram_scrub.write(|w| unsafe { w.bits(0) })
}

/// Clear the reset bit. This register is active low, so doing this will hold the peripheral
/// in a reset state
pub fn clear_reset_bit(syscfg: &mut SYSCONFIG, periph_sel: PeripheralSelect) {
    syscfg
        .peripheral_reset
        .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << periph_sel as u8)) });
}

pub fn set_reset_bit(syscfg: &mut SYSCONFIG, periph_sel: PeripheralSelect) {
    syscfg
        .peripheral_reset
        .modify(|r, w| unsafe { w.bits(r.bits() | (1 << periph_sel as u8)) });
}

/// Can be used to manually manipulate the function select of port pins
pub fn port_mux(
    ioconfig: &mut IOCONFIG,
    port: PortSel,
    pin: u8,
    funsel: Funsel,
) -> Result<(), UtilityError> {
    match port {
        PortSel::PortA => {
            if pin > 31 {
                return Err(UtilityError::InvalidPin);
            }
            ioconfig.porta[pin as usize].modify(|_, w| unsafe { w.funsel().bits(funsel as u8) });
            Ok(())
        }
        PortSel::PortB => {
            if pin > 23 {
                return Err(UtilityError::InvalidPin);
            }
            ioconfig.portb[pin as usize].modify(|_, w| unsafe { w.funsel().bits(funsel as u8) });
            Ok(())
        }
    }
}

/// Unmask and enable an IRQ with the given interrupt number
///
/// ## Safety
///
/// The unmask function can break mask-based critical sections
#[inline]
pub (crate) fn unmask_irq(irq: pac::Interrupt) {
    unsafe { cortex_m::peripheral::NVIC::unmask(irq) };
}
