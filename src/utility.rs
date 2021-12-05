//! # API for utility functions like the Error Detection and Correction (EDAC) block
//!
//! Some more information about the recommended scrub rates can be found on the
//! [Vorago White Paper website](https://www.voragotech.com/resources) in the
//! application note AN1212
use va108xx::SYSCONFIG;

#[derive(PartialEq, Debug)]
pub enum UtilityError {
    InvalidCounterResetVal,
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
