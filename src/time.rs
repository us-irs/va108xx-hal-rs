//! Time units
//!
//! See [`Hertz`], [`KiloHertz`] and [`MegaHertz`] for creating increasingly higher frequencies.
//!
//! The [`U32Ext`] trait adds various methods like `.hz()`, `.mhz()`, etc to the `u32` primitive type,
//! allowing it to be converted into frequencies.

/// Bits per second
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Debug)]
pub struct Bps(pub u32);

/// Hertz
///
/// Create a frequency specified in [Hertz](https://en.wikipedia.org/wiki/Hertz).
///
/// See also [`KiloHertz`] and [`MegaHertz`] for semantically correct ways of creating higher
/// frequencies.
///
/// # Examples
///
/// ## Create an 60 Hz frequency
///
/// ```rust
/// use stm32f1xx_hal::time::Hertz;
///
/// let freq = 60.hz();
/// ```
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Debug)]
pub struct Hertz(pub u32);

/// Kilohertz
///
/// Create a frequency specified in kilohertz.
///
/// See also [`Hertz`] and [`MegaHertz`] for semantically correct ways of creating lower or higher
/// frequencies.
///
/// # Examples
///
/// ## Create a 100 Khz frequency
///
/// This example creates a 100 KHz frequency. This could be used to set an I2C data rate or PWM
/// frequency, etc.
///
/// ```rust
/// use stm32f1xx_hal::time::Hertz;
///
/// let freq = 100.khz();
/// ```
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Debug)]
pub struct KiloHertz(pub u32);

/// Megahertz
///
/// Create a frequency specified in megahertz.
///
/// See also [`Hertz`] and [`KiloHertz`] for semantically correct ways of creating lower
/// frequencies.
///
/// # Examples
///
/// ## Create a an 8 MHz frequency
///
/// This example creates an 8 MHz frequency that could be used to configure an SPI peripheral, etc.
///
/// ```rust
/// use stm32f1xx_hal::time::Hertz;
///
/// let freq = 8.mhz();
/// ```
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Debug)]
pub struct MegaHertz(pub u32);

/// Time unit
#[derive(PartialEq, Eq, PartialOrd, Clone, Copy)]
pub struct MilliSeconds(pub u32);

#[derive(PartialEq, Eq, PartialOrd, Clone, Copy)]
pub struct MicroSeconds(pub u32);

/// Extension trait that adds convenience methods to the `u32` type
pub trait U32Ext {
    /// Wrap in `Bps`
    fn bps(self) -> Bps;

    /// Wrap in `Hertz`
    fn hz(self) -> Hertz;

    /// Wrap in `KiloHertz`
    fn khz(self) -> KiloHertz;

    /// Wrap in `MegaHertz`
    fn mhz(self) -> MegaHertz;

    /// Wrap in `MilliSeconds`
    fn ms(self) -> MilliSeconds;

    /// Wrap in `MicroSeconds`
    fn us(self) -> MicroSeconds;
}

impl U32Ext for u32 {
    fn bps(self) -> Bps {
        Bps(self)
    }

    fn hz(self) -> Hertz {
        Hertz(self)
    }

    fn khz(self) -> KiloHertz {
        KiloHertz(self)
    }

    fn mhz(self) -> MegaHertz {
        MegaHertz(self)
    }

    fn ms(self) -> MilliSeconds {
        MilliSeconds(self)
    }

    fn us(self) -> MicroSeconds {
        MicroSeconds(self)
    }
}

impl From<KiloHertz> for Hertz {
    fn from(val: KiloHertz) -> Self {
        Self(val.0 * 1_000)
    }
}

impl From<MegaHertz> for Hertz {
    fn from(val: MegaHertz) -> Self {
        Self(val.0 * 1_000_000)
    }
}

impl From<MegaHertz> for KiloHertz {
    fn from(val: MegaHertz) -> Self {
        Self(val.0 * 1_000)
    }
}

impl From<MilliSeconds> for Hertz {
    fn from(val: MilliSeconds) -> Self {
        Self(1_000 / val.0)
    }
}

impl From<MicroSeconds> for Hertz {
    fn from(val: MicroSeconds) -> Self {
        Self(1_000_000 / val.0)
    }
}
