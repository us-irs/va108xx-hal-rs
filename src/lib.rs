#![no_std]

pub use va108xx;
pub use va108xx as pac;

pub mod clock;
pub mod gpio;
pub mod i2c;
pub mod prelude;
pub mod pwm;
pub mod spi;
pub mod time;
pub mod timer;
pub mod typelevel;
pub mod uart;
pub mod utility;

mod private {
    /// Super trait used to mark traits with an exhaustive set of
    /// implementations
    pub trait Sealed {}
}

pub(crate) use private::Sealed;
