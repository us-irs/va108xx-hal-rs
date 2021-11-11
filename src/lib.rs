#![no_std]

pub use va108xx;

pub mod clock;
pub mod gpio;
pub mod prelude;
pub mod time;
pub mod timer;
pub mod typelevel;
pub mod uart;

pub use va108xx as pac;

mod private {
    /// Super trait used to mark traits with an exhaustive set of
    /// implementations
    pub trait Sealed {}
}

pub(crate) use private::Sealed;
