#![no_std]

pub use va108xx;

pub mod clock;
pub mod gpio;
pub mod prelude;
pub mod time;
pub mod timer;
pub mod uart;

pub use va108xx as pac;

mod sealed {
    pub trait Sealed {}
}
