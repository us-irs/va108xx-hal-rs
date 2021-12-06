//! # API for the GPIO peripheral
//!
//! The implementation of this GPIO module is heavily based on the
//! [ATSAMD HAL implementation](https://docs.rs/atsamd-hal/0.13.0/atsamd_hal/gpio/v2/index.html).
//!
//! This API provides two different submodules, [`mod@pins`] and [`dynpins`],
//! representing two different ways to handle GPIO pins. The default, [`mod@pins`],
//! is a type-level API that tracks the state of each pin at compile-time. The
//! alternative, [`dynpins`] is a type-erased, value-level API that tracks the
//! state of each pin at run-time.
//!
//! The type-level API is strongly preferred. By representing the state of each
//! pin within the type system, the compiler can detect logic errors at
//! compile-time. Furthermore, the type-level API has absolutely zero run-time
//! cost.
//!
//! If needed, [`dynpins`] can be used to erase the type-level differences
//! between pins. However, by doing so, pins must now be tracked at run-time,
//! and each pin has a non-zero memory footprint.
//!
//! ## Examples
//!
//! - [Blinky example](https://egit.irs.uni-stuttgart.de/rust/va108xx-hal/src/branch/main/examples/blinky.rs)
pub mod dynpins;
pub use dynpins::*;

pub mod pins;
pub use pins::*;

mod reg;
