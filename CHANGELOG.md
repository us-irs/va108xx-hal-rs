Change Log
=======

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

## [v0.5.2] 2024-06-16

## Fixed

- Replaced usage to `ptr::write_volatile` in UART module which is denied on more recent Rust
  compilers.

## [v0.5.1]

### Changes

- Updated dependencies:
  - `cortex-m-rtic` (dev-depencency) to 1.1.2
  - `once_cell` to 1.12.0
  - Other dependencies: Only revision has changed

## [v0.5.0]

### Added

- Reactored IRQ handling, so that `unmask` operations can be moved to HAL
- Added UART IRQ handler. Right now, can only perform reception, TX still needs to be done in
  a blocking manner
- Added RTIC template and RTIC UART IRQ application

### Fixed

- Bugfix in UART code where RX and TX could not be enabled or disabled independently

## [v0.4.3]

- Various smaller fixes for READMEs, update of links in documentation
- Simplified CI for github, do not use `cross`
- New `blinky-pac` example
- Use HAL delay in `blinky` example

## [v0.4.2]

### Added

- `port_mux` function to set pin function select manually

### Changed

- Clear TX and RX FIFO in SPI transfer function

## [v0.4.1]

### Fixed

- Initial blockmode setting was not set in SPI constructor

## [v0.4.0]

### Changed

- Replaced `Hertz` by `impl Into<Hertz>` completely and removed
  `+ Copy` where not necessary

## [v0.3.1]

- Updated all links to point to new repository

## [v0.3.0]

### Added

- TIM Cascade example

### Changed

- `CountDownTimer` new function now expects an `impl Into<Hertz>` instead of `Hertz`
- Primary repository now hosted on IRS external git: https://egit.irs.uni-stuttgart.de/rust/va108xx-hal
- Relicensed as Apache-2.0

## [0.2.3]

### Added

- Basic API for EDAC functionality
- PWM implementation and example
- API to perform peripheral resets

### Changed

- Improved Timer API. It is now possible to simply use `new` on `CountDownTimer`

## [0.2.2]

### Added

- DelayUs and DelayMs trait implementations for timer
- SPI implementation for blocking API, supports blockmode as well
- Basic I2C implementation for blocking API

### Changed

- API which expects values in Hertz now uses `impl Into<Hertz>` as input parameter

## [0.2.1]

### Added

- Adds the IRQ interface to configure interrupts on output and input pins
- Utility function to set up millisecond timer with `TIM0`
- Function to set clock divisor registers in `clock` module

### Changed

- Minor optimizations and tweaks for GPIO module
- Moved the `FilterClkSel` struct to the `clock` module, re-exporting in `gpio`
- Clearing output state at initialization of Output pins

## [0.2.0]

### Changed

- New GPIO implementation which uses type-level programming. Implementation heavily based on the
  ATSAMD GPIO HAL: https://docs.rs/atsamd-hal/0.13.0/atsamd_hal/gpio/v2/index.html
- Changes to API, therefore minor version bump

### Added

- UART implementation
- UART example
- Some bugfixes for GPIO implementation
- Rust edition updated to 2021

## [0.1.0]

### Added

- First version of the HAL which adds the GPIO implementation and timer implementation.
- Also adds some examples and helper files to set up new binary crates
- RTT example application
- Added basic test binary in form of an example
- README with basic instructions how to set up own binary crate
