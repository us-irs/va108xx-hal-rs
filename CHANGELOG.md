Change Log
=======

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

## [unreleased]

### Added

- TIM Cascade example

### Changed

- `CountDownTimer` new function now expects an `impl Into<Hertz>` instead of `Hertz`

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
