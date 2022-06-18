[![Crates.io](https://img.shields.io/crates/v/va108xx-hal)](https://crates.io/crates/va108xx-hal)
[![ci](https://github.com/us-irs/va108xx-hal-rs/actions/workflows/ci.yml/badge.svg)](https://github.com/us-irs/va108xx-hal-rs/actions/workflows/ci.yml)
[![docs.rs](https://img.shields.io/docsrs/va108xx-hal)](https://docs.rs/va108xx-hal)

# HAL for the Vorago VA108xx MCU family

This repository contains the **H**ardware **A**bstraction **L**ayer (HAL), which is an additional
hardware abstraction on top of the [peripheral access API](https://egit.irs.uni-stuttgart.de/rust/va108xx).

It is the result of reading the datasheet for the device and encoding a type-safe layer over the
raw PAC. This crate also implements traits specified by the
[embedded-hal](https://github.com/rust-embedded/embedded-hal) project, making it compatible with
various drivers in the embedded rust ecosystem.

In contrats to other HAL implementations, there is only one chip variant available here so there
is no need to pass the chip variant as a feature.

## Supported Boards

 The first way to use this HAL will probably be with the
 [REB1 development board](https://www.voragotech.com/products/reb1-va108x0-development-board-0).
 The BSP provided for this board also contains instructions how to flash the board.

 | Crate | Version |
|:------|:--------|
[vorago-reb1](https://crates.io/crates/vorago-reb1) | [![Crates.io](https://img.shields.io/crates/v/vorago-reb1)](https://crates.io/crates/vorago-reb1) |

## Building

Building an application requires the `thumbv6m-none-eabi` cross-compiler toolchain.
If you have not installed it yet, you can do so with

```sh
rustup target add thumbv6m-none-eabi
```

After that, you can use `cargo build` to build the development version of the crate.

If you have not done this yet, it is recommended to read some of the excellent resources
available to learn Rust:

- [Rust Embedded Book](https://docs.rust-embedded.org/book/)
- [Rust Discovery Book](https://docs.rust-embedded.org/discovery/)

## Examples

Some examples, which are not specific to a particular board were provided as well.
You can build the timer example with

```sh
cargo build --example timer-ticks
```

## Setting up your own binary crate

If you have a custom board, you might be interested in setting up a new binary crate for your
project. These steps aim to provide a complete list to get a binary crate working to flash
your custom board.

The hello world of embedded development is usually to blinky a LED. This example
is contained within the
[examples folder](https://egit.irs.uni-stuttgart.de/rust/va108xx-hal/src/branch/main/examples/blinky.rs).

1. Set up your Rust cross-compiler if you have not done so yet. See more in the [build chapter](#Building)
2. Create a new binary crate with `cargo init`
3. To ensure that `cargo build` cross-compiles, it is recommended to create a `cargo/config.toml`
   file. A sample `.cargo/config.toml` file is provided in this repository as well
4. Copy the `memory.x` file into your project. This file contains information required by the linker.
5. Copy the `blinky.rs` file to the `src/main.rs` file in your binary crate
6. You need to add some dependencies to your `Cargo.toml` file

   ```toml
	[dependencies]
	cortex-m = "<Compatible Version>"
	cortex-m-rt = "<Compatible Version>"
	panic-halt = "<Compatible Version>"
	embedded-hal = "<Compatible Version>"

	[dependencies.va108xx-hal]
	version = "<Most Recent Version>"
	features = ["rt"]
   ```

6. Build the application with `cargo build`

7. Flashing the board might work differently for different boards and there is usually
   more than one way. You can find example instructions for the REB1 development board
   [here](https://egit.irs.uni-stuttgart.de/rust/vorago-reb1).
