//! # Type-level module for GPIO pins
//!
//! This module provides a type-level API for GPIO pins. It uses the type system
//! to track the state of pins at compile-time. Representing GPIO pins in this
//! manner incurs no run-time overhead. Each [`Pin`] struct is zero-sized, so
//! there is no data to copy around. Instead, real code is generated as a side
//! effect of type transformations, and the resulting assembly is nearly
//! identical to the equivalent, hand-written C.
//!
//! To track the state of pins at compile-time, this module uses traits to
//! represent [type classes] and types as instances of those type classes. For
//! example, the trait [`InputConfig`] acts as a [type-level enum] of the
//! available input configurations, and the types [`Floating`], [`PullDown`] and
//! [`PullUp`] are its type-level variants.
//!
//! Type-level [`Pin`]s are parameterized by two type-level enums, [`PinId`] and
//! [`PinMode`].
//!
//! ```
//! pub struct Pin<I, M>
//! where
//!     I: PinId,
//!     M: PinMode,
//! {
//!     // ...
//! }
//! ```
//!
//! A `PinId` identifies a pin by it's group (A, B) and pin number. Each
//! `PinId` instance is named according to its datasheet identifier, e.g.
//! [`PA02`].
//!
//! A `PinMode` represents the various pin modes. The available `PinMode`
//! variants are [`Input`], [`Output`] and [`Alternate`], each with its own corresponding
//! configurations.
//!
//! It is not possible for users to create new instances of a [`Pin`]. Singleton
//! instances of each pin are made available to users through the [`Pins`]
//! struct.
//!
//! To create the [`PinsA`] or [`PinsB`] struct, users must supply the PAC
//! [`PORTA`](crate::pac::PORTA) or  [`PORTB`](crate::pac::PORTB)peripheral.
//! The struct takes ownership of the port and provides the corresponding pins. Each [`Pin`]
//! within the [`PinsA`] or [`PinsB`] struct can be moved out and used individually.
//! The pins structure can also consume the [`IOCONFIG`] structure optionally by
//! passing it as an option
//!
//! ```
//! let mut peripherals = Peripherals::take().unwrap();
//! let pins = Pins::new(&mut dp.SYSCONFIG, Some(dp.IOCONFIG), dp.PORTA);
//! ```
//!
//! Pins can be converted between modes using several different methods.
//!
//! ```
//! // Use one of the literal function names
//! let pa0 = pins.pa0.into_floating_input();
//! // Use a generic method and one of the `PinMode` variant types
//! let pa0 = pins.pa0.into_mode::<FloatingInput>();
//! // Specify the target type and use `From`/`Into`
//! let pa0: Pin<PA0, FloatingInput> = pins.pa0.into();
//! ```
//!
//! # Embedded HAL traits
//!
//! This module implements all of the embedded HAL GPIO traits for each [`Pin`]
//! in the corresponding [`PinMode`]s, namely: [`InputPin`], [`OutputPin`],
//! [`ToggleableOutputPin`].
//!
//! For example, you can control the logic level of an `OutputPin` like so
//!
//! ```
//! use atsamd_hal::pac::Peripherals;
//! use atsamd_hal::gpio::v2::Pins;
//! use embedded_hal::digital::v2::OutputPin;
//!
//! let mut peripherals = Peripherals::take().unwrap();
//! let mut pins = Pins::new(&mut dp.SYSCONFIG, Some(dp.IOCONFIG), dp.PORTA);
//! pins.pa0.set_high();
//! ```
//!
//! # Type-level features
//!
//! This module also provides additional, type-level tools to work with GPIO
//! pins.
//!
//! The [`AnyPin`] trait defines an [`AnyKind`] type class
//! for all `Pin` types.

use super::dynpins::{DynAlternate, DynGroup, DynInput, DynOutput, DynPinId, DynPinMode};
use super::reg::RegisterInterface;
use crate::{
    pac::{self, IOCONFIG, IRQSEL, PORTA, PORTB, SYSCONFIG},
    typelevel::Is,
    Sealed,
};
use core::convert::Infallible;
use core::marker::PhantomData;
use embedded_hal::digital::v2::{InputPin, OutputPin, ToggleableOutputPin};
use paste::paste;

//==================================================================================================
//  Errors and Definitions
//==================================================================================================

#[derive(Debug, PartialEq)]
pub enum InterruptEdge {
    HighToLow,
    LowToHigh,
    BothEdges,
}

#[derive(Debug, PartialEq)]
pub enum InterruptLevel {
    Low = 0,
    High = 1,
}

#[derive(Debug, PartialEq)]
pub enum PinState {
    Low = 0,
    High = 1,
}

/// GPIO error type
#[derive(Debug, PartialEq)]
pub enum PinError {
    /// The pin did not have the correct ID or mode for the requested operation.
    /// [`DynPin`](crate::gpio::DynPin)s are not tracked and verified at compile-time, so run-time
    /// operations are fallible.
    InvalidPinType,
    IsMasked,
}

//==================================================================================================
// Input configuration
//==================================================================================================

/// Type-level enum for input configurations
///
/// The valid options are [`Floating`], [`PullDown`] and [`PullUp`].
pub trait InputConfig: Sealed {
    /// Corresponding [`DynInput`](super::DynInput)
    const DYN: DynInput;
}

pub enum Floating {}
pub enum PullDown {}
pub enum PullUp {}

impl InputConfig for Floating {
    const DYN: DynInput = DynInput::Floating;
}
impl InputConfig for PullDown {
    const DYN: DynInput = DynInput::PullDown;
}
impl InputConfig for PullUp {
    const DYN: DynInput = DynInput::PullUp;
}

impl Sealed for Floating {}
impl Sealed for PullDown {}
impl Sealed for PullUp {}

/// Type-level variant of [`PinMode`] for floating input mode
pub type InputFloating = Input<Floating>;
/// Type-level variant of [`PinMode`] for pull-down input mode
pub type InputPullDown = Input<PullDown>;
/// Type-level variant of [`PinMode`] for pull-up input mode
pub type InputPullUp = Input<PullUp>;

/// Type-level variant of [`PinMode`] for input modes
///
/// Type `C` is one of three input configurations: [`Floating`], [`PullDown`] or
/// [`PullUp`]
pub struct Input<C: InputConfig> {
    cfg: PhantomData<C>,
}

impl<C: InputConfig> Sealed for Input<C> {}

#[derive(Debug, PartialEq)]
pub enum FilterType {
    SystemClock = 0,
    DirectInputWithSynchronization = 1,
    FilterOneClockCycle = 2,
    FilterTwoClockCycles = 3,
    FilterThreeClockCycles = 4,
    FilterFourClockCycles = 5,
}

pub use crate::clock::FilterClkSel;

//==================================================================================================
// Output configuration
//==================================================================================================

pub trait OutputConfig: Sealed {
    const DYN: DynOutput;
}

/// Type-level variant of [`OutputConfig`] for a push-pull configuration
pub enum PushPull {}
/// Type-level variant of [`OutputConfig`] for an open drain configuration
pub enum OpenDrain {}

/// Type-level variant of [`OutputConfig`] for a readable push-pull configuration
pub enum ReadablePushPull {}
/// Type-level variant of [`OutputConfig`] for a readable open-drain configuration
pub enum ReadableOpenDrain {}

impl Sealed for PushPull {}
impl Sealed for OpenDrain {}
impl Sealed for ReadableOpenDrain {}
impl Sealed for ReadablePushPull {}

impl OutputConfig for PushPull {
    const DYN: DynOutput = DynOutput::PushPull;
}
impl OutputConfig for OpenDrain {
    const DYN: DynOutput = DynOutput::OpenDrain;
}
impl OutputConfig for ReadablePushPull {
    const DYN: DynOutput = DynOutput::ReadablePushPull;
}
impl OutputConfig for ReadableOpenDrain {
    const DYN: DynOutput = DynOutput::ReadableOpenDrain;
}

/// Type-level variant of [`PinMode`] for output modes
///
/// Type `C` is one of two output configurations: [`PushPull`] or [`Readable`]
pub struct Output<C: OutputConfig> {
    cfg: PhantomData<C>,
}

impl<C: OutputConfig> Sealed for Output<C> {}

/// Type-level variant of [`PinMode`] for push-pull output mode
pub type PushPullOutput = Output<PushPull>;
/// Type-level variant of [`PinMode`] for open drain output mode
pub type OutputOpenDrain = Output<OpenDrain>;

pub type OutputReadablePushPull = Output<ReadablePushPull>;
pub type OutputReadableOpenDrain = Output<ReadableOpenDrain>;

//==================================================================================================
//  Alternate configurations
//==================================================================================================

/// Type-level enum for alternate peripheral function configurations
pub trait AlternateConfig: Sealed {
    const DYN: DynAlternate;
}

pub enum Funsel1 {}
pub enum Funsel2 {}
pub enum Funsel3 {}

impl AlternateConfig for Funsel1 {
    const DYN: DynAlternate = DynAlternate::Funsel1;
}
impl AlternateConfig for Funsel2 {
    const DYN: DynAlternate = DynAlternate::Funsel2;
}
impl AlternateConfig for Funsel3 {
    const DYN: DynAlternate = DynAlternate::Funsel3;
}

impl Sealed for Funsel1 {}
impl Sealed for Funsel2 {}
impl Sealed for Funsel3 {}

/// Type-level variant of [`PinMode`] for alternate peripheral functions
///
/// Type `C` is an [`AlternateConfig`]
pub struct Alternate<C: AlternateConfig> {
    cfg: PhantomData<C>,
}

impl<C: AlternateConfig> Sealed for Alternate<C> {}

pub type AltFunc1 = Alternate<Funsel1>;
pub type AltFunc2 = Alternate<Funsel2>;
pub type AltFunc3 = Alternate<Funsel3>;

/// Type alias for the [`PinMode`] at reset
pub type Reset = InputFloating;

//==================================================================================================
//  Pin modes
//==================================================================================================

/// Type-level enum representing pin modes
///
/// The valid options are [`Input`], [`Output`] and [`Alternate`].
pub trait PinMode: Sealed {
    /// Corresponding [`DynPinMode`](super::DynPinMode)
    const DYN: DynPinMode;
}

impl<C: InputConfig> PinMode for Input<C> {
    const DYN: DynPinMode = DynPinMode::Input(C::DYN);
}
impl<C: OutputConfig> PinMode for Output<C> {
    const DYN: DynPinMode = DynPinMode::Output(C::DYN);
}
impl<C: AlternateConfig> PinMode for Alternate<C> {
    const DYN: DynPinMode = DynPinMode::Alternate(C::DYN);
}

//==================================================================================================
//  Pin IDs
//==================================================================================================

/// Type-level enum for pin IDs
pub trait PinId: Sealed {
    /// Corresponding [`DynPinId`](super::DynPinId)
    const DYN: DynPinId;
}

macro_rules! pin_id {
    ($Group:ident, $Id:ident, $NUM:literal) => {
        // Need paste macro to use ident in doc attribute
        paste! {
            #[doc = "Pin ID representing pin " $Id]
            pub enum $Id {}
            impl Sealed for $Id {}
            impl PinId for $Id {
                const DYN: DynPinId = DynPinId {
                    group: DynGroup::$Group,
                    num: $NUM,
                };
            }
        }
    };
}

//==================================================================================================
//  Pin
//==================================================================================================

/// A type-level GPIO pin, parameterized by [`PinId`] and [`PinMode`] types

pub struct Pin<I: PinId, M: PinMode> {
    pub(in crate::gpio) regs: Registers<I>,
    mode: PhantomData<M>,
}

impl<I: PinId, M: PinMode> Sealed for Pin<I, M> {}

impl<I: PinId, M: PinMode> AnyPin for Pin<I, M> {
    type Id = I;
    type Mode = M;
}

macro_rules! common_reg_if_functions {
    () => {
        paste!(
            #[inline]
            pub fn datamask(&self) -> bool {
                self.regs.datamask()
            }

            #[inline]
            pub fn clear_datamask(self) -> Self {
                self.regs.clear_datamask();
                self
            }

            #[inline]
            pub fn set_datamask(self) -> Self {
                self.regs.set_datamask();
                self
            }

            #[inline]
            pub fn is_high_masked(&self) -> Result<bool, PinError> {
                self.regs.read_pin_masked()
            }

            #[inline]
            pub fn is_low_masked(&self) -> Result<bool, PinError> {
                self.regs.read_pin_masked().map(|v| !v)
            }

            #[inline]
            pub fn set_high_masked(&mut self) -> Result<(), PinError> {
                self.regs.write_pin_masked(true)
            }

            #[inline]
            pub fn set_low_masked(&mut self) -> Result<(), PinError> {
                self.regs.write_pin_masked(false)
            }

            fn _irq_enb(
                &mut self,
                syscfg: Option<&mut va108xx::SYSCONFIG>,
                irqsel: &mut va108xx::IRQSEL,
                interrupt: va108xx::Interrupt,
            ) {
                if syscfg.is_some() {
                    crate::clock::enable_peripheral_clock(
                        syscfg.unwrap(),
                        crate::clock::PeripheralClocks::Irqsel,
                    );
                }
                self.regs.enable_irq();
                match self.regs.id().group {
                    // Set the correct interrupt number in the IRQSEL register
                    DynGroup::A => {
                        irqsel.porta[self.regs.id().num as usize]
                            .write(|w| unsafe { w.bits(interrupt as u32) });
                    }
                    DynGroup::B => {
                        irqsel.portb[self.regs.id().num as usize]
                            .write(|w| unsafe { w.bits(interrupt as u32) });
                    }
                }
            }
        );
    };
}

pub(crate) use common_reg_if_functions;

impl<I: PinId, M: PinMode> Pin<I, M> {
    /// Create a new [`Pin`]
    ///
    /// # Safety
    ///
    /// Each [`Pin`] must be a singleton. For a given [`PinId`], there must be
    /// at most one corresponding [`Pin`] in existence at any given time.
    /// Violating this requirement is `unsafe`.
    #[inline]
    pub(crate) unsafe fn new() -> Pin<I, M> {
        Pin {
            regs: Registers::new(),
            mode: PhantomData,
        }
    }

    /// Convert the pin to the requested [`PinMode`]
    #[inline]
    pub fn into_mode<N: PinMode>(mut self) -> Pin<I, N> {
        // Only modify registers if we are actually changing pin mode
        // This check should compile away
        if N::DYN != M::DYN {
            self.regs.change_mode::<N>();
        }
        // Safe because we drop the existing Pin
        unsafe { Pin::new() }
    }

    /// Configure the pin for function select 1. See Programmer Guide p.40 for the function table
    #[inline]
    pub fn into_funsel_1(self) -> Pin<I, AltFunc1> {
        self.into_mode()
    }

    /// Configure the pin for function select 2. See Programmer Guide p.40 for the function table
    #[inline]
    pub fn into_funsel_2(self) -> Pin<I, AltFunc2> {
        self.into_mode()
    }

    /// Configure the pin for function select 3. See Programmer Guide p.40 for the function table
    #[inline]
    pub fn into_funsel_3(self) -> Pin<I, AltFunc3> {
        self.into_mode()
    }

    /// Configure the pin to operate as a floating input
    #[inline]
    pub fn into_floating_input(self) -> Pin<I, InputFloating> {
        self.into_mode()
    }

    /// Configure the pin to operate as a pulled down input
    #[inline]
    pub fn into_pull_down_input(self) -> Pin<I, InputPullDown> {
        self.into_mode()
    }

    /// Configure the pin to operate as a pulled up input
    #[inline]
    pub fn into_pull_up_input(self) -> Pin<I, InputPullUp> {
        self.into_mode()
    }

    /// Configure the pin to operate as a push-pull output
    #[inline]
    pub fn into_push_pull_output(self) -> Pin<I, PushPullOutput> {
        self.into_mode()
    }

    /// Configure the pin to operate as a readable push-pull output
    #[inline]
    pub fn into_readable_push_pull_output(self) -> Pin<I, OutputReadablePushPull> {
        self.into_mode()
    }

    /// Configure the pin to operate as a readable open-drain output
    #[inline]
    pub fn into_readable_open_drain_output(self) -> Pin<I, OutputReadableOpenDrain> {
        self.into_mode()
    }

    common_reg_if_functions!();

    #[inline]
    pub(crate) fn _set_high(&mut self) {
        self.regs.write_pin(true)
    }

    #[inline]
    pub(crate) fn _set_low(&mut self) {
        self.regs.write_pin(false)
    }

    #[inline]
    pub(crate) fn _toggle(&mut self) {
        self.regs.toggle();
    }

    #[inline]
    pub(crate) fn _is_low(&self) -> bool {
        !self.regs.read_pin()
    }

    #[inline]
    pub(crate) fn _is_high(&self) -> bool {
        self.regs.read_pin()
    }
}

pub type SpecificPin<P> = Pin<<P as AnyPin>::Id, <P as AnyPin>::Mode>;

//==================================================================================================
//  AnyPin
//==================================================================================================

/// Type class for [`Pin`] types
///
/// This trait uses the [`AnyKind`] trait pattern to create a [type class] for
/// [`Pin`] types. See the `AnyKind` documentation for more details on the
/// pattern.
pub trait AnyPin: Is<Type = SpecificPin<Self>> {
    type Id: PinId;
    type Mode: PinMode;
}

impl<I: PinId, M: PinMode> AsRef<Self> for Pin<I, M> {
    #[inline]
    fn as_ref(&self) -> &Self {
        self
    }
}

impl<I: PinId, M: PinMode> AsMut<Self> for Pin<I, M> {
    #[inline]
    fn as_mut(&mut self) -> &mut Self {
        self
    }
}

//==================================================================================================
//  Additional functionality
//==================================================================================================

impl<I: PinId, C: InputConfig> Pin<I, Input<C>> {
    pub fn interrupt_edge(
        mut self,
        edge_type: InterruptEdge,
        syscfg: Option<&mut SYSCONFIG>,
        irqsel: &mut IRQSEL,
        interrupt: pac::Interrupt,
    ) -> Self {
        self._irq_enb(syscfg, irqsel, interrupt);
        self.regs.interrupt_edge(edge_type);
        self
    }

    pub fn interrupt_level(
        mut self,
        level_type: InterruptLevel,
        syscfg: Option<&mut SYSCONFIG>,
        irqsel: &mut IRQSEL,
        interrupt: pac::Interrupt,
    ) -> Self {
        self._irq_enb(syscfg, irqsel, interrupt);
        self.regs.interrupt_level(level_type);
        self
    }
}

impl<I: PinId, C: OutputConfig> Pin<I, Output<C>> {
    /// See p.53 of the programmers guide for more information.
    /// Possible delays in clock cycles:
    ///  - Delay 1: 1
    ///  - Delay 2: 2
    ///  - Delay 1 + Delay 2: 3
    #[inline]
    pub fn delay(self, delay_1: bool, delay_2: bool) -> Self {
        self.regs.delay(delay_1, delay_2);
        self
    }

    /// See p.52 of the programmers guide for more information.
    /// When configured for pulse mode, a given pin will set the non-default state for exactly
    /// one clock cycle before returning to the configured default state
    pub fn pulse_mode(self, enable: bool, default_state: PinState) -> Self {
        self.regs.pulse_mode(enable, default_state);
        self
    }

    pub fn interrupt_edge(
        mut self,
        edge_type: InterruptEdge,
        syscfg: Option<&mut SYSCONFIG>,
        irqsel: &mut IRQSEL,
        interrupt: pac::Interrupt,
    ) -> Self {
        self._irq_enb(syscfg, irqsel, interrupt);
        self.regs.interrupt_edge(edge_type);
        self
    }

    pub fn interrupt_level(
        mut self,
        level_type: InterruptLevel,
        syscfg: Option<&mut SYSCONFIG>,
        irqsel: &mut IRQSEL,
        interrupt: pac::Interrupt,
    ) -> Self {
        self._irq_enb(syscfg, irqsel, interrupt);
        self.regs.interrupt_level(level_type);
        self
    }
}

impl<I: PinId, C: InputConfig> Pin<I, Input<C>> {
    /// See p.37 and p.38 of the programmers guide for more information.
    #[inline]
    pub fn filter_type(self, filter: FilterType, clksel: FilterClkSel) -> Self {
        self.regs.filter_type(filter, clksel);
        self
    }
}

//==================================================================================================
//  Embedded HAL traits
//==================================================================================================

impl<I: PinId, C: OutputConfig> OutputPin for Pin<I, Output<C>> {
    type Error = Infallible;

    #[inline]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self._set_high();
        Ok(())
    }

    #[inline]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self._set_low();
        Ok(())
    }
}

impl<I: PinId, C: OutputConfig> ToggleableOutputPin for Pin<I, Output<C>> {
    type Error = Infallible;

    #[inline]
    fn toggle(&mut self) -> Result<(), Self::Error> {
        self._toggle();
        Ok(())
    }
}

impl<I: PinId, C: InputConfig> InputPin for Pin<I, Input<C>> {
    type Error = Infallible;

    #[inline]
    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(self._is_high())
    }
    #[inline]
    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(self._is_low())
    }
}

impl<I: PinId> InputPin for Pin<I, OutputReadableOpenDrain> {
    type Error = Infallible;

    #[inline]
    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(self._is_high())
    }

    #[inline]
    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(self._is_low())
    }
}

impl<I: PinId> InputPin for Pin<I, OutputReadablePushPull> {
    type Error = Infallible;

    #[inline]
    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(self._is_high())
    }

    #[inline]
    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(self._is_low())
    }
}

//==================================================================================================
//  Registers
//==================================================================================================

/// Provide a safe register interface for [`Pin`]s
///
/// This `struct` takes ownership of a [`PinId`] and provides an API to
/// access the corresponding regsiters.
pub(in crate::gpio) struct Registers<I: PinId> {
    id: PhantomData<I>,
}

// [`Registers`] takes ownership of the [`PinId`], and [`Pin`] guarantees that
// each pin is a singleton, so this implementation is safe.
unsafe impl<I: PinId> RegisterInterface for Registers<I> {
    #[inline]
    fn id(&self) -> DynPinId {
        I::DYN
    }
}

impl<I: PinId> Registers<I> {
    /// Create a new instance of [`Registers`]
    ///
    /// # Safety
    ///
    /// Users must never create two simultaneous instances of this `struct` with
    /// the same [`PinId`]
    #[inline]
    unsafe fn new() -> Self {
        Registers { id: PhantomData }
    }

    /// Provide a type-level equivalent for the
    /// [`RegisterInterface::change_mode`] method.
    #[inline]
    pub(in crate::gpio) fn change_mode<M: PinMode>(&mut self) {
        RegisterInterface::change_mode(self, M::DYN);
    }
}

//==================================================================================================
//  Pin definitions
//==================================================================================================

macro_rules! pins {
    (
        $Port:ident, $PinsName:ident, $($Id:ident,)+,
    ) => {
        paste!(
            /// Collection of all the individual [`Pin`]s for a given port (PORTA or PORTB)
            pub struct $PinsName {
                iocfg: Option<IOCONFIG>,
                port: $Port,
                $(
                    #[doc = "Pin " $Id]
                    pub [<$Id:lower>]: Pin<$Id, Reset>,
                )+
            }

            impl $PinsName {
                /// Create a new struct containing all the Pins. Passing the IOCONFIG peripheral
                /// is optional because it might be required to create pin definitions for both
                /// ports.
                #[inline]
                pub fn new(
                    syscfg: &mut SYSCONFIG,
                    iocfg: Option<IOCONFIG>,
                    port: $Port
                ) -> $PinsName {
                    syscfg.peripheral_clk_enable.modify(|_, w| {
                        w.[<$Port:lower>]().set_bit();
                        w.gpio().set_bit();
                        w.ioconfig().set_bit()
                    });
                    $PinsName {
                        iocfg,
                        port,
                        // Safe because we only create one `Pin` per `PinId`
                        $(
                            [<$Id:lower>]: unsafe { Pin::new() },
                        )+
                    }
                }

                /// Get the peripheral ID
                /// Safety: Read-only register
                pub fn get_perid() -> u32 {
                    let port = unsafe { &(*$Port::ptr()) };
                    port.perid.read().bits()
                }

                /// Consumes the Pins struct and returns the port definitions
                pub fn release(self) -> (Option<IOCONFIG>, $Port) {
                    (self.iocfg, self.port)
                }
            }
        );
    }
}

macro_rules! declare_pins {
    (
        $Group:ident, $PinsName:ident, $Port:ident, [$(($Id:ident, $NUM:literal),)+]
    ) => {
        pins!($Port, $PinsName, $($Id,)+,);
        $(
            pin_id!($Group, $Id, $NUM);
        )+
    }
}

declare_pins!(
    A,
    PinsA,
    PORTA,
    [
        (PA0, 0),
        (PA1, 1),
        (PA2, 2),
        (PA3, 3),
        (PA4, 4),
        (PA5, 5),
        (PA6, 6),
        (PA7, 7),
        (PA8, 8),
        (PA9, 9),
        (PA10, 10),
        (PA11, 11),
        (PA12, 12),
        (PA13, 13),
        (PA14, 14),
        (PA15, 15),
        (PA16, 16),
        (PA17, 17),
        (PA18, 18),
        (PA19, 19),
        (PA20, 20),
        (PA21, 21),
        (PA22, 22),
        (PA23, 23),
        (PA24, 24),
        (PA25, 25),
        (PA26, 26),
        (PA27, 27),
        (PA28, 28),
        (PA29, 29),
        (PA30, 30),
        (PA31, 31),
    ]
);

declare_pins!(
    B,
    PinsB,
    PORTB,
    [
        (PB0, 0),
        (PB1, 1),
        (PB2, 2),
        (PB3, 3),
        (PB4, 4),
        (PB5, 5),
        (PB6, 6),
        (PB7, 7),
        (PB8, 8),
        (PB9, 9),
        (PB10, 10),
        (PB11, 11),
        (PB12, 12),
        (PB13, 13),
        (PB14, 14),
        (PB15, 15),
        (PB16, 16),
        (PB17, 17),
        (PB18, 18),
        (PB19, 19),
        (PB20, 20),
        (PB21, 21),
        (PB22, 22),
        (PB23, 23),
    ]
);
