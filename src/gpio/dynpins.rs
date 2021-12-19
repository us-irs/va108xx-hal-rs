//! # Type-erased, value-level module for GPIO pins
//!
//! Although the type-level API is generally preferred, it is not suitable in
//! all cases. Because each pin is represented by a distinct type, it is not
//! possible to store multiple pins in a homogeneous data structure. The
//! value-level API solves this problem by erasing the type information and
//! tracking the pin at run-time.
//!
//! Value-level pins are represented by the [`DynPin`] type. [`DynPin`] has two
//! fields, `id` and `mode` with types [`DynPinId`] and [`DynPinMode`]
//! respectively. The implementation of these types closely mirrors the
//! type-level API.
//!
//! Instances of [`DynPin`] cannot be created directly. Rather, they must be
//! created from their type-level equivalents using [`From`]/[`Into`].
//!
//! ```
//! // Move a pin out of the Pins struct and convert to a DynPin
//! let pa0: DynPin = pins.pa0.into();
//! ```
//!
//! Conversions between pin modes use a value-level version of the type-level
//! API.
//!
//! ```
//! // Use one of the literal function names
//! pa0.into_floating_input();
//! // Use a method and a DynPinMode variant
//! pa0.into_mode(DYN_FLOATING_INPUT);
//! ```
//!
//! Because the pin state cannot be tracked at compile-time, many [`DynPin`]
//! operations become fallible. Run-time checks are inserted to ensure that
//! users don't try to, for example, set the output level of an input pin.
//!
//! Users may try to convert value-level pins back to their type-level
//! equivalents. However, this option is fallible, because the compiler cannot
//! guarantee the pin has the correct ID or is in the correct mode at
//! compile-time. Use [`TryFrom`](core::convert::TryFrom)/
//! [`TryInto`](core::convert::TryInto) for this conversion.
//!
//! ```
//! // Convert to a `DynPin`
//! let pa0: DynPin = pins.pa0.into();
//! // Change pin mode
//! pa0.into_floating_input();
//! // Convert back to a `Pin`
//! let pa0: Pin<PA0, FloatingInput> = pa0.try_into().unwrap();
//! ```
//!
//! # Embedded HAL traits
//!
//! This module implements all of the embedded HAL GPIO traits for [`DynPin`].
//! However, whereas the type-level API uses
//! `Error = core::convert::Infallible`, the value-level API can return a real
//! error. If the [`DynPin`] is not in the correct [`DynPinMode`] for the
//! operation, the trait functions will return
//! [`InvalidPinType`](PinError::InvalidPinType).

use super::{
    pins::{
        common_reg_if_functions, FilterType, InterruptEdge, InterruptLevel, Pin, PinError, PinId,
        PinMode, PinState,
    },
    reg::RegisterInterface,
};
use crate::{
    clock::FilterClkSel,
    pac::{IRQSEL, SYSCONFIG},
    utility::{Funsel, IrqCfg},
};
use embedded_hal::digital::v2::{InputPin, OutputPin, ToggleableOutputPin};
use paste::paste;

//==================================================================================================
//  DynPinMode configurations
//==================================================================================================

/// Value-level `enum` for disabled configurations
#[derive(PartialEq, Eq, Clone, Copy)]
pub enum DynDisabled {
    Floating,
    PullDown,
    PullUp,
}

/// Value-level `enum` for input configurations
#[derive(PartialEq, Eq, Clone, Copy)]
pub enum DynInput {
    Floating,
    PullDown,
    PullUp,
}

/// Value-level `enum` for output configurations
#[derive(PartialEq, Eq, Clone, Copy)]
pub enum DynOutput {
    PushPull,
    OpenDrain,
    ReadablePushPull,
    ReadableOpenDrain,
}

pub type DynAlternate = Funsel;

//==================================================================================================
//  DynPinMode
//==================================================================================================

/// Value-level `enum` representing pin modes
#[derive(PartialEq, Eq, Clone, Copy)]
pub enum DynPinMode {
    Input(DynInput),
    Output(DynOutput),
    Alternate(DynAlternate),
}

/// Value-level variant of [`DynPinMode`] for floating input mode
pub const DYN_FLOATING_INPUT: DynPinMode = DynPinMode::Input(DynInput::Floating);
/// Value-level variant of [`DynPinMode`] for pull-down input mode
pub const DYN_PULL_DOWN_INPUT: DynPinMode = DynPinMode::Input(DynInput::PullDown);
/// Value-level variant of [`DynPinMode`] for pull-up input mode
pub const DYN_PULL_UP_INPUT: DynPinMode = DynPinMode::Input(DynInput::PullUp);

/// Value-level variant of [`DynPinMode`] for push-pull output mode
pub const DYN_PUSH_PULL_OUTPUT: DynPinMode = DynPinMode::Output(DynOutput::PushPull);
/// Value-level variant of [`DynPinMode`] for open-drain output mode
pub const DYN_OPEN_DRAIN_OUTPUT: DynPinMode = DynPinMode::Output(DynOutput::OpenDrain);
/// Value-level variant of [`DynPinMode`] for readable push-pull output mode
pub const DYN_RD_PUSH_PULL_OUTPUT: DynPinMode = DynPinMode::Output(DynOutput::ReadablePushPull);
/// Value-level variant of [`DynPinMode`] for readable opendrain output mode
pub const DYN_RD_OPEN_DRAIN_OUTPUT: DynPinMode = DynPinMode::Output(DynOutput::ReadableOpenDrain);

/// Value-level variant of [`DynPinMode`] for function select 1
pub const DYN_ALT_FUNC_1: DynPinMode = DynPinMode::Alternate(DynAlternate::Funsel1);
/// Value-level variant of [`DynPinMode`] for function select 2
pub const DYN_ALT_FUNC_2: DynPinMode = DynPinMode::Alternate(DynAlternate::Funsel2);
/// Value-level variant of [`DynPinMode`] for function select 3
pub const DYN_ALT_FUNC_3: DynPinMode = DynPinMode::Alternate(DynAlternate::Funsel3);

//==================================================================================================
//  DynGroup & DynPinId
//==================================================================================================

/// Value-level `enum` for pin groups
#[derive(PartialEq, Clone, Copy)]
pub enum DynGroup {
    A,
    B,
}

/// Value-level `struct` representing pin IDs
#[derive(PartialEq, Clone, Copy)]
pub struct DynPinId {
    pub group: DynGroup,
    pub num: u8,
}

//==================================================================================================
//  DynRegisters
//==================================================================================================

/// Provide a safe register interface for [`DynPin`]s
///
/// This `struct` takes ownership of a [`DynPinId`] and provides an API to
/// access the corresponding regsiters.
struct DynRegisters {
    id: DynPinId,
}

// [`DynRegisters`] takes ownership of the [`DynPinId`], and [`DynPin`]
// guarantees that each pin is a singleton, so this implementation is safe.
unsafe impl RegisterInterface for DynRegisters {
    #[inline]
    fn id(&self) -> DynPinId {
        self.id
    }
}

impl DynRegisters {
    /// Create a new instance of [`DynRegisters`]
    ///
    /// # Safety
    ///
    /// Users must never create two simultaneous instances of this `struct` with
    /// the same [`DynPinId`]
    #[inline]
    unsafe fn new(id: DynPinId) -> Self {
        DynRegisters { id }
    }
}

//==================================================================================================
//  DynPin
//==================================================================================================

/// A value-level pin, parameterized by [`DynPinId`] and [`DynPinMode`]
///
/// This type acts as a type-erased version of [`Pin`]. Every pin is represented
/// by the same type, and pins are tracked and distinguished at run-time.
pub struct DynPin {
    regs: DynRegisters,
    mode: DynPinMode,
}

impl DynPin {
    /// Create a new [`DynPin`]
    ///
    /// # Safety
    ///
    /// Each [`DynPin`] must be a singleton. For a given [`DynPinId`], there
    /// must be at most one corresponding [`DynPin`] in existence at any given
    /// time.  Violating this requirement is `unsafe`.
    #[inline]
    unsafe fn new(id: DynPinId, mode: DynPinMode) -> Self {
        DynPin {
            regs: DynRegisters::new(id),
            mode,
        }
    }

    /// Return a copy of the pin ID
    #[inline]
    pub fn id(&self) -> DynPinId {
        self.regs.id
    }

    /// Return a copy of the pin mode
    #[inline]
    pub fn mode(&self) -> DynPinMode {
        self.mode
    }

    /// Convert the pin to the requested [`DynPinMode`]
    #[inline]
    pub fn into_mode(&mut self, mode: DynPinMode) {
        // Only modify registers if we are actually changing pin mode
        if mode != self.mode {
            self.regs.change_mode(mode);
            self.mode = mode;
        }
    }

    #[inline]
    pub fn into_funsel_1(&mut self) {
        self.into_mode(DYN_ALT_FUNC_1);
    }

    #[inline]
    pub fn into_funsel_2(&mut self) {
        self.into_mode(DYN_ALT_FUNC_2);
    }

    #[inline]
    pub fn into_funsel_3(&mut self) {
        self.into_mode(DYN_ALT_FUNC_3);
    }

    /// Configure the pin to operate as a floating input
    #[inline]
    pub fn into_floating_input(&mut self) {
        self.into_mode(DYN_FLOATING_INPUT);
    }

    /// Configure the pin to operate as a pulled down input
    #[inline]
    pub fn into_pull_down_input(&mut self) {
        self.into_mode(DYN_PULL_DOWN_INPUT);
    }

    /// Configure the pin to operate as a pulled up input
    #[inline]
    pub fn into_pull_up_input(&mut self) {
        self.into_mode(DYN_PULL_UP_INPUT);
    }

    /// Configure the pin to operate as a push-pull output
    #[inline]
    pub fn into_push_pull_output(&mut self) {
        self.into_mode(DYN_PUSH_PULL_OUTPUT);
    }

    /// Configure the pin to operate as a push-pull output
    #[inline]
    pub fn into_open_drain_output(&mut self) {
        self.into_mode(DYN_OPEN_DRAIN_OUTPUT);
    }

    /// Configure the pin to operate as a push-pull output
    #[inline]
    pub fn into_readable_push_pull_output(&mut self) {
        self.into_mode(DYN_RD_PUSH_PULL_OUTPUT);
    }

    /// Configure the pin to operate as a push-pull output
    #[inline]
    pub fn into_readable_open_drain_output(&mut self) {
        self.into_mode(DYN_RD_OPEN_DRAIN_OUTPUT);
    }

    common_reg_if_functions!();

    /// See p.53 of the programmers guide for more information.
    /// Possible delays in clock cycles:
    ///  - Delay 1: 1
    ///  - Delay 2: 2
    ///  - Delay 1 + Delay 2: 3
    #[inline]
    pub fn delay(self, delay_1: bool, delay_2: bool) -> Result<Self, PinError> {
        match self.mode {
            DynPinMode::Output(_) => {
                self.regs.delay(delay_1, delay_2);
                Ok(self)
            }
            _ => Err(PinError::InvalidPinType),
        }
    }

    /// See p.52 of the programmers guide for more information.
    /// When configured for pulse mode, a given pin will set the non-default state for exactly
    /// one clock cycle before returning to the configured default state
    pub fn pulse_mode(self, enable: bool, default_state: PinState) -> Result<Self, PinError> {
        match self.mode {
            DynPinMode::Output(_) => {
                self.regs.pulse_mode(enable, default_state);
                Ok(self)
            }
            _ => Err(PinError::InvalidPinType),
        }
    }

    /// See p.37 and p.38 of the programmers guide for more information.
    #[inline]
    pub fn filter_type(self, filter: FilterType, clksel: FilterClkSel) -> Result<Self, PinError> {
        match self.mode {
            DynPinMode::Input(_) => {
                self.regs.filter_type(filter, clksel);
                Ok(self)
            }
            _ => Err(PinError::InvalidPinType),
        }
    }

    pub fn interrupt_edge(
        mut self,
        edge_type: InterruptEdge,
        irq_cfg: IrqCfg,
        syscfg: Option<&mut SYSCONFIG>,
        irqsel: Option<&mut IRQSEL>,
    ) -> Result<Self, PinError> {
        match self.mode {
            DynPinMode::Input(_) | DynPinMode::Output(_) => {
                self.regs.interrupt_edge(edge_type);
                self.irq_enb(irq_cfg, syscfg, irqsel);
                Ok(self)
            }
            _ => Err(PinError::InvalidPinType),
        }
    }

    pub fn interrupt_level(
        mut self,
        level_type: InterruptLevel,
        irq_cfg: IrqCfg,
        syscfg: Option<&mut SYSCONFIG>,
        irqsel: Option<&mut IRQSEL>,
    ) -> Result<Self, PinError> {
        match self.mode {
            DynPinMode::Input(_) | DynPinMode::Output(_) => {
                self.regs.interrupt_level(level_type);
                self.irq_enb(irq_cfg, syscfg, irqsel);
                Ok(self)
            }
            _ => Err(PinError::InvalidPinType),
        }
    }

    #[inline]
    fn _read(&self) -> Result<bool, PinError> {
        match self.mode {
            DynPinMode::Input(_) | DYN_RD_OPEN_DRAIN_OUTPUT | DYN_RD_PUSH_PULL_OUTPUT => {
                Ok(self.regs.read_pin())
            }
            _ => Err(PinError::InvalidPinType),
        }
    }
    #[inline]
    fn _write(&mut self, bit: bool) -> Result<(), PinError> {
        match self.mode {
            DynPinMode::Output(_) => {
                self.regs.write_pin(bit);
                Ok(())
            }
            _ => Err(PinError::InvalidPinType),
        }
    }
    #[inline]
    fn _toggle(&mut self) -> Result<(), PinError> {
        match self.mode {
            DynPinMode::Output(_) => {
                self.regs.toggle();
                Ok(())
            }
            _ => Err(PinError::InvalidPinType),
        }
    }

    #[inline]
    fn _is_low(&self) -> Result<bool, PinError> {
        self._read().map(|v| !v)
    }
    #[inline]
    fn _is_high(&self) -> Result<bool, PinError> {
        self._read()
    }
    #[inline]
    fn _set_low(&mut self) -> Result<(), PinError> {
        self._write(false)
    }
    #[inline]
    fn _set_high(&mut self) -> Result<(), PinError> {
        self._write(true)
    }
}

//==================================================================================================
//  Convert between Pin and DynPin
//==================================================================================================

impl<I: PinId, M: PinMode> From<Pin<I, M>> for DynPin {
    /// Erase the type-level information in a [`Pin`] and return a value-level
    /// [`DynPin`]
    #[inline]
    fn from(_pin: Pin<I, M>) -> Self {
        // The `Pin` is consumed, so it is safe to replace it with the
        // corresponding `DynPin`
        unsafe { DynPin::new(I::DYN, M::DYN) }
    }
}

impl<I: PinId, M: PinMode> TryFrom<DynPin> for Pin<I, M> {
    type Error = PinError;

    /// Try to recreate a type-level [`Pin`] from a value-level [`DynPin`]
    ///
    /// There is no way for the compiler to know if the conversion will be
    /// successful at compile-time. We must verify the conversion at run-time
    /// or refuse to perform it.
    #[inline]
    fn try_from(pin: DynPin) -> Result<Self, PinError> {
        if pin.regs.id == I::DYN && pin.mode == M::DYN {
            // The `DynPin` is consumed, so it is safe to replace it with the
            // corresponding `Pin`
            Ok(unsafe { Self::new() })
        } else {
            Err(PinError::InvalidPinType)
        }
    }
}

//==================================================================================================
// Embedded HAL traits
//==================================================================================================

impl OutputPin for DynPin {
    type Error = PinError;
    #[inline]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self._set_high()
    }
    #[inline]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self._set_low()
    }
}

impl InputPin for DynPin {
    type Error = PinError;
    #[inline]
    fn is_high(&self) -> Result<bool, Self::Error> {
        self._is_high()
    }
    #[inline]
    fn is_low(&self) -> Result<bool, Self::Error> {
        self._is_low()
    }
}

impl ToggleableOutputPin for DynPin {
    type Error = PinError;
    #[inline]
    fn toggle(&mut self) -> Result<(), Self::Error> {
        self._toggle()
    }
}
