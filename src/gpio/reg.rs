use super::dynpins::{self, DynGroup, DynPinId, DynPinMode};
use super::pins::{FilterClkSel, FilterType, PinError, PinState};
use va108xx::{ioconfig, porta, IOCONFIG, PORTA, PORTB};

/// Type definition to avoid confusion: These register blocks are identical
type PortRegisterBlock = porta::RegisterBlock;

//==================================================================================================
//  ModeFields
//==================================================================================================

/// Collect all fields needed to set the [`PinMode`](super::PinMode)
#[derive(Default)]
struct ModeFields {
    dir: bool,
    opendrn: bool,
    pull_en: bool,
    /// true for pullup, false for pulldown
    pull_dir: bool,
    funsel: u8,
    enb_input: bool,
}

impl From<DynPinMode> for ModeFields {
    #[inline]
    fn from(mode: DynPinMode) -> Self {
        let mut fields = Self::default();
        use DynPinMode::*;
        match mode {
            Input(config) => {
                use dynpins::DynInput::*;
                fields.dir = false;
                match config {
                    Floating => {
                        fields.pull_en = false;
                    }
                    PullUp => {
                        fields.pull_en = true;
                        fields.pull_dir = true;
                    }
                    PullDown => {
                        fields.pull_en = true;
                        fields.pull_dir = false;
                    }
                }
            }
            Output(config) => {
                use dynpins::DynOutput::*;
                fields.dir = true;
                match config {
                    PushPull => {
                        fields.opendrn = false;
                    }
                    OpenDrain => {
                        fields.opendrn = true;
                    }
                    ReadableOpenDrain => {
                        fields.enb_input = true;
                        fields.opendrn = true;
                    }
                    ReadablePushPull => {
                        fields.enb_input = true;
                        fields.opendrn = false;
                    }
                }
            }
            Alternate(config) => {
                use dynpins::DynAlternate::*;
                match config {
                    Funsel1 => {
                        fields.funsel = 1;
                    }
                    Funsel2 => {
                        fields.funsel = 2;
                    }
                    Funsel3 => {
                        fields.funsel = 3;
                    }
                }
            }
        }
        fields
    }
}
//==================================================================================================
// Register Interface
//==================================================================================================

pub type IocfgPort = ioconfig::PORTA;
#[repr(C)]
pub(super) struct IOCFG_PORT_GROUP {
    port: [IocfgPort; 32],
}

/// Provide a safe register interface for pin objects
///
/// [`PORTA`] and [`PORTB`], like every PAC `struct`, is [`Send`] but not [`Sync`], because it
/// points to a `RegisterBlock` of `VolatileCell`s. Unfortunately, such an
/// interface is quite restrictive. Instead, it would be ideal if we could split
/// the [`PORT`] into independent pins that are both [`Send`] and [`Sync`].
///
/// [`PORT`] is a single, zero-sized marker `struct` that provides access to
/// every [`PORT`] register. Instead, we would like to create zero-sized marker
/// `struct`s for every pin, where each pin is only allowed to control its own
/// registers. Furthermore, each pin `struct` should be a singleton, so that
/// exclusive access to the `struct` also guarantees exclusive access to the
/// corresponding registers. Finally, the pin `struct`s should not have any
/// interior mutability. Together, these requirements would allow the pin
/// `struct`s to be both [`Send`] and [`Sync`].
///
/// This trait creates a safe API for accomplishing these goals. Implementers
/// supply a pin ID through the [`id`] function. The remaining functions provide
/// a safe API for accessing the registers associated with that pin ID. Any
/// modification of the registers requires `&mut self`, which destroys interior
/// mutability.
///
/// # Safety
///
/// Users should only implement the [`id`] function. No default function
/// implementations should be overridden. The implementing type must also have
/// "control" over the corresponding pin ID, i.e. it must guarantee that a each
/// pin ID is a singleton.
///
/// [`id`]: Self::id
pub(super) unsafe trait RegisterInterface {
    /// Provide a [`DynPinId`] identifying the set of registers controlled by
    /// this type.
    fn id(&self) -> DynPinId;

    const PORTA: *const PortRegisterBlock = PORTA::ptr();
    const PORTB: *const PortRegisterBlock = PORTB::ptr();
    const PORT_CFG: *const IOCFG_PORT_GROUP = IOCONFIG::ptr() as *const _;

    /// Change the pin mode
    #[inline]
    fn change_mode(&mut self, mode: DynPinMode) {
        let ModeFields {
            dir,
            funsel,
            opendrn,
            pull_dir,
            pull_en,
            enb_input,
        } = mode.into();
        let (portreg, iocfg) = (self.port_reg(), self.iocfg_port());
        iocfg.port[self.id().num as usize].write(|w| {
            w.opendrn().bit(opendrn);
            w.pen().bit(pull_en);
            w.plevel().bit(pull_dir);
            w.iewo().bit(enb_input);
            unsafe { w.funsel().bits(funsel) }
        });
        let mask = self.mask_32();
        unsafe {
            if dir {
                portreg.dir().modify(|r, w| w.bits(r.bits() | mask));
            } else {
                portreg.dir().modify(|r, w| w.bits(r.bits() & !mask));
            }
        }
    }

    #[inline]
    fn port_reg(&self) -> &PortRegisterBlock {
        match self.id().group {
            DynGroup::A => unsafe { &(*Self::PORTA) },
            DynGroup::B => unsafe { &(*Self::PORTB) },
        }
    }
    fn iocfg_port(&self) -> &IOCFG_PORT_GROUP {
        match self.id().group {
            DynGroup::A => unsafe { &*Self::PORT_CFG },
            DynGroup::B => unsafe { &*Self::PORT_CFG.add(1) },
        }
    }

    #[inline]
    fn mask_32(&self) -> u32 {
        1 << self.id().num
    }

    #[inline]
    fn enable_input_for_output(&self, enable: bool) {
        let iocfg = self.iocfg_port();
        iocfg.port[self.id().num as usize].modify(|_, w| w.iewo().bit(enable))
    }

    /// Set the direction of a pin
    #[inline]
    fn set_dir(&mut self, bit: bool) {
        let portreg = self.port_reg();
        let mask = self.mask_32();
        // Safety: Only the bit for this Pin ID is modified
        unsafe {
            if bit {
                portreg.dir().modify(|r, w| w.bits(r.bits() | mask));
            } else {
                portreg.dir().modify(|r, w| w.bits(r.bits() & !mask));
            }
        }
    }

    fn get_perid(&self) -> u32 {
        let portreg = self.port_reg();
        portreg.perid.read().bits()
    }

    /// Read the logic level of an output pin
    fn read_pin(&self) -> bool {
        let portreg = self.port_reg();
        ((portreg.datainraw().read().bits() >> self.id().num) & 0x01) == 1
    }

    // Get DATAMASK bit for this particular pin
    #[inline(always)]
    fn datamask(&self) -> bool {
        let portreg = self.port_reg();
        (portreg.datamask().read().bits() >> self.id().num) == 1
    }

    /// Read a pin but use the masked version but check whether the datamask for the pin is
    /// cleared as well
    fn read_pin_masked(&self) -> Result<bool, PinError> {
        if !self.datamask() {
            Err(PinError::IsMasked)
        } else {
            let portreg = self.port_reg();
            Ok(((portreg.datain().read().bits() >> self.id().num) & 0x01) == 1)
        }
    }

    /// Write the logic level of an output pin
    #[inline]
    fn write_pin(&mut self, bit: bool) {
        let portreg = self.port_reg();
        let mask = self.mask_32();
        // Safety: SETOUT is a "mask" register, and we only write the bit for
        // this pin ID
        unsafe {
            if bit {
                portreg.setout().write(|w| w.bits(mask));
            } else {
                portreg.clrout().write(|w| w.bits(mask));
            }
        }
    }

    /// Write the logic level of an output pin but check whether the datamask for the pin is
    /// cleared as well
    #[inline]
    fn write_pin_masked(&mut self, bit: bool) -> Result<(), PinError> {
        if !self.datamask() {
            Err(PinError::IsMasked)
        } else {
            let portreg = self.port_reg();
            let mask = self.mask_32();
            // Safety: SETOUT is a "mask" register, and we only write the bit for
            // this pin ID
            unsafe {
                if bit {
                    portreg.setout().write(|w| w.bits(mask));
                } else {
                    portreg.clrout().write(|w| w.bits(mask));
                }
                Ok(())
            }
        }
    }

    /// Toggle the logic level of an output pin
    #[inline]
    fn toggle(&mut self) {
        let portreg = self.port_reg();
        let mask = self.mask_32();
        // Safety: TOGOUT is a "mask" register, and we only write the bit for
        // this pin ID
        unsafe { portreg.togout().write(|w| w.bits(mask)) };
    }

    /// Only useful for input pins
    #[inline]
    fn filter_type(&self, filter: FilterType, clksel: FilterClkSel) {
        let iocfg = self.iocfg_port();
        iocfg.port[self.id().num as usize].modify(|_, w| {
            // Safety: Only write to register for this Pin ID
            unsafe {
                w.flttype().bits(filter as u8);
                w.fltclk().bits(clksel as u8)
            }
        });
    }

    /// Set DATAMASK bit for this particular pin. 1 is the default
    /// state of the bit and allows access of the corresponding bit
    #[inline(always)]
    fn set_datamask(&self) {
        let portreg = self.port_reg();
        unsafe {
            portreg
                .datamask()
                .modify(|r, w| w.bits(r.bits() | self.mask_32()))
        }
    }

    /// Clear DATAMASK bit for this particular pin. This prevents access
    /// of the corresponding bit for output and input operations
    #[inline(always)]
    fn clear_datamask(&self) {
        let portreg = self.port_reg();
        unsafe {
            portreg
                .datamask()
                .modify(|r, w| w.bits(r.bits() & !self.mask_32()))
        }
    }

    /// Only useful for output pins
    /// See p.52 of the programmers guide for more information.
    /// When configured for pulse mode, a given pin will set the non-default state for exactly
    /// one clock cycle before returning to the configured default state
    fn pulse_mode(&self, enable: bool, default_state: PinState) {
        let portreg = self.port_reg();
        unsafe {
            if enable {
                portreg
                    .pulse()
                    .modify(|r, w| w.bits(r.bits() | self.mask_32()));
            } else {
                portreg
                    .pulse()
                    .modify(|r, w| w.bits(r.bits() & !self.mask_32()));
            }
            if default_state == PinState::Low {
                portreg
                    .pulsebase()
                    .modify(|r, w| w.bits(r.bits() & !self.mask_32()));
            } else {
                portreg
                    .pulsebase()
                    .modify(|r, w| w.bits(r.bits() | self.mask_32()));
            }
        }
    }

    /// Only useful for output pins
    fn delay(&self, delay_1: bool, delay_2: bool) {
        let portreg = self.port_reg();
        unsafe {
            if delay_1 {
                portreg
                    .delay1()
                    .modify(|r, w| w.bits(r.bits() | self.mask_32()));
            } else {
                portreg
                    .delay1()
                    .modify(|r, w| w.bits(r.bits() & !self.mask_32()));
            }
            if delay_2 {
                portreg
                    .delay2()
                    .modify(|r, w| w.bits(r.bits() | self.mask_32()));
            } else {
                portreg
                    .delay2()
                    .modify(|r, w| w.bits(r.bits() & !self.mask_32()));
            }
        }
    }
}
