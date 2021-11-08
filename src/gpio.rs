use crate::pac::SYSCONFIG;
use core::convert::Infallible;
use core::marker::PhantomData;
use cortex_m::singleton;
use embedded_hal::digital::v2::{InputPin, OutputPin, StatefulOutputPin, ToggleableOutputPin};
use va108xx::IOCONFIG;

/// Extension trait to split a GPIO peripheral in independent pins and registers
pub trait GpioExt {
    /// The parts to split the GPIO into.
    type Parts;

    /// Splits the GPIO block into independent pins and registers.
    fn split(&mut self, syscfg: &mut SYSCONFIG) -> Option<Self::Parts>;
}

#[derive(Debug, PartialEq)]
pub enum PinModeError {
    InputDisabledForOutput,
    IsMasked,
}

#[derive(Debug, PartialEq)]
pub enum PinState {
    Low = 0,
    High = 1,
}

enum PortId {
    A,
    B,
}

trait GpioRegExt {
    fn is_low(&self, pos: u8) -> bool;
    fn is_set_low(&self, pos: usize, port_id: &PortId) -> Result<bool, PinModeError>;
    fn input_enabled_for_output(&self, pos: usize, port_id: &PortId) -> bool;
    fn set_high(&self, pos: u8);
    fn set_low(&self, pos: u8);
    fn toggle(&self, pos: u8);
}

/// Input mode (type state)
pub struct Input<MODE> {
    _mode: PhantomData<MODE>,
}

/// Output mode (type state)
pub struct Output<MODE> {
    _mode: PhantomData<MODE>,
}

/// Floating input (type state)
pub struct Floating;
/// Pulled down input (type state)
pub struct PullDown;
/// Pulled up input (type state)
pub struct PullUp;
/// Open drain output (type state)
pub struct OpenDrain;
// Push-pull output (type state)
pub struct PushPull;

pub struct OutputInverted;

pub struct InputInverted;

// FUNSEL0 is the regular GPIO port configuration
pub struct FUNSEL1;
pub struct FUNSEL2;
pub struct FUNSEL3;

/// Function select (type state)
pub struct Funsel<FUN> {
    _mode: PhantomData<FUN>,
}

pub enum FilterType {
    SystemClock = 0,
    DirectInputWithSynchronization = 1,
    FilterOneClockCycle = 2,
    FilterTwoClockCycles = 3,
    FilterThreeClockCycles = 4,
    FilterFourClockCycles = 5,
}

pub enum FilterClkSel {
    SysClk = 0,
    Clk1 = 1,
    Clk2 = 2,
    Clk3 = 3,
    Clk4 = 4,
    Clk5 = 5,
    Clk6 = 6,
    Clk7 = 7,
}

/// Fully erased pin
pub struct Pin<MODE> {
    i: u8,
    port_id: PortId,
    port: *const dyn GpioRegExt,
    _mode: PhantomData<MODE>,
}
// NOTE(unsafe) this only enables read access to the same pin from multiple threads
unsafe impl<MODE> Send for Pin<MODE> {}
impl<MODE> StatefulOutputPin for Pin<Output<MODE>> {
    #[inline(always)]
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        self.is_set_low().map(|v| !v)
    }
    #[inline(always)]
    fn is_set_low(&self) -> Result<bool, Self::Error> {
        unsafe { (*self.port).is_set_low(self.i.into(), &self.port_id) }
    }
}
impl<MODE> OutputPin for Pin<Output<MODE>> {
    type Error = PinModeError;
    #[inline(always)]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        unsafe { (*self.port).set_high(self.i) };
        Ok(())
    }
    #[inline(always)]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        unsafe { (*self.port).set_low(self.i) }
        Ok(())
    }
}

impl<MODE> ToggleableOutputPin for Pin<Output<MODE>> {
    type Error = Infallible;
    #[inline(always)]
    fn toggle(&mut self) -> Result<(), Self::Error> {
        unsafe { (*self.port).toggle(self.i) }
        Ok(())
    }
}

impl InputPin for Pin<Output<OpenDrain>> {
    type Error = Infallible;
    #[inline(always)]
    fn is_high(&self) -> Result<bool, Self::Error> {
        self.is_low().map(|v| !v)
    }
    #[inline(always)]
    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(unsafe { (*self.port).is_low(self.i) })
    }
}
impl<MODE> InputPin for Pin<Input<MODE>> {
    type Error = Infallible;
    #[inline(always)]
    fn is_high(&self) -> Result<bool, Self::Error> {
        self.is_low().map(|v| !v)
    }
    #[inline(always)]
    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(unsafe { (*self.port).is_low(self.i) })
    }
}

// This is only needs to be implemented for PORTA because PORTB is derived
// from PORTA
impl GpioRegExt for crate::pac::porta::RegisterBlock {
    #[inline(always)]
    fn is_low(&self, pos: u8) -> bool {
        self.datainraw().read().bits() & (1 << pos) == 0
    }
    #[inline(always)]
    fn is_set_low(&self, pos: usize, port_id: &PortId) -> Result<bool, PinModeError> {
        if self.input_enabled_for_output(pos, port_id) {
            Ok(self.datainraw().read().bits() & (1 << pos) == 0)
        } else {
            Err(PinModeError::InputDisabledForOutput)
        }
    }
    #[inline(always)]
    fn set_high(&self, pos: u8) {
        unsafe { self.setout().write(|w| w.bits(1 << pos)) }
    }
    #[inline(always)]
    fn set_low(&self, pos: u8) {
        unsafe { self.clrout().write(|w| w.bits(1 << pos)) }
    }
    #[inline(always)]
    fn toggle(&self, pos: u8) {
        unsafe { self.togout().write(|w| w.bits(1 << pos)) }
    }
    #[inline(always)]
    fn input_enabled_for_output(&self, pos: usize, port_id: &PortId) -> bool {
        unsafe {
            let iocfg = &(*IOCONFIG::ptr());
            match port_id {
                PortId::A => iocfg.porta[pos].read().iewo().bit_is_set(),
                PortId::B => iocfg.portb[pos].read().iewo().bit_is_set(),
            }
        }
    }
}

macro_rules! gpio {
    ($PORTX:ident, $portx:ident, $port_id:path, [
        $($PXi:ident: ($pxi:ident, $i:expr),)+
    ]) => {
        pub mod $portx {
            use core::marker::PhantomData;
            use core::convert::Infallible;
            use super::{
                FUNSEL1, FUNSEL2, FUNSEL3, Floating, Funsel, GpioExt, Input, OpenDrain,
                PullUp, Output, FilterType, FilterClkSel, Pin, GpioRegExt, PushPull,
                PinModeError, PinState, PortId, singleton
            };
            use crate::{pac::$PORTX, pac::SYSCONFIG, pac::IOCONFIG};
            use embedded_hal::digital::v2::{
                InputPin, OutputPin, StatefulOutputPin, ToggleableOutputPin
            };
            pub struct Parts {
                $(
                    pub $pxi: $PXi<Input<Floating>>,
                )+
            }

            pub fn get_perid(port: &$PORTX) -> u32 {
                port.perid.read().bits()
            }

            impl GpioExt for $PORTX {
                type Parts = Parts;

                /// This function splits the PORT into the individual pins
                /// Should only be called once and returns None on subsequent calls
                fn split(&mut self, syscfg: &mut SYSCONFIG) -> Option<Parts> {
                    let _: &'static mut bool = singleton!(: bool = false)?;
                    syscfg.peripheral_clk_enable.modify(|_, w| {
                        w.$portx().set_bit();
                        w.gpio().set_bit();
                        w.ioconfig().set_bit();
                        w
                    });
                    Some(Parts {
                        $(
                            $pxi: $PXi { _mode : PhantomData },
                        )+
                    })
                }
            }

            fn _set_alternate_mode(iocfg: &mut IOCONFIG, index: usize, mode: u8) {
                unsafe {
                    iocfg.$portx[index].modify(|_, w| {
                        w.funsel().bits(mode)
                    })
                }
            }

            $(
                pub struct $PXi<MODE> {
                    _mode: PhantomData<MODE>,
                }

                impl<MODE> $PXi<MODE> {
                    pub fn into_funsel_1(self, iocfg: &mut IOCONFIG) -> $PXi<Funsel<FUNSEL1>> {
                        _set_alternate_mode(iocfg, $i, 1);
                        $PXi { _mode: PhantomData }
                    }
                    pub fn into_funsel_2(self, iocfg: &mut IOCONFIG) -> $PXi<Funsel<FUNSEL2>> {
                        _set_alternate_mode(iocfg, $i, 2);
                        $PXi { _mode: PhantomData }
                    }
                    pub fn into_funsel_3(self, iocfg: &mut IOCONFIG) -> $PXi<Funsel<FUNSEL3>> {
                        _set_alternate_mode(iocfg, $i, 3);
                        $PXi { _mode: PhantomData }
                    }

                    // Get DATAMASK bit for this particular pin
                    #[inline(always)]
                    pub fn datamask(&self, port: &$PORTX) -> bool {
                        (port.datamask().read().bits() >> $i) == 1
                    }
                    /// Set DATAMASK bit for this particular pin. 1 is the default
                    /// state of the bit and allows access of the corresponding bit
                    #[inline(always)]
                    pub fn set_datamask(self, port: &mut $PORTX) -> Self  {
                        unsafe {
                            port.datamask().modify(|r, w| w.bits(r.bits() | (1 << $i)))
                        }
                        self
                    }

                    /// Clear DATAMASK bit for this particular pin. This prevents access
                    /// of the corresponding bit for output and input operations
                    #[inline(always)]
                    pub fn clear_datamask(self, port: &mut $PORTX) -> Self  {
                        unsafe {
                            port.datamask().modify(|r, w| w.bits(r.bits() & !(1 << $i)))
                        }
                        self
                    }

                    pub fn into_floating_input(
                        self, iocfg: &mut IOCONFIG, port: &mut $PORTX
                    ) -> $PXi<Input<Floating>> {
                        unsafe {
                            iocfg.$portx[$i].modify(|_, w| {
                                w.funsel().bits(0);
                                w.pen().clear_bit();
                                w.opendrn().clear_bit()
                            });
                            port.dir().modify(|r,w| w.bits(r.bits() & !(1 << $i)));
                        }
                        $PXi { _mode: PhantomData }
                    }

                    pub fn into_pull_up_input(self, iocfg: &mut IOCONFIG) -> $PXi<Input<PullUp>> {
                        unsafe {
                            iocfg.$portx[$i].modify(|_, w| {
                                w.funsel().bits(0);
                                w.pen().set_bit();
                                w.plevel().set_bit();
                                w.opendrn().clear_bit()
                            });
                            let port_reg = &(*$PORTX::ptr());
                            port_reg.dir().modify(|r,w| w.bits(r.bits() & !(1 << $i)));
                        }
                        $PXi { _mode: PhantomData }
                    }

                    pub fn into_pull_down_input(
                        self, iocfg: &mut IOCONFIG, port_reg: &mut $PORTX
                    ) -> $PXi<Input<PullUp>> {
                        unsafe {
                            iocfg.$portx[$i].modify(|_, w| {
                                w.funsel().bits(0);
                                w.pen().set_bit();
                                w.plevel().clear_bit();
                                w.opendrn().clear_bit()
                            });
                            port_reg.dir().modify(|r,w| w.bits(r.bits() & !(1 << $i)));
                        }
                        $PXi { _mode: PhantomData }
                    }

                    pub fn into_open_drain_output(
                        self, iocfg: &mut IOCONFIG
                    ) -> $PXi<Output<OpenDrain>> {
                        unsafe {
                            iocfg.$portx[$i].modify(|_, w| {
                                w.funsel().bits(0);
                                w.pen().clear_bit();
                                w.opendrn().set_bit()
                            });
                            let port_reg = &(*$PORTX::ptr());
                            port_reg.dir().modify(|r,w| w.bits(r.bits() | (1 << $i)));
                        }
                        $PXi { _mode: PhantomData }
                    }

                    pub fn into_push_pull_output(
                        self, iocfg: &mut IOCONFIG, port_reg: &mut $PORTX
                    ) -> $PXi<Output<PushPull>> {
                        unsafe {
                            iocfg.$portx[$i].modify(|_, w| {
                                w.funsel().bits(0);
                                w.opendrn().clear_bit()
                            });
                            port_reg.dir().modify(|r,w| w.bits(r.bits() | (1 << $i)));
                            port_reg.clrout().write(|w| w.bits(1 << $i))
                        }
                        $PXi { _mode: PhantomData }
                    }

                    pub fn filter_type(
                        self, iocfg: &mut IOCONFIG, filter: FilterType, clksel: FilterClkSel
                     ) -> Self {
                        unsafe {
                            iocfg.$portx[$i].modify(|_, w| {
                                w.flttype().bits(filter as u8);
                                w.fltclk().bits(clksel as u8)
                            });
                        }
                        self
                    }
                }

                impl<MODE> $PXi<Input<MODE>> {
                    pub fn input_inversion(self, iocfg: &mut IOCONFIG, enable: bool) -> Self {
                        if enable {
                            iocfg.$portx[$i].modify(|_, w| w.invinp().set_bit());
                        } else {
                            iocfg.$portx[$i].modify(|_, w| w.invinp().clear_bit());
                        }
                        self
                    }

                    /// This function takes account for the data mask register
                    #[inline(always)]
                    pub fn is_high_masked(&self, port: &$PORTX) -> Result<bool, PinModeError> {
                        self.is_low_masked(port).map(|v| !v)
                    }

                    /// This function takes account for the data mask register
                    #[inline(always)]
                    pub fn is_low_masked(&self, port: &$PORTX) -> Result<bool, PinModeError> {
                        if ((port.datamask().read().bits() >> $i) & 1) == 0 {
                            Err(PinModeError::IsMasked)
                        } else {
                            Ok(port.datain().read().bits() & (1 << $i) == 0)
                        }
                    }
                }

                impl<MODE> $PXi<Output<MODE>> {
                    pub fn output_inversion(self, iocfg: &mut IOCONFIG, enable: bool) -> Self {
                        if enable {
                            iocfg.$portx[$i].modify(|_, w| w.invout().set_bit());
                        } else {
                            iocfg.$portx[$i].modify(|_, w| w.invout().clear_bit());
                        }
                        self
                    }

                    /// Enable Input even when in output mode. In
                    /// this mode the input receiver is enabled even
                    /// if the direction is configured as an output.
                    /// This allows monitoring of output values
                    pub fn enable_input(self, iocfg: &mut IOCONFIG, enable: bool) -> Self {
                        if enable {
                            iocfg.$portx[$i].modify(|_, w| w.iewo().set_bit());
                        } else {
                            iocfg.$portx[$i].modify(|_, w| w.iewo().clear_bit());
                        }
                        self
                    }

                    /// Enable Pull up/down even when output is active. The Default is to
                    /// disable pull up/down when output is actively driven. This bit enables the
                    /// pull up/down all the time.
                    ///
                    /// # Arguments
                    ///
                    /// `enable` - Enable the peripheral functionality
                    /// `enable_pullup` - Enable the pullup itself
                    pub fn enable_pull_up(
                        self, iocfg: &mut IOCONFIG, enable: bool, enable_pullup: bool
                    ) -> Self {
                        iocfg.$portx[$i].modify(|_, w| {
                            if enable { w.pwoa().set_bit(); } else { w.pwoa().clear_bit(); }
                            if enable_pullup { w.pen().set_bit(); } else { w.pen().clear_bit(); }
                            w.plevel().set_bit()
                        });
                        self
                    }
                    /// Enable Pull up/down even when output is active. The Default is to disable
                    /// pull up/down when output is actively driven. This bit enables the
                    /// pull up/down all the time.
                    ///
                    /// # Arguments
                    ///
                    /// `enable` - Enable the peripheral functionality
                    /// `enable_pullup` - Enable the pulldown itself
                    pub fn enable_pull_down(
                        self, iocfg: &mut IOCONFIG, enable: bool, enable_pulldown: bool
                    ) -> Self {
                        iocfg.$portx[$i].modify(|_, w| {
                            if enable { w.pwoa().set_bit(); } else { w.pwoa().clear_bit(); }
                            if enable_pulldown { w.pen().set_bit(); } else { w.pen().clear_bit(); }
                            w.plevel().clear_bit()
                        });
                        self
                    }
                }

                impl<MODE> $PXi<Output<MODE>> {
                    /// Erases the pin number from the type
                    ///
                    /// This is useful when you want to collect the pins into an array where you
                    /// need all the elements to have the same type
                    pub fn downgrade(self) -> Pin<Output<MODE>> {
                        Pin {
                            i: $i,
                            port_id: $port_id,
                            port: $PORTX::ptr() as *const dyn GpioRegExt,
                            _mode: self._mode,
                        }
                    }

                    /// This function takes account for the data mask register
                    #[inline(always)]
                    pub fn set_low_masked(&self, port: &$PORTX) -> Result<(), PinModeError> {
                        if ((port.datamask().read().bits() >> $i) & 1) == 0 {
                            Err(PinModeError::IsMasked)
                        } else {
                            Ok(unsafe { (*$PORTX::ptr()).set_low($i) })
                        }
                    }
                    /// This function takes account for the data mask register
                    #[inline(always)]
                    pub fn set_high_masked(&self, port: &$PORTX) -> Result<(), PinModeError> {
                        if ((port.datamask().read().bits() >> $i) & 1) == 0 {
                            Err(PinModeError::IsMasked)
                        } else {
                            Ok(unsafe { (*$PORTX::ptr()).set_high($i) })
                        }
                    }

                    pub fn pulse_mode(self, port: &mut $PORTX, enable: bool, default_state: PinState) -> Self {
                        unsafe {
                            if enable {
                                port.pulse().modify(|r, w| w.bits(r.bits() | (1 << $i)));
                            } else {
                                port.pulse().modify(|r, w| w.bits(r.bits() & !(1 << $i)));
                            }
                            if default_state == PinState::Low {
                                port.pulsebase().modify(|r,w| w.bits(r.bits() & !(1 << $i)));
                            }
                            else {
                                port.pulsebase().modify(|r,w| w.bits(r.bits() | (1 << $i)));
                            }
                        }
                        self
                    }

                    pub fn delay(self, port: &mut $PORTX, delay_1: bool, delay_2: bool) -> Self {
                        unsafe {
                            if delay_1 {
                                port.delay1().modify(|r, w| w.bits(r.bits() | (1 << $i)));
                            } else {
                                port.delay1().modify(|r, w| w.bits(r.bits() & !(1 << $i)));
                            }
                            if delay_2 {
                                port.delay2().modify(|r, w| w.bits(r.bits() | (1 << $i)));
                            } else {
                                port.delay2().modify(|r, w| w.bits(r.bits() & !(1 << $i)));
                            }
                        }
                        self
                    }
                }

                impl<MODE> StatefulOutputPin for $PXi<Output<MODE>> {
                    #[inline(always)]
                    fn is_set_high(&self) -> Result<bool, Self::Error> {
                        self.is_set_low().map(|v| !v)
                    }
                    #[inline(always)]
                    fn is_set_low(&self) -> Result<bool, Self::Error> {
                        unsafe {
                            (*$PORTX::ptr()).is_set_low($i, &$port_id)
                        }
                    }
                }
                impl<MODE> OutputPin for $PXi<Output<MODE>> {
                    type Error = PinModeError;
                    #[inline(always)]
                    fn set_high(&mut self) -> Result<(), Self::Error> {
                        Ok(unsafe { (*$PORTX::ptr()).set_high($i) })
                    }
                    #[inline(always)]
                    fn set_low(&mut self) -> Result<(), Self::Error> {
                        Ok(unsafe { (*$PORTX::ptr()).set_low($i) })
                    }
                }
                impl<MODE> ToggleableOutputPin for $PXi<Output<MODE>> {
                    type Error = Infallible;
                    #[inline(always)]
                    fn toggle(&mut self) -> Result<(), Self::Error> {
                        Ok(unsafe { (*$PORTX::ptr()).toggle($i) })
                    }
                }
                impl<MODE> $PXi<Input<MODE>> {
                    /// Erases the pin number from the type
                    ///
                    /// This is useful when you want to collect the pins into an array where you
                    /// need all the elements to have the same type
                    pub fn downgrade(self) -> Pin<Input<MODE>> {
                        Pin {
                            i: $i,
                            port_id: $port_id,
                            port: $PORTX::ptr() as *const dyn GpioRegExt,
                            _mode: self._mode,
                        }
                    }
                }
                impl<MODE> InputPin for $PXi<Input<MODE>> {
                    type Error = Infallible;
                    #[inline(always)]
                    fn is_high(&self) -> Result<bool, Self::Error> {
                        self.is_low().map(|v| !v)
                    }
                    #[inline(always)]
                    fn is_low(&self) -> Result<bool, Self::Error> {
                        Ok(unsafe { (*$PORTX::ptr()).is_low($i) })
                    }
                }
            )+
        }
    }
}

gpio!(PORTA, porta, PortId::A, [
    PA0: (pa0, 0),
    PA1: (pa1, 1),
    PA2: (pa2, 2),
    PA3: (pa3, 3),
    PA4: (pa4, 4),
    PA5: (pa5, 5),
    PA6: (pa6, 6),
    PA7: (pa7, 7),
    PA8: (pa8, 8),
    PA9: (pa9, 9),
    PA10: (pa10, 10),
    PA11: (pa11, 11),
    PA12: (pa12, 12),
    PA13: (pa13, 13),
    PA14: (pa14, 14),
    PA15: (pa15, 15),
    PA16: (pa16, 16),
    PA17: (pa17, 17),
    PA18: (pa18, 18),
    PA19: (pa19, 19),
    PA20: (pa20, 20),
    PA21: (pa21, 21),
    PA22: (pa22, 22),
    PA23: (pa23, 23),
    PA24: (pa24, 24),
    PA25: (pa25, 25),
    PA26: (pa26, 26),
    PA27: (pa27, 27),
    PA28: (pa28, 28),
    PA29: (pa29, 29),
    PA30: (pa30, 30),
    PA31: (pa31, 31),
]);

gpio!(PORTB, portb, PortId::B, [
    PB0: (pb0, 0),
    PB1: (pb1, 1),
    PB2: (pb2, 2),
    PB3: (pb3, 3),
    PB4: (pb4, 4),
    PB5: (pb5, 5),
    PB6: (pb6, 6),
    PB7: (pb7, 7),
    PB8: (pb8, 8),
    PB9: (pb9, 9),
    PB10: (pb10, 10),
    PB11: (pb11, 11),
    PB12: (pb12, 12),
    PB13: (pb13, 13),
    PB14: (pb14, 14),
    PB15: (pb15, 15),
    PB16: (pb16, 16),
    PB17: (pb17, 17),
    PB18: (pb18, 18),
    PB19: (pb19, 19),
    PB20: (pb20, 20),
    PB21: (pb21, 21),
    PB22: (pb22, 22),
    PB23: (pb23, 23),
]);
