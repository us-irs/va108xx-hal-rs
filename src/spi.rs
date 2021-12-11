//! API for the SPI peripheral
//!
//! ## Examples
//!
//! - [Blocking SPI example](https://egit.irs.uni-stuttgart.de/rust/va108xx-hal/src/branch/main/examples/spi.rs)
use crate::Sealed;
use crate::{
    clock::{enable_peripheral_clock, PeripheralClocks},
    gpio::pins::{
        AltFunc1, AltFunc2, AltFunc3, Pin, PA10, PA11, PA12, PA13, PA14, PA15, PA16, PA17, PA18,
        PA19, PA20, PA21, PA22, PA23, PA24, PA25, PA26, PA27, PA28, PA29, PA30, PA31, PB0, PB1,
        PB10, PB11, PB12, PB13, PB14, PB15, PB16, PB17, PB18, PB19, PB2, PB22, PB23, PB3, PB4, PB5,
        PB6, PB7, PB8, PB9,
    },
    pac::{SPIA, SPIB, SPIC, SYSCONFIG},
    time::Hertz,
};
use core::{convert::Infallible, fmt::Debug, marker::PhantomData};
use embedded_hal::{
    blocking,
    spi::{FullDuplex, Mode, MODE_0, MODE_1, MODE_2, MODE_3},
};

//==================================================================================================
// Defintions
//==================================================================================================

#[derive(Debug, PartialEq, Copy, Clone)]
pub enum HwChipSelectId {
    Id0 = 0,
    Id1 = 1,
    Id2 = 2,
    Id3 = 3,
    Id4 = 4,
    Id5 = 5,
    Id6 = 6,
    Id7 = 7,
    Invalid = 0xff,
}

#[derive(Debug, PartialEq, Copy, Clone)]
pub enum WordSize {
    OneBit = 0x00,
    FourBits = 0x03,
    EightBits = 0x07,
    SixteenBits = 0x0f,
}

//==================================================================================================
// Pin type definitions
//==================================================================================================

pub trait PinSck<SPI>: Sealed {}
pub trait PinMosi<SPI>: Sealed {}
pub trait PinMiso<SPI>: Sealed {}

pub trait HwCs: Sealed {
    const CS_ID: HwChipSelectId;
}
pub trait OptionalHwCs<SPI>: HwCs + Sealed {}

macro_rules! hw_cs_pin {
    ($SPIx:ident, $PXx:ident, $AFx:ident, $HwCsIdent:path, $typedef:ident) => {
        impl HwCs for Pin<$PXx, $AFx> {
            const CS_ID: HwChipSelectId = $HwCsIdent;
        }
        impl OptionalHwCs<$SPIx> for Pin<$PXx, $AFx> {}
        pub type $typedef = Pin<$PXx, $AFx>;
    };
}

impl HwCs for NoneT {
    const CS_ID: HwChipSelectId = HwChipSelectId::Invalid;
}

impl OptionalHwCs<SPIA> for NoneT {}
impl OptionalHwCs<SPIB> for NoneT {}

// SPIA

impl PinSck<SPIA> for Pin<PA31, AltFunc1> {}
impl PinMosi<SPIA> for Pin<PA30, AltFunc1> {}
impl PinMiso<SPIA> for Pin<PA29, AltFunc1> {}

pub type SpiAPortASck = Pin<PA31, AltFunc1>;
pub type SpiAPortAMosi = Pin<PA30, AltFunc1>;
pub type SpiAPortAMiso = Pin<PA29, AltFunc1>;

impl PinSck<SPIA> for Pin<PB9, AltFunc2> {}
impl PinMosi<SPIA> for Pin<PB8, AltFunc2> {}
impl PinMiso<SPIA> for Pin<PB7, AltFunc2> {}

pub type SpiAPortBSck = Pin<PB9, AltFunc2>;
pub type SpiAPortBMosi = Pin<PB8, AltFunc2>;
pub type SpiAPortBMiso = Pin<PB7, AltFunc2>;

hw_cs_pin!(SPIA, PA28, AltFunc1, HwChipSelectId::Id0, HwCs0SpiAPortA);
hw_cs_pin!(SPIA, PA27, AltFunc1, HwChipSelectId::Id1, HwCs1SpiAPortA);
hw_cs_pin!(SPIA, PA26, AltFunc1, HwChipSelectId::Id2, HwCs2SpiAPortA);
hw_cs_pin!(SPIA, PA25, AltFunc1, HwChipSelectId::Id3, HwCs3SpiAPortA);
hw_cs_pin!(SPIA, PA24, AltFunc1, HwChipSelectId::Id4, HwCs4SpiAPortA);
hw_cs_pin!(SPIA, PA23, AltFunc1, HwChipSelectId::Id5, HwCs5SpiAPortA);
hw_cs_pin!(SPIA, PA22, AltFunc1, HwChipSelectId::Id6, HwCs6SpiAPortA);
hw_cs_pin!(SPIA, PA21, AltFunc1, HwChipSelectId::Id7, HwCs7SpiAPortA);

hw_cs_pin!(SPIA, PB6, AltFunc2, HwChipSelectId::Id0, HwCs0SpiAPortB);
hw_cs_pin!(SPIA, PB5, AltFunc2, HwChipSelectId::Id6, HwCs6SpiAPortB);
hw_cs_pin!(SPIA, PB4, AltFunc2, HwChipSelectId::Id5, HwCs5SpiAPortB);
hw_cs_pin!(SPIA, PB3, AltFunc2, HwChipSelectId::Id4, HwCs4SpiAPortB);
hw_cs_pin!(SPIA, PB2, AltFunc2, HwChipSelectId::Id3, HwCs3SpiAPortB);
hw_cs_pin!(SPIA, PB1, AltFunc2, HwChipSelectId::Id2, HwCs2SpiAPortB);
hw_cs_pin!(SPIA, PB0, AltFunc2, HwChipSelectId::Id1, HwCs1SpiAPortB);

// SPIB

impl PinSck<SPIB> for Pin<PA20, AltFunc2> {}
impl PinMosi<SPIB> for Pin<PA19, AltFunc2> {}
impl PinMiso<SPIB> for Pin<PA18, AltFunc2> {}

pub type SpiBPortASck = Pin<PA20, AltFunc2>;
pub type SpiBPortAMosi = Pin<PA19, AltFunc2>;
pub type SpiBPortAMiso = Pin<PA18, AltFunc2>;

impl PinSck<SPIB> for Pin<PB19, AltFunc1> {}
impl PinMosi<SPIB> for Pin<PB18, AltFunc1> {}
impl PinMiso<SPIB> for Pin<PB17, AltFunc1> {}

impl PinSck<SPIB> for Pin<PB5, AltFunc1> {}
impl PinMosi<SPIB> for Pin<PB4, AltFunc1> {}
impl PinMiso<SPIB> for Pin<PB3, AltFunc1> {}

hw_cs_pin!(SPIB, PB16, AltFunc1, HwChipSelectId::Id0, HwCs0SpiBPortB0);
hw_cs_pin!(SPIB, PB15, AltFunc1, HwChipSelectId::Id1, HwCs1SpiBPortB0);
hw_cs_pin!(SPIB, PB14, AltFunc1, HwChipSelectId::Id2, HwCs2SpiBPortB0);
hw_cs_pin!(SPIB, PB13, AltFunc1, HwChipSelectId::Id3, HwCs3SpiBPortB);
hw_cs_pin!(SPIB, PB12, AltFunc1, HwChipSelectId::Id4, HwCs4SpiBPortB);
hw_cs_pin!(SPIB, PB11, AltFunc1, HwChipSelectId::Id5, HwCs5SpiBPortB);
hw_cs_pin!(SPIB, PB10, AltFunc1, HwChipSelectId::Id6, HwCs6SpiBPortB);

hw_cs_pin!(SPIB, PB2, AltFunc1, HwChipSelectId::Id0, HwCs0SpiBPortB1);
hw_cs_pin!(SPIB, PB1, AltFunc1, HwChipSelectId::Id1, HwCs1SpiBPortB1);
hw_cs_pin!(SPIB, PB0, AltFunc1, HwChipSelectId::Id2, HwCs2SpiBPortB1);

hw_cs_pin!(SPIB, PB12, AltFunc2, HwChipSelectId::Id0, HwCs0SpiBPortB2);
hw_cs_pin!(SPIB, PB11, AltFunc2, HwChipSelectId::Id1, HwCs1SpiBPortB2);
hw_cs_pin!(SPIB, PB10, AltFunc2, HwChipSelectId::Id2, HwCs2SpiBPortB2);

hw_cs_pin!(SPIB, PA17, AltFunc2, HwChipSelectId::Id0, HwCs0SpiBPortA);
hw_cs_pin!(SPIB, PA16, AltFunc2, HwChipSelectId::Id1, HwCs1SpiBPortA);
hw_cs_pin!(SPIB, PA15, AltFunc2, HwChipSelectId::Id2, HwCs2SpiBPortA);
hw_cs_pin!(SPIB, PA14, AltFunc2, HwChipSelectId::Id3, HwCs3SpiBPortA);
hw_cs_pin!(SPIB, PA13, AltFunc2, HwChipSelectId::Id4, HwCs4SpiBPortA);
hw_cs_pin!(SPIB, PA12, AltFunc2, HwChipSelectId::Id5, HwCs5SpiBPortA0);
hw_cs_pin!(SPIB, PA11, AltFunc2, HwChipSelectId::Id6, HwCs6SpiBPortA0);
hw_cs_pin!(SPIB, PA10, AltFunc2, HwChipSelectId::Id7, HwCs7SpiBPortA0);

hw_cs_pin!(SPIB, PA23, AltFunc2, HwChipSelectId::Id5, HwCs5SpiBPortA1);
hw_cs_pin!(SPIB, PA22, AltFunc2, HwChipSelectId::Id6, HwCs6SpiBPortA1);
hw_cs_pin!(SPIB, PA21, AltFunc2, HwChipSelectId::Id7, HwCs7SpiBPortA1);

// SPIC

hw_cs_pin!(SPIC, PB9, AltFunc3, HwChipSelectId::Id1, HwCs1SpiCPortB0);
hw_cs_pin!(SPIC, PB8, AltFunc3, HwChipSelectId::Id2, HwCs2SpiCPortB0);
hw_cs_pin!(SPIC, PB7, AltFunc3, HwChipSelectId::Id3, HwCs3SpiCPortB);

hw_cs_pin!(SPIC, PB22, AltFunc3, HwChipSelectId::Id1, HwCs1SpiCPortB1);
hw_cs_pin!(SPIC, PB23, AltFunc3, HwChipSelectId::Id2, HwCs2SpiCPortB1);

hw_cs_pin!(SPIC, PA20, AltFunc1, HwChipSelectId::Id1, HwCs1SpiCPortA0);
hw_cs_pin!(SPIC, PA19, AltFunc1, HwChipSelectId::Id2, HwCs2SpiCPortA0);
hw_cs_pin!(SPIC, PB18, AltFunc1, HwChipSelectId::Id3, HwCs3SpiCPortA0);

hw_cs_pin!(SPIC, PA23, AltFunc3, HwChipSelectId::Id1, HwCs1SpiCPortA1);
hw_cs_pin!(SPIC, PA22, AltFunc3, HwChipSelectId::Id2, HwCs2SpiCPortA1);
hw_cs_pin!(SPIC, PA21, AltFunc3, HwChipSelectId::Id3, HwCs3SpiCPortA1);
hw_cs_pin!(SPIC, PA20, AltFunc3, HwChipSelectId::Id4, HwCs4SpiCPortA);

//==================================================================================================
// Config
//==================================================================================================

pub trait GenericTransferConfig {
    fn sod(&mut self, sod: bool);
    fn blockmode(&mut self, blockmode: bool);
    fn mode(&mut self, mode: Mode);
    fn frequency(&mut self, spi_clk: Hertz);
    fn hw_cs_id(&self) -> u8;
}

/// This struct contains all configuration parameter which are transfer specific
/// and might change for transfers to different SPI slaves
#[derive(Copy, Clone)]
pub struct TransferConfig<HWCS> {
    pub spi_clk: Hertz,
    pub mode: Mode,
    /// This only works if the Slave Output Disable (SOD) bit of the [`SpiConfig`] is set to
    /// false
    pub hw_cs: Option<HWCS>,
    pub sod: bool,
    /// If this is enabled, all data in the FIFO is transmitted in a single frame unless
    /// the BMSTOP bit is set on a dataword. A frame is defined as CSn being active for the
    /// duration of multiple data words
    pub blockmode: bool,
}

/// Type erased variant of the transfer configuration. This is required to avoid generics in
/// the SPI constructor.
pub struct ReducedTransferConfig {
    pub spi_clk: Hertz,
    pub mode: Mode,
    pub sod: bool,
    /// If this is enabled, all data in the FIFO is transmitted in a single frame unless
    /// the BMSTOP bit is set on a dataword. A frame is defined as CSn being active for the
    /// duration of multiple data words
    pub blockmode: bool,
    pub hw_cs: HwChipSelectId,
}

impl TransferConfig<NoneT> {
    pub fn new_no_hw_cs(spi_clk: impl Into<Hertz>, mode: Mode, blockmode: bool, sod: bool) -> Self {
        TransferConfig {
            spi_clk: spi_clk.into(),
            mode,
            hw_cs: None,
            sod,
            blockmode,
        }
    }
}

impl<HWCS: HwCs> TransferConfig<HWCS> {
    pub fn new(
        spi_clk: impl Into<Hertz>,
        mode: Mode,
        hw_cs: Option<HWCS>,
        blockmode: bool,
        sod: bool,
    ) -> Self {
        TransferConfig {
            spi_clk: spi_clk.into(),
            mode,
            hw_cs,
            sod,
            blockmode,
        }
    }

    pub fn downgrade(self) -> ReducedTransferConfig {
        ReducedTransferConfig {
            spi_clk: self.spi_clk,
            mode: self.mode,
            sod: self.sod,
            blockmode: self.blockmode,
            hw_cs: HWCS::CS_ID,
        }
    }
}

impl<HWCS: HwCs> GenericTransferConfig for TransferConfig<HWCS> {
    /// Slave Output Disable
    fn sod(&mut self, sod: bool) {
        self.sod = sod;
    }

    fn blockmode(&mut self, blockmode: bool) {
        self.blockmode = blockmode;
    }

    fn mode(&mut self, mode: Mode) {
        self.mode = mode;
    }

    fn frequency(&mut self, spi_clk: Hertz) {
        self.spi_clk = spi_clk;
    }

    fn hw_cs_id(&self) -> u8 {
        HWCS::CS_ID as u8
    }
}

#[derive(Default)]
/// Configuration options for the whole SPI bus. See Programmer Guide p.92 for more details
pub struct SpiConfig {
    /// Serial clock rate divider. Together with the CLKPRESCALE register, it determines
    /// the SPI clock rate in master mode. 0 by default. Specifying a higher value
    /// limits the maximum attainable SPI speed
    pub scrdv: u8,
    /// By default, configure SPI for master mode (ms == false)
    ms: bool,
    /// Slave output disable. Useful if separate GPIO pins or decoders are used for CS control
    sod: bool,
    /// Loopback mode. If you use this, don't connect MISO to MOSI, they will be tied internally
    lbm: bool,
    /// Enable Master Delayer Capture Mode. See Programmers Guide p.92 for more details
    pub mdlycap: bool,
}

impl SpiConfig {
    pub fn loopback(mut self, enable: bool) -> Self {
        self.lbm = enable;
        self
    }

    pub fn master_mode(mut self, master: bool) -> Self {
        self.ms = !master;
        self
    }

    pub fn slave_output_disable(mut self, sod: bool) -> Self {
        self.sod = sod;
        self
    }
}

//==================================================================================================
// Word Size
//==================================================================================================

/// Configuration trait for the Word Size
/// used by the SPI peripheral
pub trait Word {
    fn word_reg() -> u8;
}

impl Word for u8 {
    fn word_reg() -> u8 {
        0x07
    }
}

impl Word for u16 {
    fn word_reg() -> u8 {
        0x0f
    }
}

//==================================================================================================
// Spi
//==================================================================================================

pub struct SpiBase<SPI, Word = u8> {
    spi: SPI,
    cfg: SpiConfig,
    sys_clk: Hertz,
    blockmode: bool,
    _word: PhantomData<Word>,
}
pub struct Spi<SPI, PINS, Word = u8> {
    spi_base: SpiBase<SPI, Word>,
    pins: PINS,
}

// Re-export this so it can be used for the constructor
pub use crate::typelevel::NoneT;

macro_rules! spi {
    ($($SPIX:ident: ($spix:ident, $clk_enb:path) => ($($WORD:ident),+),)+) => {
        $(
            impl<Sck: PinSck<$SPIX>, Miso: PinMiso<$SPIX>, Mosi: PinMosi<$SPIX>,
                WORD: Word> Spi<$SPIX, (Sck, Miso, Mosi), WORD>
            {
                /// Create a new SPI struct
                ///
                /// You can delete the pin type information by calling the
                /// [`downgrade`](Self::downgrade) function
                ///
                /// ## Arguments
                /// * `spi` - SPI bus to use
                /// * `pins` - Pins to be used for SPI transactions. These pins are consumed
                ///     to ensure the pins can not be used for other purposes anymore
                /// * `spi_cfg` - Configuration specific to the SPI bus
                /// * `transfer_cfg` - Optional initial transfer configuration which includes
                ///     configuration which can change across individual SPI transfers like SPI mode
                ///     or SPI clock. If only one device is connected, this configuration only needs
                ///     to be done once.
                /// * `syscfg` - Can be passed optionally to enable the peripheral clock
                pub fn $spix(
                    spi: $SPIX,
                    pins: (Sck, Miso, Mosi),
                    sys_clk: impl Into<Hertz> + Copy,
                    spi_cfg: SpiConfig,
                    syscfg: Option<&mut SYSCONFIG>,
                    transfer_cfg: Option<&ReducedTransferConfig>,
                ) -> Self {
                    if let Some(syscfg) = syscfg {
                        enable_peripheral_clock(syscfg, $clk_enb);
                    }
                    let SpiConfig {
                        scrdv,
                        ms,
                        sod,
                        lbm,
                        mdlycap,
                    } = spi_cfg;
                    let mut mode = MODE_0;
                    let mut clk_prescale = 0x02;
                    let mut ss = 0;
                    let mut init_blockmode = false;
                    if let Some(transfer_cfg) = transfer_cfg {
                        mode = transfer_cfg.mode;
                        clk_prescale = sys_clk.into().0 / (transfer_cfg.spi_clk.0 * (scrdv as u32 + 1));
                        if  transfer_cfg.hw_cs != HwChipSelectId::Invalid {
                            ss = transfer_cfg.hw_cs as u8;
                        }
                        init_blockmode = transfer_cfg.blockmode;
                    }

                    let (cpo_bit, cph_bit) = match mode {
                        MODE_0 => (false, false),
                        MODE_1 => (false, true),
                        MODE_2 => (true, false),
                        MODE_3 => (true, true),
                    };
                    spi.ctrl0.write(|w| {
                        unsafe {
                            w.size().bits(WORD::word_reg());
                            w.scrdv().bits(scrdv);
                            // Clear clock phase and polarity. Will be set to correct value for each
                            // transfer
                            w.spo().bit(cpo_bit);
                            w.sph().bit(cph_bit)
                        }
                    });
                    spi.ctrl1.write(|w| {
                        w.lbm().bit(lbm);
                        w.sod().bit(sod);
                        w.ms().bit(ms);
                        w.mdlycap().bit(mdlycap);
                        w.blockmode().bit(init_blockmode);
                        unsafe { w.ss().bits(ss) }
                    });

                    spi.fifo_clr.write(|w| {
                        w.rxfifo().set_bit();
                        w.txfifo().set_bit()
                    });
                    spi.clkprescale.write(|w| unsafe { w.bits(clk_prescale) });
                    // Enable the peripheral as the last step as recommended in the
                    // programmers guide
                    spi.ctrl1.modify(|_, w| w.enable().set_bit());
                    Spi {
                        spi_base: SpiBase {
                            spi,
                            cfg: spi_cfg,
                            sys_clk: sys_clk.into(),
                            blockmode: init_blockmode,
                            _word: PhantomData,
                        },
                        pins,
                    }
                }

                #[inline]
                pub fn cfg_clock(&mut self, spi_clk: impl Into<Hertz>) {
                    self.spi_base.cfg_clock(spi_clk);
                }

                #[inline]
                pub fn cfg_mode(&mut self, mode: Mode) {
                    self.spi_base.cfg_mode(mode);
                }

                #[inline]
                pub fn perid(&self) -> u32 {
                    self.spi_base.perid()
                }

                pub fn cfg_transfer<HwCs: OptionalHwCs<$SPIX>>(&mut self, transfer_cfg: &TransferConfig<HwCs>) {
                    self.spi_base.cfg_transfer(transfer_cfg);
                }

                /// Releases the SPI peripheral and associated pins
                pub fn release(self) -> ($SPIX, (Sck, Miso, Mosi), SpiConfig) {
                    (self.spi_base.spi, self.pins, self.spi_base.cfg)
                }

                pub fn downgrade(self) -> SpiBase<$SPIX, WORD> {
                    self.spi_base
                }
            }

            impl<WORD: Word> SpiBase<$SPIX, WORD> {
                #[inline]
                pub fn cfg_clock(&mut self, spi_clk: impl Into<Hertz>) {
                    let clk_prescale = self.sys_clk.0 / (spi_clk.into().0 * (self.cfg.scrdv as u32 + 1));
                    self.spi
                        .clkprescale
                        .write(|w| unsafe { w.bits(clk_prescale) });
                }

                #[inline]
                pub fn cfg_mode(&mut self, mode: Mode) {
                    let (cpo_bit, cph_bit) = match mode {
                        MODE_0 => (false, false),
                        MODE_1 => (false, true),
                        MODE_2 => (true, false),
                        MODE_3 => (true, true),
                    };
                    self.spi.ctrl0.modify(|_, w| {
                        w.spo().bit(cpo_bit);
                        w.sph().bit(cph_bit)
                    });
                }

                #[inline]
                pub fn perid(&self) -> u32 {
                    self.spi.perid.read().bits()
                }

                pub fn cfg_transfer<HwCs: OptionalHwCs<$SPIX>>(&mut self, transfer_cfg: &TransferConfig<HwCs>) {
                    self.cfg_clock(transfer_cfg.spi_clk);
                    self.cfg_mode(transfer_cfg.mode);
                    self.blockmode = transfer_cfg.blockmode;
                    self.spi.ctrl1.modify(|_, w| {
                        if transfer_cfg.sod {
                            w.sod().set_bit();
                        } else if transfer_cfg.hw_cs.is_some() {
                            w.sod().clear_bit();
                            unsafe {
                                w.ss().bits(HwCs::CS_ID as u8);
                            }
                        } else {
                            w.sod().clear_bit();
                        }
                        if transfer_cfg.blockmode {
                            w.blockmode().set_bit();
                        } else {
                            w.blockmode().clear_bit();
                        }
                        w
                    });
                }
            }

            /// Changing the word size also requires a type conversion
            impl <Sck: PinSck<$SPIX>, Miso: PinMiso<$SPIX>, Mosi: PinMosi<$SPIX>>
                From<Spi<$SPIX, (Sck, Miso, Mosi), u8>> for Spi<$SPIX, (Sck, Miso, Mosi), u16>
            {
                fn from(
                    old_spi: Spi<$SPIX, (Sck, Miso, Mosi), u8>
                ) -> Self {
                    old_spi.spi_base.spi.ctrl0.modify(|_, w| {
                        unsafe {
                            w.size().bits(WordSize::SixteenBits as u8)
                        }
                    });
                    Spi {
                        spi_base: SpiBase {
                            spi: old_spi.spi_base.spi,
                            cfg: old_spi.spi_base.cfg,
                            blockmode: old_spi.spi_base.blockmode,
                            sys_clk: old_spi.spi_base.sys_clk,
                            _word: PhantomData,
                        },
                        pins: old_spi.pins,
                    }
                }
            }

            /// Changing the word size also requires a type conversion
            impl <Sck: PinSck<$SPIX>, Miso: PinMiso<$SPIX>, Mosi: PinMosi<$SPIX>>
                From<Spi<$SPIX, (Sck, Miso, Mosi), u16>> for
                Spi<$SPIX, (Sck, Miso, Mosi), u8>
            {
                fn from(
                    old_spi: Spi<$SPIX, (Sck, Miso, Mosi), u16>
                ) -> Self {
                    old_spi.spi_base.spi.ctrl0.modify(|_, w| {
                        unsafe {
                            w.size().bits(WordSize::EightBits as u8)
                        }
                    });
                    Spi {
                        spi_base: SpiBase {
                            spi: old_spi.spi_base.spi,
                            cfg: old_spi.spi_base.cfg,
                            blockmode: old_spi.spi_base.blockmode,
                            sys_clk: old_spi.spi_base.sys_clk,
                            _word: PhantomData,
                        },
                        pins: old_spi.pins,
                    }
                }
            }

            $(

                impl FullDuplex<$WORD> for SpiBase<$SPIX, $WORD>
                {
                    type Error = Infallible;

                    /// Sends a word to the slave
                    #[inline(always)]
                    fn send(&mut self, word: $WORD) -> nb::Result<(), Self::Error> {
                        if self.spi.status.read().tnf().bit_is_clear() {
                            return Err(nb::Error::WouldBlock);
                        }
                        self.spi.data.write(|w| unsafe { w.bits(word as u32) });
                        Ok(())
                    }

                    /// Read a word from the slave. Must be preceeded by a [`send`](Self::send) call
                    #[inline(always)]
                    fn read(&mut self) -> nb::Result<$WORD, Self::Error> {
                        if self.spi.status.read().rne().bit_is_clear() {
                            return Err(nb::Error::WouldBlock);
                        }
                        Ok((self.spi.data.read().bits() & 0xffff) as $WORD)
                    }
                }

                impl<Sck: PinSck<$SPIX>, Miso: PinMiso<$SPIX>, Mosi: PinMosi<$SPIX>>
                    FullDuplex<$WORD> for Spi<$SPIX, (Sck, Miso, Mosi), $WORD>
                {
                    type Error = Infallible;

                    #[inline(always)]
                    fn read(&mut self) -> nb::Result<$WORD, Self::Error> {
                        self.spi_base.read()
                    }

                    #[inline(always)]
                    fn send(&mut self, word: $WORD) -> nb::Result<(), Self::Error> {
                        self.spi_base.send(word)
                    }
                }

                impl SpiBase<$SPIX, $WORD>
                    where SpiBase<$SPIX, $WORD>: FullDuplex<$WORD>
                {
                    /// Internal implementation for blocking::spi::Transfer and
                    /// blocking::spi::Write using the FIFO
                    fn transfer_internal<'w>(
                        &mut self,
                        write_words: &'w [$WORD],
                        read_words: Option<&'w mut [$WORD]>,
                    ) -> Result<(), Infallible> {
                        // FIFO has a depth of 16.
                        const FILL_DEPTH: usize = 12;

                        if self.blockmode {
                            self.spi.ctrl1.modify(|_, w| {
                                w.mtxpause().set_bit()
                            })
                        }
                        // Fill the first half of the write FIFO
                        let len = write_words.len();
                        let mut write = write_words.iter();
                        for _ in 0..core::cmp::min(FILL_DEPTH, len) {
                            nb::block!(self.send(*write.next().unwrap())).ok().unwrap();
                        }
                        if self.blockmode {
                            self.spi.ctrl1.modify(|_, w| {
                                w.mtxpause().clear_bit()
                            })
                        }
                        if let Some(read) = read_words {
                            let mut read = read.iter_mut();

                            // Continue filling write FIFO and emptying read FIFO
                            for word in write {
                                nb::block!(self.send(*word)).ok().unwrap();
                                *read.next().unwrap() = nb::block!(self.read()).ok().unwrap();
                            }

                            // Finish emptying the read FIFO
                            for word in read {
                                *word = nb::block!(self.read()).ok().unwrap();
                            }
                        } else {
                            // Continue filling write FIFO and emptying read FIFO
                            for word in write {
                                nb::block!(self.send(*word)).ok().unwrap();
                                let _ = nb::block!(self.read()).ok().unwrap();
                            }

                            // Dummy read from the read FIFO
                            for _ in 0..core::cmp::min(FILL_DEPTH, len) {
                                let _ = nb::block!(self.read()).ok().unwrap();
                            }
                        }
                        Ok(())
                    }
                }

                impl<Sck: PinSck<$SPIX>, Miso: PinMiso<$SPIX>, Mosi: PinMosi<$SPIX>>
                    Spi<$SPIX, (Sck, Miso, Mosi), $WORD>
                    where Spi<$SPIX, (Sck, Miso, Mosi), $WORD>: FullDuplex<$WORD>
                {
                    /// Internal implementation for blocking::spi::Transfer and
                    /// blocking::spi::Write using the FIFO
                    fn transfer_internal<'w>(
                        &mut self,
                        write_words: &'w [$WORD],
                        read_words: Option<&'w mut [$WORD]>,
                    ) -> Result<(), Infallible> {
                        return self.spi_base.transfer_internal(write_words, read_words)
                    }
                }

                impl blocking::spi::Transfer<$WORD> for SpiBase<$SPIX, $WORD>
                where
                    SpiBase<$SPIX, $WORD>: FullDuplex<$WORD>
                {
                    type Error = Infallible;

                    fn transfer<'w>(
                        &mut self,
                        words: &'w mut [$WORD]
                    ) -> Result<&'w [$WORD], Self::Error> {
                        if words.is_empty() {
                            return Ok(words);
                        }
                        // SAFETY: transfer_internal always writes out bytes
                        // before modifying them
                        let write = unsafe {
                            core::slice::from_raw_parts(words.as_ptr(), words.len())
                        };
                        self.transfer_internal(write, Some(words))?;
                        Ok(words)
                    }
                }

                impl<Sck: PinSck<$SPIX>, Miso: PinMiso<$SPIX>, Mosi: PinMosi<$SPIX>>
                    blocking::spi::Transfer<$WORD> for Spi<$SPIX, (Sck, Miso, Mosi), $WORD>
                where
                    Spi<$SPIX, (Sck, Miso, Mosi), $WORD>: FullDuplex<$WORD>,
                {
                    type Error = Infallible;

                    fn transfer<'w>(
                        &mut self,
                        words: &'w mut [$WORD]
                    ) -> Result<&'w [$WORD], Self::Error> {
                        self.spi_base.transfer(words)
                    }
                }

                impl blocking::spi::Write<$WORD> for SpiBase<$SPIX, $WORD>
                where
                    SpiBase<$SPIX, $WORD>: FullDuplex<$WORD>
                {
                    type Error = Infallible;
                    fn write(&mut self, words: &[$WORD]) -> Result<(), Self::Error> {
                        self.transfer_internal(words, None)
                    }
                }

                impl<Sck: PinSck<$SPIX>, Miso: PinMiso<$SPIX>, Mosi: PinMosi<$SPIX>>
                    blocking::spi::Write<$WORD> for Spi<$SPIX, (Sck, Miso, Mosi), $WORD>
                where
                    Spi<$SPIX, (Sck, Miso, Mosi), $WORD>: FullDuplex<$WORD>,
                {
                    type Error = Infallible;
                    fn write(&mut self, words: &[$WORD]) -> Result<(), Self::Error> {
                        self.transfer_internal(words, None)
                    }
                }
            )+

        )+
    }
}

spi!(
    SPIA: (spia, PeripheralClocks::Spi0) => (u8, u16),
    SPIB: (spib, PeripheralClocks::Spi1) => (u8, u16),
    SPIC: (spic, PeripheralClocks::Spi2) => (u8, u16),
);
