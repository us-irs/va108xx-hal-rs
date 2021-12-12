//! API for the I2C peripheral
//!
//! ## Examples
//!
//! - [REB1 I2C temperature sensor example](https://egit.irs.uni-stuttgart.de/rust/vorago-reb1/src/branch/main/examples/adt75-temp-sensor.rs)
use crate::{
    clock::{enable_peripheral_clock, PeripheralClocks},
    pac::{I2CA, I2CB, SYSCONFIG},
    time::Hertz,
    Sealed,
};
use core::marker::PhantomData;
use embedded_hal::blocking::i2c::{Read, Write, WriteIter, WriteIterRead, WriteRead};

pub use embedded_hal::blocking::i2c::{SevenBitAddress, TenBitAddress};

//==================================================================================================
// Defintions
//==================================================================================================

#[derive(Debug, PartialEq, Copy, Clone)]
pub enum FifoEmptyMode {
    Stall = 0,
    EndTransaction = 1,
}

#[derive(Debug, PartialEq)]
pub enum Error {
    InvalidTimingParams,
    ArbitrationLost,
    NackAddr,
    /// Data not acknowledged in write operation
    NackData,
    /// Not enough data received in read operation
    InsufficientDataReceived,
    /// Number of bytes in transfer too large (larger than 0x7fe)
    DataTooLarge,
    WrongAddrMode,
}

#[derive(Debug, PartialEq, Copy, Clone)]
enum I2cCmd {
    Start = 0b00,
    Stop = 0b10,
    StartWithStop = 0b11,
    Cancel = 0b100,
}

#[derive(Debug, PartialEq, Copy, Clone)]
pub enum I2cSpeed {
    Regular100khz = 0,
    Fast400khz = 1,
}

#[derive(Debug, PartialEq)]
pub enum I2cDirection {
    Send = 0,
    Read = 1,
}

#[derive(Debug, PartialEq, Copy, Clone)]
pub enum I2cAddress {
    Regular(u8),
    TenBit(u16),
}

//==================================================================================================
// Config
//==================================================================================================

pub struct TrTfThighTlow(u8, u8, u8, u8);
pub struct TsuStoTsuStaThdStaTBuf(u8, u8, u8, u8);

pub struct TimingCfg {
    // 4 bit max width
    tr: u8,
    // 4 bit max width
    tf: u8,
    // 4 bit max width
    thigh: u8,
    // 4 bit max width
    tlow: u8,
    // 4 bit max width
    tsu_sto: u8,
    // 4 bit max width
    tsu_sta: u8,
    // 4 bit max width
    thd_sta: u8,
    // 4 bit max width
    tbuf: u8,
}

impl TimingCfg {
    pub fn new(
        first_16_bits: TrTfThighTlow,
        second_16_bits: TsuStoTsuStaThdStaTBuf,
    ) -> Result<Self, Error> {
        if first_16_bits.0 > 0xf
            || first_16_bits.1 > 0xf
            || first_16_bits.2 > 0xf
            || first_16_bits.3 > 0xf
            || second_16_bits.0 > 0xf
            || second_16_bits.1 > 0xf
            || second_16_bits.2 > 0xf
            || second_16_bits.3 > 0xf
        {
            return Err(Error::InvalidTimingParams);
        }
        Ok(TimingCfg {
            tr: first_16_bits.0,
            tf: first_16_bits.1,
            thigh: first_16_bits.2,
            tlow: first_16_bits.3,
            tsu_sto: second_16_bits.0,
            tsu_sta: second_16_bits.1,
            thd_sta: second_16_bits.2,
            tbuf: second_16_bits.3,
        })
    }

    pub fn reg(&self) -> u32 {
        ((self.tbuf as u32) << 28
            | (self.thd_sta as u32) << 24
            | (self.tsu_sta as u32) << 20
            | (self.tsu_sto as u32) << 16
            | (self.tlow as u32) << 12
            | (self.thigh as u32) << 8
            | (self.tf as u32) << 4
            | (self.tr as u32)) as u32
    }
}

impl Default for TimingCfg {
    fn default() -> Self {
        TimingCfg {
            tr: 0x02,
            tf: 0x01,
            thigh: 0x08,
            tlow: 0x09,
            tsu_sto: 0x8,
            tsu_sta: 0x0a,
            thd_sta: 0x8,
            tbuf: 0xa,
        }
    }
}

pub struct MasterConfig {
    pub tx_fe_mode: FifoEmptyMode,
    pub rx_fe_mode: FifoEmptyMode,
    /// Enable the analog delay glitch filter
    pub alg_filt: bool,
    /// Enable the digital glitch filter
    pub dlg_filt: bool,
    pub tm_cfg: Option<TimingCfg>,
    // Loopback mode
    // lbm: bool,
}

impl Default for MasterConfig {
    fn default() -> Self {
        MasterConfig {
            tx_fe_mode: FifoEmptyMode::Stall,
            rx_fe_mode: FifoEmptyMode::Stall,
            alg_filt: false,
            dlg_filt: false,
            tm_cfg: None,
        }
    }
}

impl Sealed for MasterConfig {}

pub struct SlaveConfig {
    pub tx_fe_mode: FifoEmptyMode,
    pub rx_fe_mode: FifoEmptyMode,
    /// Maximum number of words before issuing a negative acknowledge.
    /// Range should be 0 to 0x7fe. Setting the value to 0x7ff has the same effect as not setting
    /// the enable bit since RXCOUNT stops counting at 0x7fe.
    pub max_words: Option<usize>,
    /// A received address is compared to the ADDRESS register (addr) using the address mask
    /// (addr_mask). Those bits with a 1 in the address mask must match for there to be an address
    /// match
    pub addr: I2cAddress,
    /// The default address mask will be 0x3ff to only allow full matches
    pub addr_mask: Option<u16>,
    /// Optionally specify a second I2C address the slave interface responds to
    pub addr_b: Option<I2cAddress>,
    pub addr_b_mask: Option<u16>,
}

impl SlaveConfig {
    /// Build a default slave config given a specified slave address to respond to
    pub fn new(addr: I2cAddress) -> Self {
        SlaveConfig {
            tx_fe_mode: FifoEmptyMode::Stall,
            rx_fe_mode: FifoEmptyMode::Stall,
            max_words: None,
            addr,
            addr_mask: None,
            addr_b: None,
            addr_b_mask: None,
        }
    }
}

impl Sealed for SlaveConfig {}

//==================================================================================================
// I2C Base
//==================================================================================================

pub struct I2cBase<I2C> {
    i2c: I2C,
    sys_clk: Hertz,
}

impl<I2C> I2cBase<I2C> {
    #[inline]
    fn unwrap_addr(addr: I2cAddress) -> (u16, u32) {
        match addr {
            I2cAddress::Regular(addr) => (addr as u16, 0 << 15),
            I2cAddress::TenBit(addr) => (addr, 1 << 15),
        }
    }
}

macro_rules! i2c_base {
    ($($I2CX:ident: ($i2cx:ident, $clk_enb:path),)+) => {
        $(
            impl I2cBase<$I2CX> {
                pub fn $i2cx(
                    i2c: $I2CX,
                    sys_clk: impl Into<Hertz>,
                    speed_mode: I2cSpeed,
                    ms_cfg: Option<&MasterConfig>,
                    sl_cfg: Option<&SlaveConfig>,
                    sys_cfg: Option<&mut SYSCONFIG>,
                ) -> Self {
                    if let Some(sys_cfg) = sys_cfg {
                        enable_peripheral_clock(sys_cfg, $clk_enb);
                    }
                    let mut i2c_base = I2cBase {
                        i2c,
                        sys_clk: sys_clk.into(),
                    };
                    if let Some(ms_cfg) = ms_cfg {
                        i2c_base.cfg_master(ms_cfg);
                    }

                    if let Some(sl_cfg) = sl_cfg {
                        i2c_base.cfg_slave(sl_cfg);
                    }
                    i2c_base.cfg_clk_scale(speed_mode);
                    i2c_base
                }

                fn cfg_master(&mut self, ms_cfg: &MasterConfig) {
                    let (txfemd, rxfemd) = match (ms_cfg.tx_fe_mode, ms_cfg.rx_fe_mode) {
                        (FifoEmptyMode::Stall, FifoEmptyMode::Stall) => (false, false),
                        (FifoEmptyMode::Stall, FifoEmptyMode::EndTransaction) => (false, true),
                        (FifoEmptyMode::EndTransaction, FifoEmptyMode::Stall) => (true, false),
                        (FifoEmptyMode::EndTransaction, FifoEmptyMode::EndTransaction) => (true, true),
                    };
                    self.i2c.ctrl.modify(|_, w| {
                        w.txfemd().bit(txfemd);
                        w.rxffmd().bit(rxfemd);
                        w.dlgfilter().bit(ms_cfg.dlg_filt);
                        w.algfilter().bit(ms_cfg.alg_filt)
                    });
                    if let Some(ref tm_cfg) = ms_cfg.tm_cfg {
                        self.i2c.tmconfig.write(|w| unsafe { w.bits(tm_cfg.reg()) });
                    }
                    self.i2c.fifo_clr.write(|w| {
                        w.rxfifo().set_bit();
                        w.txfifo().set_bit()
                    });
                }

                fn cfg_slave(&mut self, sl_cfg: &SlaveConfig) {
                    let (txfemd, rxfemd) = match (sl_cfg.tx_fe_mode, sl_cfg.rx_fe_mode) {
                        (FifoEmptyMode::Stall, FifoEmptyMode::Stall) => (false, false),
                        (FifoEmptyMode::Stall, FifoEmptyMode::EndTransaction) => (false, true),
                        (FifoEmptyMode::EndTransaction, FifoEmptyMode::Stall) => (true, false),
                        (FifoEmptyMode::EndTransaction, FifoEmptyMode::EndTransaction) => (true, true),
                    };
                    self.i2c.s0_ctrl.modify(|_, w| {
                        w.txfemd().bit(txfemd);
                        w.rxffmd().bit(rxfemd)
                    });
                    self.i2c.s0_fifo_clr.write(|w| {
                        w.rxfifo().set_bit();
                        w.txfifo().set_bit()
                    });
                    let max_words = sl_cfg.max_words;
                    if let Some(max_words) = max_words {
                        self.i2c
                            .s0_maxwords
                            .write(|w| unsafe { w.bits(1 << 31 | max_words as u32) });
                    }
                    let (addr, addr_mode_mask) = Self::unwrap_addr(sl_cfg.addr);
                    // The first bit is the read/write value. Normally, both read and write are matched
                    // using the RWMASK bit of the address mask register
                    self.i2c
                        .s0_address
                        .write(|w| unsafe { w.bits((addr << 1) as u32 | addr_mode_mask) });
                    if let Some(addr_mask) = sl_cfg.addr_mask {
                        self.i2c
                            .s0_addressmask
                            .write(|w| unsafe { w.bits((addr_mask << 1) as u32) });
                    }
                    if let Some(addr_b) = sl_cfg.addr_b {
                        let (addr, addr_mode_mask) = Self::unwrap_addr(addr_b);
                        self.i2c
                            .s0_addressb
                            .write(|w| unsafe { w.bits((addr << 1) as u32 | addr_mode_mask) })
                    }
                    if let Some(addr_b_mask) = sl_cfg.addr_b_mask {
                        self.i2c
                            .s0_addressmaskb
                            .write(|w| unsafe { w.bits((addr_b_mask << 1) as u32) })
                    }
                }

                #[inline]
                pub fn filters(&mut self, digital_filt: bool, analog_filt: bool) {
                    self.i2c.ctrl.modify(|_, w| {
                        w.dlgfilter().bit(digital_filt);
                        w.algfilter().bit(analog_filt)
                    });
                }

                #[inline]
                pub fn fifo_empty_mode(&mut self, rx: FifoEmptyMode, tx: FifoEmptyMode) {
                    self.i2c.ctrl.modify(|_, w| {
                        w.txfemd().bit(tx as u8 != 0);
                        w.rxffmd().bit(rx as u8 != 0)
                    });
                }

                fn calc_clk_div(&self, speed_mode: I2cSpeed) -> u8 {
                    if speed_mode == I2cSpeed::Regular100khz {
                        ((self.sys_clk.0 / (u32::pow(10, 5) * 20)) - 1) as u8
                    } else {
                        (((10 * self.sys_clk.0) / u32::pow(10, 8)) - 1) as u8
                    }
                }

                /// Configures the clock scale for a given speed mode setting
                pub fn cfg_clk_scale(&mut self, speed_mode: I2cSpeed) {
                    self.i2c.clkscale.write(|w| unsafe {
                        w.bits((speed_mode as u32) << 31 | self.calc_clk_div(speed_mode) as u32)
                    });
                }

                pub fn load_address(&mut self, addr: u16) {
                    // Load address
                    self.i2c
                        .address
                        .write(|w| unsafe { w.bits((addr << 1) as u32) });
                }

                #[inline]
                fn stop_cmd(&mut self) {
                    self.i2c
                        .cmd
                        .write(|w| unsafe { w.bits(I2cCmd::Stop as u32) });
                }
            }
        )+
    }
}

// Unique mode to use the loopback functionality
// pub struct I2cLoopback<I2C> {
//     i2c_base: I2cBase<I2C>,
//     master_cfg: MasterConfig,
//     slave_cfg: SlaveConfig,
// }

i2c_base!(
    I2CA: (i2ca, PeripheralClocks::I2c0),
    I2CB: (i2cb, PeripheralClocks::I2c1),
);

//==================================================================================================
// I2C Master
//==================================================================================================

pub struct I2cMaster<I2C, ADDR = SevenBitAddress> {
    i2c_base: I2cBase<I2C>,
    _addr: PhantomData<ADDR>,
}

macro_rules! i2c_master {
    ($($I2CX:ident: ($i2cx:ident, $clk_enb:path),)+) => {
        $(
            impl<ADDR> I2cMaster<$I2CX, ADDR> {
                pub fn $i2cx(
                    i2c: $I2CX,
                    cfg: MasterConfig,
                    sys_clk: impl Into<Hertz> + Copy,
                    speed_mode: I2cSpeed,
                    sys_cfg: Option<&mut SYSCONFIG>,
                ) -> Self {
                    I2cMaster {
                        i2c_base: I2cBase::$i2cx(
                            i2c,
                            sys_clk,
                            speed_mode,
                            Some(&cfg),
                            None,
                            sys_cfg
                        ),
                        _addr: PhantomData,
                    }
                    .enable_master()
                }

                #[inline]
                pub fn cancel_transfer(&self) {
                    self.i2c_base
                        .i2c
                        .cmd
                        .write(|w| unsafe { w.bits(I2cCmd::Cancel as u32) });
                }

                #[inline]
                pub fn clear_tx_fifo(&self) {
                    self.i2c_base.i2c.fifo_clr.write(|w| w.txfifo().set_bit());
                }

                #[inline]
                pub fn clear_rx_fifo(&self) {
                    self.i2c_base.i2c.fifo_clr.write(|w| w.rxfifo().set_bit());
                }

                #[inline]
                pub fn enable_master(self) -> Self {
                    self.i2c_base.i2c.ctrl.modify(|_, w| w.enable().set_bit());
                    self
                }

                #[inline]
                pub fn disable_master(self) -> Self {
                    self.i2c_base.i2c.ctrl.modify(|_, w| w.enable().clear_bit());
                    self
                }

                #[inline(always)]
                fn load_fifo(&self, word: u8) {
                    self.i2c_base
                        .i2c
                        .data
                        .write(|w| unsafe { w.bits(word as u32) });
                }

                #[inline(always)]
                fn read_fifo(&self) -> u8 {
                    self.i2c_base.i2c.data.read().bits() as u8
                }

                fn error_handler_write(&mut self, init_cmd: &I2cCmd) {
                    self.clear_tx_fifo();
                    if *init_cmd == I2cCmd::Start {
                        self.i2c_base.stop_cmd()
                    }
                }

                fn write_base(
                    &mut self,
                    addr: I2cAddress,
                    init_cmd: I2cCmd,
                    bytes: impl IntoIterator<Item = u8>,
                ) -> Result<(), Error> {
                    let mut iter = bytes.into_iter();
                    // Load address
                    let (addr, addr_mode_bit) = I2cBase::<$I2CX>::unwrap_addr(addr);
                    self.i2c_base.i2c.address.write(|w| unsafe {
                        w.bits(I2cDirection::Send as u32 | (addr << 1) as u32 | addr_mode_bit)
                    });

                    self.i2c_base
                        .i2c
                        .cmd
                        .write(|w| unsafe { w.bits(init_cmd as u32) });
                    let mut load_if_next_available = || {
                        if let Some(next_byte) = iter.next() {
                            self.load_fifo(next_byte);
                        }
                    };
                    loop {
                        let status_reader = self.i2c_base.i2c.status.read();
                        if status_reader.arblost().bit_is_set() {
                            self.error_handler_write(&init_cmd);
                            return Err(Error::ArbitrationLost);
                        } else if status_reader.nackaddr().bit_is_set() {
                            self.error_handler_write(&init_cmd);
                            return Err(Error::NackAddr);
                        } else if status_reader.nackdata().bit_is_set() {
                            self.error_handler_write(&init_cmd);
                            return Err(Error::NackData);
                        } else if status_reader.idle().bit_is_set() {
                            return Ok(());
                        } else {
                            while !status_reader.txnfull().bit_is_set() {
                                load_if_next_available();
                            }
                        }
                    }
                }

                fn write_from_buffer(
                    &mut self,
                    init_cmd: I2cCmd,
                    addr: I2cAddress,
                    output: &[u8],
                ) -> Result<(), Error> {
                    let len = output.len();
                    // It should theoretically possible to transfer larger data sizes by tracking
                    // the number of sent words and setting it to 0x7fe as soon as only that many
                    // bytes are remaining. However, large transfer like this are not common. This
                    // feature will therefore not be supported for now.
                    if len > 0x7fe {
                        return Err(Error::DataTooLarge);
                    }
                    // Load number of words
                    self.i2c_base
                        .i2c
                        .words
                        .write(|w| unsafe { w.bits(len as u32) });
                    let mut bytes = output.iter();
                    // FIFO has a depth of 16. We load slightly above the trigger level
                    // but not all of it because the transaction might fail immediately
                    const FILL_DEPTH: usize = 12;

                    // load the FIFO
                    for _ in 0..core::cmp::min(FILL_DEPTH, len) {
                        self.load_fifo(*bytes.next().unwrap());
                    }

                    self.write_base(addr, init_cmd, output.iter().cloned())
                }

                fn load_from_iter(
                    &self, bytes: impl IntoIterator<Item = u8>
                ) -> impl IntoIterator<Item = u8> {
                    let mut iter = bytes.into_iter();
                    // FIFO has a depth of 16. We load slightly above the trigger level
                    // but not all of it because the transaction might fail immediately
                    const FILL_DEPTH: usize = 12;

                    // load the FIFO
                    for _ in 0..FILL_DEPTH {
                        if let Some(next_byte) = iter.next() {
                            self.load_fifo(next_byte);
                        } else {
                            break;
                        }
                    }
                    iter
                }

                fn read_internal(&mut self, addr: I2cAddress, buffer: &mut [u8]) -> Result<(), Error> {
                    let len = buffer.len();
                    // It should theoretically possible to transfer larger data sizes by tracking
                    // the number of sent words and setting it to 0x7fe as soon as only that many
                    // bytes are remaining. However, large transfer like this are not common. This
                    // feature will therefore not be supported for now.
                    if len > 0x7fe {
                        return Err(Error::DataTooLarge);
                    }
                    // Clear the receive FIFO
                    self.clear_rx_fifo();

                    // Load number of words
                    self.i2c_base
                        .i2c
                        .words
                        .write(|w| unsafe { w.bits(len as u32) });
                    let (addr, addr_mode_bit) = match addr {
                        I2cAddress::Regular(addr) => (addr as u16, 0 << 15),
                        I2cAddress::TenBit(addr) => (addr, 1 << 15),
                    };
                    // Load address
                    self.i2c_base.i2c.address.write(|w| unsafe {
                        w.bits(I2cDirection::Read as u32 | (addr << 1) as u32 | addr_mode_bit)
                    });

                    let mut buf_iter = buffer.iter_mut();
                    let mut read_bytes = 0;
                    // Start receive transfer
                    self.i2c_base
                        .i2c
                        .cmd
                        .write(|w| unsafe { w.bits(I2cCmd::StartWithStop as u32) });
                    let mut read_if_next_available = || {
                        if let Some(next_byte) = buf_iter.next() {
                            *next_byte = self.read_fifo();
                        }
                    };
                    loop {
                        let status_reader = self.i2c_base.i2c.status.read();
                        if status_reader.arblost().bit_is_set() {
                            self.clear_rx_fifo();
                            return Err(Error::ArbitrationLost);
                        } else if status_reader.nackaddr().bit_is_set() {
                            self.clear_rx_fifo();
                            return Err(Error::NackAddr);
                        } else if status_reader.idle().bit_is_set() {
                            if read_bytes != len {
                                return Err(Error::InsufficientDataReceived);
                            }
                            return Ok(());
                        } else if status_reader.rxnempty().bit_is_set() {
                            read_if_next_available();
                            read_bytes += 1;
                        }
                    }
                }
            }

            //======================================================================================
            // Embedded HAL I2C implementations
            //======================================================================================

            impl Write<SevenBitAddress> for I2cMaster<$I2CX, SevenBitAddress> {
                type Error = Error;

                fn write(&mut self, addr: u8, output: &[u8]) -> Result<(), Self::Error> {
                    self.write_from_buffer(I2cCmd::StartWithStop, I2cAddress::Regular(addr), output)
                }
            }

            impl Write<TenBitAddress> for I2cMaster<$I2CX, TenBitAddress> {
                type Error = Error;

                fn write(&mut self, addr: u16, output: &[u8]) -> Result<(), Self::Error> {
                    self.write_from_buffer(I2cCmd::StartWithStop, I2cAddress::TenBit(addr), output)
                }
            }

            impl Read<SevenBitAddress> for I2cMaster<$I2CX, SevenBitAddress> {
                type Error = Error;

                fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
                    self.read_internal(I2cAddress::Regular(address), buffer)
                }
            }

            impl Read<TenBitAddress> for I2cMaster<$I2CX, TenBitAddress> {
                type Error = Error;

                fn read(&mut self, address: u16, buffer: &mut [u8]) -> Result<(), Self::Error> {
                    self.read_internal(I2cAddress::TenBit(address), buffer)
                }
            }

            impl WriteRead<SevenBitAddress> for I2cMaster<$I2CX, SevenBitAddress> {
                type Error = Error;

                fn write_read(
                    &mut self,
                    address: u8,
                    bytes: &[u8],
                    buffer: &mut [u8],
                ) -> Result<(), Self::Error> {
                    let addr = I2cAddress::Regular(address);
                    self.write_from_buffer(I2cCmd::Start, addr, bytes)?;
                    self.read_internal(addr, buffer)
                }
            }

            impl WriteRead<TenBitAddress> for I2cMaster<$I2CX, TenBitAddress> {
                type Error = Error;

                fn write_read(
                    &mut self,
                    address: u16,
                    bytes: &[u8],
                    buffer: &mut [u8],
                ) -> Result<(), Self::Error> {
                    let addr = I2cAddress::TenBit(address);
                    self.write_from_buffer(I2cCmd::Start, addr, bytes)?;
                    self.read_internal(addr, buffer)
                }
            }

            impl WriteIter<SevenBitAddress> for I2cMaster<$I2CX, SevenBitAddress> {
                type Error = Error;

                fn write<B>(&mut self, address: u8, bytes: B) -> Result<(), Self::Error>
                where
                    B: IntoIterator<Item = u8>,
                {
                    let iter = self.load_from_iter(bytes);
                    self.write_base(I2cAddress::Regular(address), I2cCmd::Start, iter)
                }
            }

            impl WriteIterRead<SevenBitAddress> for I2cMaster<$I2CX, SevenBitAddress> {
                type Error = Error;
                fn write_iter_read<B>(
                    &mut self,
                    address: u8,
                    bytes: B,
                    buffer: &mut [u8],
                ) -> Result<(), Self::Error>
                where
                    B: IntoIterator<Item = u8>,
                {
                    let iter = self.load_from_iter(bytes);
                    let addr = I2cAddress::Regular(address);
                    self.write_base(addr, I2cCmd::Start, iter)?;
                    self.read_internal(addr, buffer)
                }
            }
        )+
    }
}

i2c_master!(
    I2CA: (i2ca, PeripheralClocks::I2c0),
    I2CB: (i2cb, PeripheralClocks::I2c1),
);

//==================================================================================================
// I2C Slave
//==================================================================================================

pub struct I2cSlave<I2C, ADDR = SevenBitAddress> {
    i2c_base: I2cBase<I2C>,
    _addr: PhantomData<ADDR>,
}

macro_rules! i2c_slave {
    ($($I2CX:ident: ($i2cx:ident, $i2cx_slave:ident),)+) => {
        $(
            impl<ADDR> I2cSlave<$I2CX, ADDR> {
                fn $i2cx_slave(
                    i2c: $I2CX,
                    cfg: SlaveConfig,
                    sys_clk: impl Into<Hertz>,
                    speed_mode: I2cSpeed,
                    sys_cfg: Option<&mut SYSCONFIG>,
                ) -> Self {
                    I2cSlave {
                        i2c_base: I2cBase::$i2cx(
                            i2c,
                            sys_clk,
                            speed_mode,
                            None,
                            Some(&cfg),
                            sys_cfg
                        ),
                        _addr: PhantomData,
                    }
                    .enable_slave()
                }

                #[inline]
                pub fn enable_slave(self) -> Self {
                    self.i2c_base
                        .i2c
                        .s0_ctrl
                        .modify(|_, w| w.enable().set_bit());
                    self
                }

                #[inline]
                pub fn disable_slave(self) -> Self {
                    self.i2c_base
                        .i2c
                        .s0_ctrl
                        .modify(|_, w| w.enable().clear_bit());
                    self
                }

                #[inline(always)]
                fn load_fifo(&self, word: u8) {
                    self.i2c_base
                        .i2c
                        .s0_data
                        .write(|w| unsafe { w.bits(word as u32) });
                }

                #[inline(always)]
                fn read_fifo(&self) -> u8 {
                    self.i2c_base.i2c.s0_data.read().bits() as u8
                }

                #[inline]
                fn clear_tx_fifo(&self) {
                    self.i2c_base
                        .i2c
                        .s0_fifo_clr
                        .write(|w| w.txfifo().set_bit());
                }

                #[inline]
                fn clear_rx_fifo(&self) {
                    self.i2c_base
                        .i2c
                        .s0_fifo_clr
                        .write(|w| w.rxfifo().set_bit());
                }

                /// Get the last address that was matched by the slave control and the corresponding
                /// master direction
                pub fn last_address(&self) -> (I2cDirection, u32) {
                    let bits = self.i2c_base.i2c.s0_lastaddress.read().bits();
                    match bits & 0x01 {
                        0 => (I2cDirection::Send, bits >> 1),
                        1 => (I2cDirection::Read, bits >> 1),
                        _ => (I2cDirection::Send, bits >> 1),
                    }
                }

                pub fn write(&mut self, output: &[u8]) -> Result<(), Error> {
                    let len = output.len();
                    // It should theoretically possible to transfer larger data sizes by tracking
                    // the number of sent words and setting it to 0x7fe as soon as only that many
                    // bytes are remaining. However, large transfer like this are not common. This
                    // feature will therefore not be supported for now.
                    if len > 0x7fe {
                        return Err(Error::DataTooLarge);
                    }
                    let mut bytes = output.iter();
                    // FIFO has a depth of 16. We load slightly above the trigger level
                    // but not all of it because the transaction might fail immediately
                    const FILL_DEPTH: usize = 12;

                    // load the FIFO
                    for _ in 0..core::cmp::min(FILL_DEPTH, len) {
                        self.load_fifo(*bytes.next().unwrap());
                    }

                    let status_reader = self.i2c_base.i2c.s0_status.read();
                    let mut load_if_next_available = || {
                        if let Some(next_byte) = bytes.next() {
                            self.load_fifo(*next_byte);
                        }
                    };
                    loop {
                        if status_reader.nackdata().bit_is_set() {
                            self.clear_tx_fifo();
                            return Err(Error::NackData);
                        } else if status_reader.idle().bit_is_set() {
                            return Ok(());
                        } else {
                            while !status_reader.txnfull().bit_is_set() {
                                load_if_next_available();
                            }
                        }
                    }
                }

                pub fn read(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
                    let len = buffer.len();
                    // It should theoretically possible to transfer larger data sizes by tracking
                    // the number of sent words and setting it to 0x7fe as soon as only that many
                    // bytes are remaining. However, large transfer like this are not common. This
                    // feature will therefore not be supported for now.
                    if len > 0x7fe {
                        return Err(Error::DataTooLarge);
                    }
                    // Clear the receive FIFO
                    self.clear_rx_fifo();

                    let mut buf_iter = buffer.iter_mut();
                    let mut read_bytes = 0;
                    let mut read_if_next_available = || {
                        if let Some(next_byte) = buf_iter.next() {
                            *next_byte = self.read_fifo();
                        }
                    };
                    loop {
                        let status_reader = self.i2c_base.i2c.s0_status.read();
                        if status_reader.idle().bit_is_set() {
                            if read_bytes != len {
                                return Err(Error::InsufficientDataReceived);
                            }
                            return Ok(());
                        } else if status_reader.rxnempty().bit_is_set() {
                            read_bytes += 1;
                            read_if_next_available();
                        }
                    }
                }
            }


            impl I2cSlave<$I2CX, SevenBitAddress> {
                /// Create a new I2C slave for seven bit addresses
                ///
                /// Returns a [`Error::WrongAddrMode`] error if a ten bit address is passed
                pub fn i2ca(
                    i2c: $I2CX,
                    cfg: SlaveConfig,
                    sys_clk: impl Into<Hertz>,
                    speed_mode: I2cSpeed,
                    sys_cfg: Option<&mut SYSCONFIG>,
                ) -> Result<Self, Error> {
                    if let I2cAddress::TenBit(_) = cfg.addr {
                        return Err(Error::WrongAddrMode);
                    }
                    Ok(Self::$i2cx_slave(i2c, cfg, sys_clk, speed_mode, sys_cfg))
                }
            }

            impl I2cSlave<$I2CX, TenBitAddress> {
                pub fn $i2cx(
                    i2c: $I2CX,
                    cfg: SlaveConfig,
                    sys_clk: impl Into<Hertz>,
                    speed_mode: I2cSpeed,
                    sys_cfg: Option<&mut SYSCONFIG>,
                ) -> Self {
                    Self::$i2cx_slave(i2c, cfg, sys_clk, speed_mode, sys_cfg)
                }
            }
        )+
    }
}

i2c_slave!(I2CA: (i2ca, i2ca_slave), I2CB: (i2cb, i2cb_slave),);
