//! # API for the UART peripheral
//!
//! ## Examples
//!
//! - [UART example](https://egit.irs.uni-stuttgart.de/rust/va108xx-hal/src/branch/main/examples/uart.rs)
use core::{convert::Infallible, ptr};
use core::{marker::PhantomData, ops::Deref};
use libm::floorf;

use crate::clock::enable_peripheral_clock;
pub use crate::utility::IrqCfg;
use crate::{
    clock,
    gpio::pins::{
        AltFunc1, AltFunc2, AltFunc3, Pin, PA16, PA17, PA18, PA19, PA2, PA26, PA27, PA3, PA30,
        PA31, PA8, PA9, PB18, PB19, PB20, PB21, PB22, PB23, PB6, PB7, PB8, PB9,
    },
    pac::{uarta as uart_base, IRQSEL, SYSCONFIG, UARTA, UARTB},
    prelude::*,
    time::{Bps, Hertz},
    utility::unmask_irq,
};

use embedded_hal::{blocking, serial};

//==================================================================================================
// Type-Level support
//==================================================================================================

pub trait Pins<UART> {}

impl Pins<UARTA> for (Pin<PA9, AltFunc2>, Pin<PA8, AltFunc2>) {}
impl Pins<UARTA> for (Pin<PA17, AltFunc3>, Pin<PA16, AltFunc3>) {}
impl Pins<UARTA> for (Pin<PA31, AltFunc3>, Pin<PA30, AltFunc3>) {}

impl Pins<UARTA> for (Pin<PB9, AltFunc1>, Pin<PB8, AltFunc1>) {}
impl Pins<UARTA> for (Pin<PB23, AltFunc1>, Pin<PB22, AltFunc1>) {}

impl Pins<UARTB> for (Pin<PA3, AltFunc2>, Pin<PA2, AltFunc2>) {}
impl Pins<UARTB> for (Pin<PA19, AltFunc3>, Pin<PA18, AltFunc3>) {}
impl Pins<UARTB> for (Pin<PA27, AltFunc3>, Pin<PA26, AltFunc3>) {}

impl Pins<UARTB> for (Pin<PB7, AltFunc1>, Pin<PB6, AltFunc1>) {}
impl Pins<UARTB> for (Pin<PB19, AltFunc2>, Pin<PB18, AltFunc2>) {}
impl Pins<UARTB> for (Pin<PB21, AltFunc1>, Pin<PB20, AltFunc1>) {}

//==================================================================================================
// Regular Definitions
//==================================================================================================

#[derive(Debug)]
pub enum Error {
    Overrun,
    FramingError,
    ParityError,
    BreakCondition,
    TransferPending,
    BufferTooShort,
}

#[derive(Debug, PartialEq, Copy, Clone)]
pub enum Event {
    // Receiver FIFO interrupt enable. Generates interrupt
    // when FIFO is at least half full. Half full is defined as FIFO
    // count >= RXFIFOIRQTRG
    RxFifoHalfFull,
    // Framing error, Overrun error, Parity Error and Break error
    RxError,
    // Event for timeout condition: Data in the FIFO and no receiver
    // FIFO activity for 4 character times
    RxTimeout,

    // Transmitter FIFO interrupt enable. Generates interrupt
    // when FIFO is at least half full. Half full is defined as FIFO
    // count >= TXFIFOIRQTRG
    TxFifoHalfFull,
    // FIFO overflow error
    TxError,
    // Generate interrupt when transmit FIFO is empty and TXBUSY is 0
    TxEmpty,
    // Interrupt when CTSn changes value
    TxCts,
}

#[derive(Copy, Clone, PartialEq)]
pub enum Parity {
    None,
    Odd,
    Even,
}

#[derive(Copy, Clone, PartialEq)]
pub enum StopBits {
    One = 0,
    Two = 1,
}

#[derive(Copy, Clone, PartialEq)]
pub enum WordSize {
    Five = 0,
    Six = 1,
    Seven = 2,
    Eight = 3,
}

pub struct Config {
    pub baudrate: Bps,
    pub parity: Parity,
    pub stopbits: StopBits,
    // When false, use standard 16x baud clock, other 8x baud clock
    pub baud8: bool,
    pub wordsize: WordSize,
    pub enable_tx: bool,
    pub enable_rx: bool,
}

impl Config {
    pub fn baudrate(mut self, baudrate: Bps) -> Self {
        self.baudrate = baudrate;
        self
    }

    pub fn parity_none(mut self) -> Self {
        self.parity = Parity::None;
        self
    }

    pub fn parity_even(mut self) -> Self {
        self.parity = Parity::Even;
        self
    }

    pub fn parity_odd(mut self) -> Self {
        self.parity = Parity::Odd;
        self
    }

    pub fn stopbits(mut self, stopbits: StopBits) -> Self {
        self.stopbits = stopbits;
        self
    }

    pub fn wordsize(mut self, wordsize: WordSize) -> Self {
        self.wordsize = wordsize;
        self
    }

    pub fn baud8(mut self, baud: bool) -> Self {
        self.baud8 = baud;
        self
    }
}

impl Default for Config {
    fn default() -> Config {
        let baudrate = 115_200_u32.bps();
        Config {
            baudrate,
            parity: Parity::None,
            stopbits: StopBits::One,
            baud8: false,
            wordsize: WordSize::Eight,
            enable_tx: true,
            enable_rx: true,
        }
    }
}

impl From<Bps> for Config {
    fn from(baud: Bps) -> Self {
        Config::default().baudrate(baud)
    }
}

//==================================================================================================
// IRQ Definitions
//==================================================================================================

pub struct IrqInfo {
    rx_len: usize,
    rx_idx: usize,
    irq_cfg: IrqCfg,
    mode: IrqReceptionMode,
}

pub enum IrqResultMask {
    Complete = 0,
    Overflow = 1,
    FramingError = 2,
    ParityError = 3,
    Break = 4,
    Timeout = 5,
    Addr9 = 6,
}
pub struct IrqResult {
    raw_res: u32,
    pub bytes_read: usize,
}

impl IrqResult {
    #[inline]
    pub fn raw_result(&self) -> u32 {
        self.raw_res
    }

    #[inline]
    pub(crate) fn clear_result(&mut self) {
        self.raw_res = 0;
    }
    #[inline]
    pub(crate) fn set_result(&mut self, flag: IrqResultMask) {
        self.raw_res |= 1 << flag as u32;
    }

    #[inline]
    pub fn complete(&self) -> bool {
        if ((self.raw_res >> IrqResultMask::Complete as u32) & 0x01) == 0x00 {
            return true;
        }
        false
    }

    #[inline]
    pub fn error(&self) -> bool {
        if self.overflow_error() || self.framing_error() || self.parity_error() {
            return true;
        }
        false
    }

    #[inline]
    pub fn overflow_error(&self) -> bool {
        if ((self.raw_res >> IrqResultMask::Overflow as u32) & 0x01) == 0x01 {
            return true;
        }
        false
    }

    #[inline]
    pub fn framing_error(&self) -> bool {
        if ((self.raw_res >> IrqResultMask::FramingError as u32) & 0x01) == 0x01 {
            return true;
        }
        false
    }

    #[inline]
    pub fn parity_error(&self) -> bool {
        if ((self.raw_res >> IrqResultMask::ParityError as u32) & 0x01) == 0x01 {
            return true;
        }
        false
    }

    #[inline]
    pub fn timeout(&self) -> bool {
        if ((self.raw_res >> IrqResultMask::Timeout as u32) & 0x01) == 0x01 {
            return true;
        }
        false
    }
}

#[derive(Debug, PartialEq)]
pub enum IrqReceptionMode {
    Idle,
    FixedLen,
    VarLen,
}

//==================================================================================================
// UART implementation
//==================================================================================================

/// Serial abstraction
pub struct Uart<UART, PINS> {
    uart: UART,
    pins: PINS,
    tx: Tx<UART>,
    rx: Rx<UART>,
}

pub struct UartWithIrq<UART, PINS> {
    uart_base: Uart<UART, PINS>,
    irq_info: IrqInfo,
}

/// Serial receiver
pub struct Rx<UART> {
    _usart: PhantomData<UART>,
}

/// Serial transmitter
pub struct Tx<UART> {
    _usart: PhantomData<UART>,
}

impl<UART> Rx<UART> {
    fn new() -> Self {
        Self {
            _usart: PhantomData,
        }
    }
}

impl<UART> Tx<UART> {
    fn new() -> Self {
        Self {
            _usart: PhantomData,
        }
    }
}

pub trait Instance: Deref<Target = uart_base::RegisterBlock> {
    fn ptr() -> *const uart_base::RegisterBlock;
}

impl<UART, PINS> Uart<UART, PINS>
where
    UART: Instance,
{
    /// This function assumes that the peripheral clock was alredy enabled
    /// in the SYSCONFIG register
    fn init(self, config: Config, sys_clk: Hertz) -> Self {
        let baud_multiplier = match config.baud8 {
            false => 16,
            true => 8,
        };
        let x = sys_clk.0 as f32 / (config.baudrate.0 * baud_multiplier) as f32;
        let integer_part = floorf(x) as u32;
        let frac = floorf((64.0 * (x - integer_part as f32) + 0.5) as f32) as u32;
        self.uart
            .clkscale
            .write(|w| unsafe { w.bits(integer_part * 64 + frac) });

        let (paren, pareven) = match config.parity {
            Parity::None => (false, false),
            Parity::Odd => (true, false),
            Parity::Even => (true, true),
        };
        let stopbits = match config.stopbits {
            StopBits::One => false,
            StopBits::Two => true,
        };
        let wordsize = config.wordsize as u8;
        let baud8 = config.baud8;
        self.uart.ctrl.write(|w| {
            w.paren().bit(paren);
            w.pareven().bit(pareven);
            w.stopbits().bit(stopbits);
            w.baud8().bit(baud8);
            unsafe { w.wordsize().bits(wordsize) }
        });
        let (txenb, rxenb) = (config.enable_tx, config.enable_rx);
        // Clear the FIFO
        self.uart.fifo_clr.write(|w| {
            w.rxfifo().set_bit();
            w.txfifo().set_bit()
        });
        self.uart.enable.write(|w| {
            w.rxenable().bit(rxenb);
            w.txenable().bit(txenb)
        });
        self
    }

    #[inline]
    pub fn enable_rx(&mut self) {
        self.uart.enable.write(|w| w.rxenable().set_bit());
    }

    #[inline]
    pub fn disable_rx(&mut self) {
        self.uart.enable.write(|w| w.rxenable().set_bit());
    }

    #[inline]
    pub fn enable_tx(&mut self) {
        self.uart.enable.write(|w| w.txenable().set_bit());
    }

    #[inline]
    pub fn disable_tx(&mut self) {
        self.uart.enable.write(|w| w.txenable().set_bit());
    }

    #[inline]
    pub fn clear_rx_fifo(&mut self) {
        self.uart.fifo_clr.write(|w| w.rxfifo().set_bit());
    }

    #[inline]
    pub fn clear_tx_fifo(&mut self) {
        self.uart.fifo_clr.write(|w| w.txfifo().set_bit());
    }

    #[inline]
    pub fn clear_rx_status(&mut self) {
        self.uart.fifo_clr.write(|w| w.rxsts().set_bit());
    }

    #[inline]
    pub fn clear_tx_status(&mut self) {
        self.uart.fifo_clr.write(|w| w.txsts().set_bit());
    }

    pub fn listen(self, event: Event) -> Self {
        self.uart.irq_enb.modify(|_, w| match event {
            Event::RxError => w.irq_rx_status().set_bit(),
            Event::RxFifoHalfFull => w.irq_rx().set_bit(),
            Event::RxTimeout => w.irq_rx_to().set_bit(),
            Event::TxEmpty => w.irq_tx_empty().set_bit(),
            Event::TxError => w.irq_tx_status().set_bit(),
            Event::TxFifoHalfFull => w.irq_tx().set_bit(),
            Event::TxCts => w.irq_tx_cts().set_bit(),
        });
        self
    }

    pub fn unlisten(self, event: Event) -> Self {
        self.uart.irq_enb.modify(|_, w| match event {
            Event::RxError => w.irq_rx_status().clear_bit(),
            Event::RxFifoHalfFull => w.irq_rx().clear_bit(),
            Event::RxTimeout => w.irq_rx_to().clear_bit(),
            Event::TxEmpty => w.irq_tx_empty().clear_bit(),
            Event::TxError => w.irq_tx_status().clear_bit(),
            Event::TxFifoHalfFull => w.irq_tx().clear_bit(),
            Event::TxCts => w.irq_tx_cts().clear_bit(),
        });
        self
    }

    pub fn release(self) -> (UART, PINS) {
        // Clear the FIFO
        self.uart.fifo_clr.write(|w| {
            w.rxfifo().set_bit();
            w.txfifo().set_bit()
        });
        self.uart.enable.write(|w| {
            w.rxenable().clear_bit();
            w.txenable().clear_bit()
        });
        (self.uart, self.pins)
    }

    pub fn split(self) -> (Tx<UART>, Rx<UART>) {
        (self.tx, self.rx)
    }
}

macro_rules! uart_impl {
    ($($UARTX:ident: ($uartx:ident, $clk_enb_enum:path),)+) => {
        $(
            impl Instance for $UARTX {
                fn ptr() -> *const uart_base::RegisterBlock {
                    $UARTX::ptr() as *const _
                }
            }

            impl<PINS: Pins<$UARTX>> Uart<$UARTX, PINS> {
                pub fn $uartx(
                    uart: $UARTX,
                    pins: PINS,
                    config: impl Into<Config>,
                    syscfg: &mut SYSCONFIG,
                    sys_clk: impl Into<Hertz>
                ) -> Self
                {
                    enable_peripheral_clock(syscfg, $clk_enb_enum);
                    Uart { uart, pins, tx: Tx::new(), rx: Rx::new() }.init(
                        config.into(), sys_clk.into()
                    )
                }
            }
        )+
    }
}

impl<PINS> Uart<UARTA, PINS> {
    pub fn into_uart_with_irq(self, irq_cfg: IrqCfg) -> UartWithIrq<UARTA, PINS> {
        UartWithIrq {
            uart_base: self,
            irq_info: IrqInfo {
                rx_len: 0,
                rx_idx: 0,
                irq_cfg,
                mode: IrqReceptionMode::Idle,
            },
        }
    }
}

impl<PINS> UartWithIrq<UARTA, PINS> {
    pub fn read_fixed_len_using_irq(
        &mut self,
        max_len: usize,
        enb_timeout_irq: bool,
        irqsel: Option<&mut IRQSEL>,
    ) -> Result<(), Error> {
        if self.irq_info.mode != IrqReceptionMode::Idle {
            return Err(Error::TransferPending);
        }
        self.irq_info.rx_idx = 0;
        self.irq_info.rx_len = max_len;
        self.enable_rx_irq_sources(enb_timeout_irq);
        if let Some(irqsel) = irqsel {
            if self.irq_info.irq_cfg.route {
                irqsel.uart[0].write(|w| unsafe { w.bits(self.irq_info.irq_cfg.irq as u32) });
            }
        }
        if self.irq_info.irq_cfg.enable {
            unmask_irq(self.irq_info.irq_cfg.irq);
        }
        Ok(())
    }

    #[inline]
    fn enable_rx_irq_sources(&mut self, timeout: bool) {
        self.uart_base.uart.irq_enb.modify(|_, w| {
            if timeout {
                w.irq_rx_to().set_bit();
            }
            w.irq_rx_status().set_bit();
            w.irq_rx().set_bit()
        });
    }

    #[inline]
    fn disable_rx_irq_sources(&mut self) {
        self.uart_base.uart.irq_enb.modify(|_, w| {
            w.irq_rx_to().clear_bit();
            w.irq_rx_status().clear_bit();
            w.irq_rx().clear_bit()
        });
    }

    pub fn cancel_transfer(&mut self) {
        // Disable IRQ
        cortex_m::peripheral::NVIC::mask(self.irq_info.irq_cfg.irq);
        self.disable_rx_irq_sources();
        self.uart_base.clear_tx_fifo();
        self.irq_info.rx_idx = 0;
        self.irq_info.rx_len = 0;
    }

    pub fn irq_handler(&mut self, res: &mut IrqResult, buf: &mut [u8]) -> Result<(), Error> {
        if buf.len() < self.irq_info.rx_len {
            return Err(Error::BufferTooShort);
        }

        let mut rx_status = self.uart_base.uart.rxstatus.read();
        let _tx_status = self.uart_base.uart.txstatus.read();
        let irq_end = self.uart_base.uart.irq_end.read();

        let enb_status = self.uart_base.uart.enable.read();
        let rx_enabled = enb_status.rxenable().bit_is_set();
        let _tx_enabled = enb_status.txenable().bit_is_set();
        res.clear_result();
        if irq_end.irq_rx().bit_is_set() && rx_status.rdavl().bit_is_set() {
            let mut rd_avl = rx_status.rdavl();
            // While there is data in the FIFO, write it into the reception buffer
            while self.irq_info.rx_idx < self.irq_info.rx_len && rd_avl.bit_is_set() {
                buf[self.irq_info.rx_idx] = nb::block!(self.uart_base.read()).unwrap();
                rd_avl = self.uart_base.uart.rxstatus.read().rdavl();
                self.irq_info.rx_idx += 1;
                if self.irq_info.rx_idx == self.irq_info.rx_len {
                    self.disable_rx_irq_sources();
                    res.bytes_read = self.irq_info.rx_len;
                    res.set_result(IrqResultMask::Complete);
                    self.irq_info.rx_idx = 0;
                    self.irq_info.rx_len = 0;
                    return Ok(());
                }
            }
        }

        // RX transfer not complete, check for RX errors
        if (self.irq_info.rx_idx < self.irq_info.rx_len) && rx_enabled {
            // Read status register again, might have changed since reading received data
            rx_status = self.uart_base.uart.rxstatus.read();
            if rx_status.rxovr().bit_is_set() {
                res.set_result(IrqResultMask::Overflow);
            }
            if rx_status.rxfrm().bit_is_set() {
                res.set_result(IrqResultMask::FramingError);
            }
            if rx_status.rxpar().bit_is_set() {
                res.set_result(IrqResultMask::ParityError);
            }
            if rx_status.rxbrk().bit_is_set() {
                res.set_result(IrqResultMask::Break);
            }
            if rx_status.rxto().bit_is_set() {
                // A timeout has occured but there might be some leftover data in the FIFO,
                // so read that data as well
                while self.irq_info.rx_idx < self.irq_info.rx_len && rx_status.rdavl().bit_is_set()
                {
                    buf[self.irq_info.rx_idx] = nb::block!(self.uart_base.read()).unwrap();
                    self.irq_info.rx_idx += 1;
                }
                res.bytes_read = self.irq_info.rx_idx;
                res.set_result(IrqResultMask::Timeout);
                res.set_result(IrqResultMask::Complete);
            }

            if res.raw_res != 0 {
                self.disable_rx_irq_sources();
            }
        }

        self.uart_base
            .uart
            .irq_clr
            .write(|w| unsafe { w.bits(irq_end.bits()) });
        Ok(())
    }
}

uart_impl! {
    UARTA: (uarta, clock::PeripheralClocks::Uart0),
    UARTB: (uartb, clock::PeripheralClocks::Uart1),
}

impl<UART> Tx<UART> where UART: Instance {}

impl<UART, PINS> serial::Write<u8> for Uart<UART, PINS>
where
    UART: Instance,
{
    type Error = Infallible;
    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.tx.write(word)
    }
    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.tx.flush()
    }
}

impl<UART: Instance, PINS> blocking::serial::write::Default<u8> for Uart<UART, PINS> {}

impl<UART: Instance> serial::Write<u8> for Tx<UART> {
    type Error = Infallible;

    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        let reader = unsafe { &(*UART::ptr()) }.txstatus.read();
        if reader.wrrdy().bit_is_clear() {
            return Err(nb::Error::WouldBlock);
        } else {
            // DPARITY bit not supported yet
            unsafe {
                // NOTE(unsafe) atomic write to data register
                // NOTE(write_volatile) 8-bit write that's not
                // possible through the svd2rust API
                ptr::write_volatile(&(*UART::ptr()).data as *const _ as *mut _, word);
            }
        }
        Ok(())
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        let reader = unsafe { &(*UART::ptr()) }.txstatus.read();
        if reader.wrbusy().bit_is_clear() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl<UART: Instance, PINS> serial::Read<u8> for Uart<UART, PINS> {
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Error> {
        self.rx.read()
    }
}

impl<UART: Instance> serial::Read<u8> for Rx<UART> {
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Error> {
        let uart = unsafe { &(*UART::ptr()) };
        let status_reader = uart.rxstatus.read();
        let err = if status_reader.rxovr().bit_is_set() {
            Some(Error::Overrun)
        } else if status_reader.rxfrm().bit_is_set() {
            Some(Error::FramingError)
        } else if status_reader.rxpar().bit_is_set() {
            Some(Error::ParityError)
        } else {
            None
        };
        if let Some(err) = err {
            // The status code is always related to the next bit for the framing
            // and parity status bits. We have to read the DATA register
            // so that the next status reflects the next DATA word
            // For overrun error, we read as well to clear the peripheral
            uart.data.read().bits();
            Err(err.into())
        } else if status_reader.rdavl().bit_is_set() {
            let data = uart.data.read().bits();
            Ok((data & 0xff) as u8)
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl<UART> core::fmt::Write for Tx<UART>
where
    Tx<UART>: embedded_hal::serial::Write<u8>,
{
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        s.as_bytes()
            .iter()
            .try_for_each(|c| nb::block!(self.write(*c)))
            .map_err(|_| core::fmt::Error)
    }
}
