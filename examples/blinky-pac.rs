
//! Blinky examples using only the PAC
//!
//! Additional note on LEDs:
//! Pulling the GPIOs low makes the LEDs blink. See REB1
//! schematic for more details.
#![no_main]
#![no_std]

use cortex_m_rt::entry;
use panic_halt as _;
use va108xx as pac;

// REB LED pin definitions. All on port A
const LED_D2: u32 = 1 << 10;
const LED_D3: u32 = 1 << 7;
const LED_D4: u32 = 1 << 6;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    // Enable all peripheral clocks
    dp.SYSCONFIG
        .peripheral_clk_enable
        .modify(|_, w| unsafe { w.bits(0xffffffff) });
    dp.PORTA
        .dir()
        .modify(|_, w| unsafe { w.bits(LED_D2 | LED_D3 | LED_D4) });
    dp.PORTA
        .datamask()
        .modify(|_, w| unsafe { w.bits(LED_D2 | LED_D3 | LED_D4) });
    for _ in 0..10 {
        dp.PORTA
            .clrout()
            .write(|w| unsafe { w.bits(LED_D2 | LED_D3 | LED_D4) });
        cortex_m::asm::delay(5_000_000);
        dp.PORTA
            .setout()
            .write(|w| unsafe { w.bits(LED_D2 | LED_D3 | LED_D4) });
        cortex_m::asm::delay(5_000_000);
    }
    loop {
        dp.PORTA
            .togout()
            .write(|w| unsafe { w.bits(LED_D2 | LED_D3 | LED_D4) });
        cortex_m::asm::delay(25_000_000);
    }
}
