//! Empty RTIC project template
#![no_main]
#![no_std]

#[rtic::app(device = pac)]
mod app {
    use panic_rtt_target as _;
    use rtt_target::{rprintln, rtt_init_default};
    use va108xx_hal::pac;

    #[local]
    struct Local {}

    #[shared]
    struct Shared {}

    #[init]
    fn init(_ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        rtt_init_default!();
        rprintln!("-- Vorago RTIC template --");
        (Shared {}, Local {}, init::Monotonics())
    }

    // `shared` cannot be accessed from this context
    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {}
    }
}
