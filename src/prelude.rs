//! Prelude
pub use embedded_hal::prelude::*;

// embedded-hal doesn’t yet have v2 in its prelude, so we need to
// export it ourselves
pub use embedded_hal::digital::v2::InputPin as _embedded_hal_gpio_InputPin;
pub use embedded_hal::digital::v2::OutputPin as _embedded_hal_gpio_OutputPin;
pub use embedded_hal::digital::v2::StatefulOutputPin as _embedded_hal_gpio_StatefulOutputPin;
pub use embedded_hal::digital::v2::ToggleableOutputPin as _embedded_hal_gpio_ToggleableOutputPin;

pub use crate::time::U32Ext as _va108xx_hal_time_U32Ext;
