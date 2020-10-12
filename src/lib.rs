//! 802.15.4 Radio using the nRF52 radio peripheral

#![no_std]

#[cfg(feature = "52833")]
pub use nrf52833_pac as pac;

#[cfg(feature = "52840")]
pub use nrf52840_pac as pac;

pub mod radio;
pub mod timer;
