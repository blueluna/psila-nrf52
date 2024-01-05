//! Psila support crate for nRF52 series of microcontrollers
//!
//! Currently supports nRF52840 and nRF52833
//!
//! The `radio` module contains a 802.15.4 implementation of the
//! nRF52 RADIO peripheral.
//!
//! The `timer` module contains a timer implementations using the
//! nRF52 TIMER peripheral.
//!

#![no_std]
#![warn(missing_docs)]

#[cfg(feature = "52833")]
pub use nrf52833_pac as pac;

#[cfg(feature = "52840")]
pub use nrf52840_pac as pac;

#[cfg(feature = "microbit")]
pub use microbit::pac as pac;

pub mod radio;
pub mod timer;
