//! Startup-only URDF/SRDF import for fixed-size serial robot models.
//!
//! Enable `std` to parse XML. Pass the resulting chain and collision model to
//! allocation-free runtime algorithms; XML, names, and named states stay outside the loop.

#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(feature = "std")]
mod import;
#[cfg(feature = "std")]
pub use import::CuRobotDescription;
#[cfg(feature = "std")]
pub use import::CuSrdfImport;
#[cfg(feature = "std")]
pub use import::CuUrdfImport;
