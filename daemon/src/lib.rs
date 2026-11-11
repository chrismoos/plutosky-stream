//! The on-board daemon for plutosky-stream.
//!
//! It owns the AD9363, the VITA packetizers and depacketizers in the PL, and
//! the GEM slot fast path, and serves the gRPC control service in
//! `proto/plutosky/v1/control.proto` with a UDP discovery responder beside it. Everything the gateware exposes is reached by CSR name, resolved from
//! the `csr.json` of whichever bitstream is loaded.

#[cfg(feature = "hardware")]
pub mod ad9361;
pub mod control;
pub mod csr;
pub mod daemon;
pub mod fake_gateware;
pub mod gem;
pub mod net;
pub mod radio;
pub mod rate_filter;
pub mod stream;
pub mod testing;
pub mod time;
