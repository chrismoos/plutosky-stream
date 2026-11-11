//! The control plane: the gRPC service in `proto/plutosky/v1/control.proto`
//! and the UDP discovery responder.

#[allow(clippy::all, missing_docs)]
pub mod proto {
    tonic::include_proto!("plutosky.v1");
}

mod discovery;
mod error;
mod service;

pub use discovery::{DISCOVERY_PORT, DISCOVERY_PROBE, DiscoveryResponder};
pub use error::{ApiError, ErrorCode};
pub use service::{
    ControlService, DEFAULT_CONTROL_PORT, DEFAULT_STATUS_INTERVAL, SESSION_METADATA_KEY, serve,
    serve_on,
};
