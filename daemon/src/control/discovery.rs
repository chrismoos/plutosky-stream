//! The UDP discovery responder.
//!
//! A host broadcasts a fixed probe to [`DISCOVERY_PORT`] and every board on
//! the segment answers with a `DiscoveryReply` protobuf.

use std::io;
use std::net::{SocketAddr, UdpSocket};
use std::sync::Arc;

use prost::Message;

use crate::daemon::Daemon;

/// Port the probe is broadcast to.
pub const DISCOVERY_PORT: u16 = 30001;

/// The probe payload, matched exactly.
pub const DISCOVERY_PROBE: &[u8] = b"plutosky-stream?";

/// Longest datagram the responder reads, which bounds what a stray sender can
/// make it copy.
const PROBE_BUFFER_BYTES: usize = 256;

pub struct DiscoveryResponder {
    socket: UdpSocket,
    daemon: Arc<Daemon>,
}

impl DiscoveryResponder {
    pub fn bind(daemon: Arc<Daemon>, port: u16) -> io::Result<Self> {
        let socket = UdpSocket::bind(SocketAddr::from(([0, 0, 0, 0], port)))
            .map_err(|e| io::Error::new(e.kind(), format!("bind discovery port {port}: {e}")))?;
        socket.set_broadcast(true)?;
        Ok(Self { socket, daemon })
    }

    pub fn local_addr(&self) -> io::Result<SocketAddr> {
        self.socket.local_addr()
    }

    /// Answers probes until the socket fails.
    pub fn run(&self) -> io::Result<()> {
        let mut buffer = [0u8; PROBE_BUFFER_BYTES];
        loop {
            let (len, from) = match self.socket.recv_from(&mut buffer) {
                Ok(received) => received,
                Err(error) => {
                    log::warn!("discovery receive failed: {error}");
                    continue;
                }
            };
            if buffer[..len].trim_ascii() != DISCOVERY_PROBE {
                continue;
            }
            let reply = self.daemon.discovery_reply().encode_to_vec();
            if let Err(error) = self.socket.send_to(&reply, from) {
                log::warn!("discovery reply to {from} failed: {error}");
            }
        }
    }
}
