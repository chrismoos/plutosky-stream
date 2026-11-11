//! Link-layer facts the frame template needs: the host's MAC address and the
//! board's own, both read from the kernel rather than configured.
//!
//! The host's MAC comes from the neighbour table, which the kernel fills when
//! the host ARPs for the board before sending its first data packet. The entry
//! also includes the interface the host is reachable on, which is where the
//! board's own MAC comes from.

use std::fmt;
use std::io;
use std::net::Ipv4Addr;
use std::str::FromStr;

/// The kernel's IPv4 neighbour table.
const PROC_NET_ARP: &str = "/proc/net/arp";
/// Per-interface attributes, `<SYS_CLASS_NET>/<device>/address`.
const SYS_CLASS_NET: &str = "/sys/class/net";
/// Column order of `/proc/net/arp`: address, HW type, flags, HW address, mask,
/// device.
const ARP_COLUMN_IP: usize = 0;
const ARP_COLUMN_FLAGS: usize = 2;
const ARP_COLUMN_MAC: usize = 3;
const ARP_COLUMN_DEVICE: usize = 5;
const ARP_COLUMNS: usize = 6;
/// `ATF_COM`: the entry holds a resolved hardware address.
const ARP_FLAG_COMPLETE: u32 = 0x02;

const MAC_BYTES: usize = 6;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct MacAddr(pub [u8; MAC_BYTES]);

impl MacAddr {
    pub fn octets(self) -> [u8; MAC_BYTES] {
        self.0
    }
}

impl fmt::Display for MacAddr {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let [a, b, c, d, e, g] = self.0;
        write!(f, "{a:02x}:{b:02x}:{c:02x}:{d:02x}:{e:02x}:{g:02x}")
    }
}

impl FromStr for MacAddr {
    type Err = LinkError;

    fn from_str(text: &str) -> Result<Self, Self::Err> {
        let mut octets = [0u8; MAC_BYTES];
        let mut parts = text.trim().split(':');
        for octet in octets.iter_mut() {
            let part = parts.next().ok_or_else(|| LinkError::MalformedMac {
                text: text.to_string(),
            })?;
            *octet = u8::from_str_radix(part, 16).map_err(|_| LinkError::MalformedMac {
                text: text.to_string(),
            })?;
        }
        if parts.next().is_some() {
            return Err(LinkError::MalformedMac {
                text: text.to_string(),
            });
        }
        Ok(MacAddr(octets))
    }
}

#[derive(Debug, thiserror::Error)]
pub enum LinkError {
    #[error("`{text}` is not a MAC address")]
    MalformedMac { text: String },
    #[error("no neighbour entry for {ip}: the host has not sent a packet to the data port yet")]
    HostUnknown { ip: Ipv4Addr },
    #[error("read {path}: {source}")]
    Read { path: String, source: io::Error },
}

/// Who the frames are addressed to and where they leave from.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct HostLink {
    pub host_mac: MacAddr,
    pub board_mac: MacAddr,
    pub device: String,
}

/// Where the daemon learns link-layer addresses. The real implementation reads
/// the kernel's tables. The fake one answers from fixed values so the whole
/// daemon runs on a host with no board attached.
pub trait LinkResolver: Send + Sync {
    fn resolve(&self, host_ip: Ipv4Addr) -> Result<HostLink, LinkError>;
    /// MAC of an interface, for the discovery reply.
    fn interface_mac(&self, device: &str) -> Result<MacAddr, LinkError>;
}

/// Reads `/proc/net/arp` and `/sys/class/net`.
pub struct KernelLinkResolver;

impl LinkResolver for KernelLinkResolver {
    fn resolve(&self, host_ip: Ipv4Addr) -> Result<HostLink, LinkError> {
        let table = std::fs::read_to_string(PROC_NET_ARP).map_err(|source| LinkError::Read {
            path: PROC_NET_ARP.to_string(),
            source,
        })?;
        let (host_mac, device) = parse_arp_table(&table, host_ip)?;
        let board_mac = self.interface_mac(&device)?;
        Ok(HostLink {
            host_mac,
            board_mac,
            device,
        })
    }

    fn interface_mac(&self, device: &str) -> Result<MacAddr, LinkError> {
        let path = format!("{SYS_CLASS_NET}/{device}/address");
        let text = std::fs::read_to_string(&path).map_err(|source| LinkError::Read {
            path: path.clone(),
            source,
        })?;
        text.parse()
    }
}

/// Answers with fixed addresses, for `--fake` and tests.
pub struct FixedLinkResolver {
    pub host_mac: MacAddr,
    pub board_mac: MacAddr,
    pub device: String,
}

impl LinkResolver for FixedLinkResolver {
    fn resolve(&self, _host_ip: Ipv4Addr) -> Result<HostLink, LinkError> {
        Ok(HostLink {
            host_mac: self.host_mac,
            board_mac: self.board_mac,
            device: self.device.clone(),
        })
    }

    fn interface_mac(&self, _device: &str) -> Result<MacAddr, LinkError> {
        Ok(self.board_mac)
    }
}

/// Finds a complete neighbour entry for `host_ip` and returns its hardware
/// address and interface.
fn parse_arp_table(table: &str, host_ip: Ipv4Addr) -> Result<(MacAddr, String), LinkError> {
    let wanted = host_ip.to_string();
    for line in table.lines().skip(1) {
        let columns: Vec<&str> = line.split_whitespace().collect();
        if columns.len() < ARP_COLUMNS || columns[ARP_COLUMN_IP] != wanted {
            continue;
        }
        let flags = columns[ARP_COLUMN_FLAGS]
            .strip_prefix("0x")
            .and_then(|hex| u32::from_str_radix(hex, 16).ok())
            .unwrap_or(0);
        if flags & ARP_FLAG_COMPLETE == 0 {
            continue;
        }
        let mac: MacAddr = columns[ARP_COLUMN_MAC].parse()?;
        return Ok((mac, columns[ARP_COLUMN_DEVICE].to_string()));
    }
    Err(LinkError::HostUnknown { ip: host_ip })
}

#[cfg(test)]
mod tests {
    use super::*;

    const TABLE: &str = "\
IP address       HW type     Flags       HW address            Mask     Device
192.168.2.9      0x1         0x0         00:00:00:00:00:00     *        eth0
192.168.2.10     0x1         0x2         aa:bb:cc:dd:ee:01     *        eth0
192.168.3.10     0x1         0x2         aa:bb:cc:dd:ee:02     *        eth1
";

    #[test]
    fn a_complete_entry_yields_the_host_mac_and_its_interface() {
        let (mac, device) = parse_arp_table(TABLE, Ipv4Addr::new(192, 168, 2, 10)).unwrap();
        assert_eq!(mac.to_string(), "aa:bb:cc:dd:ee:01");
        assert_eq!(device, "eth0");
        let (_, device) = parse_arp_table(TABLE, Ipv4Addr::new(192, 168, 3, 10)).unwrap();
        assert_eq!(device, "eth1");
    }

    #[test]
    fn an_incomplete_or_absent_entry_reports_the_host_as_unknown() {
        for ip in [Ipv4Addr::new(192, 168, 2, 9), Ipv4Addr::new(10, 0, 0, 1)] {
            assert!(matches!(
                parse_arp_table(TABLE, ip),
                Err(LinkError::HostUnknown { .. })
            ));
        }
    }

    #[test]
    fn mac_addresses_round_trip_through_their_text_form() {
        let mac: MacAddr = "00:1a:2B:3c:4d:5e".parse().unwrap();
        assert_eq!(mac.octets(), [0x00, 0x1a, 0x2b, 0x3c, 0x4d, 0x5e]);
        assert_eq!(mac.to_string(), "00:1a:2b:3c:4d:5e");
        for bad in ["", "00:1a:2b:3c:4d", "00:1a:2b:3c:4d:5e:6f", "zz:..."] {
            assert!(bad.parse::<MacAddr>().is_err(), "{bad} parsed");
        }
    }
}
