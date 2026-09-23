//! iceoryx2 backend (ADR-0006, M2): the proven `fusion/bench/iox2`
//! pattern behind [`Transport`].
//!
//! `send_copy` (one copy — same as the ring's producer memcpy), buffer
//! 1024 + `subscriber_max_buffer_size` 1024 (matches the ring depth so the
//! comparison isolates mechanism), spinning `receive()` like the ring's
//! spin. Service name is a constructor parameter so tests
//! (`otolith/test`) never collide with the bench (`otolith/bench_f`,
//! itself distinct from the C++ bench's `otolith/bench`).

use iceoryx2::port::publisher::Publisher;
use iceoryx2::port::subscriber::Subscriber;
use iceoryx2::prelude::*;
use iceoryx2::service::port_factory::publish_subscribe::PortFactory;

use super::{BenchMsg, Transport};

pub const SERVICE_F: &str = "otolith/bench_f";
pub const SERVICE_TEST: &str = "otolith/test";
pub const BUFFER: usize = 1024;

type Payload = [u8; BenchMsg::BYTES];

/// Role-selected endpoint. Each side owns its node + port; the unused side
/// is `None` and its method returns `Err` (bench mains only call their own
/// side — same discipline as the C++ mains, where each role links only
/// what it uses).
pub struct Iox2Transport {
    _node: Node<ipc::Service>,
    publisher: Option<Publisher<ipc::Service, Payload, ()>>,
    subscriber: Option<Subscriber<ipc::Service, Payload, ()>>,
}

fn open_service(
    node: &Node<ipc::Service>,
    service: &str,
) -> Result<PortFactory<ipc::Service, Payload, ()>, Box<dyn std::error::Error>> {
    Ok(node
        .service_builder(&service.try_into()?)
        .publish_subscribe::<Payload>()
        .subscriber_max_buffer_size(BUFFER)
        .open_or_create()?)
}

impl Iox2Transport {
    pub fn publisher(service: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let node = NodeBuilder::new().create::<ipc::Service>()?;
        let svc = open_service(&node, service)?;
        Ok(Self {
            _node: node,
            publisher: Some(svc.publisher_builder().create()?),
            subscriber: None,
        })
    }

    pub fn subscriber(service: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let node = NodeBuilder::new().create::<ipc::Service>()?;
        let svc = open_service(&node, service)?;
        Ok(Self {
            _node: node,
            publisher: None,
            subscriber: Some(svc.subscriber_builder().buffer_size(BUFFER).create()?),
        })
    }
}

impl Transport for Iox2Transport {
    fn send(&mut self, msg: &BenchMsg) -> std::io::Result<bool> {
        let Some(p) = &self.publisher else {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                "send on subscriber endpoint",
            ));
        };
        // Delivery failure (congestion) counts as a drop — same accounting
        // as the ring's full-slot drop. Pacing stays sacred either way.
        match p.send_copy(*msg.as_bytes()) {
            Ok(_) => Ok(true),
            Err(_) => Ok(false),
        }
    }

    fn try_recv(&mut self, out: &mut BenchMsg) -> std::io::Result<bool> {
        let Some(s) = &self.subscriber else {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                "try_recv on publisher endpoint",
            ));
        };
        match s.receive() {
            Ok(Some(sample)) => {
                *out = BenchMsg::from_bytes(sample.payload());
                Ok(true)
            }
            Ok(None) => Ok(false),
            Err(e) => Err(std::io::Error::other(format!("receive: {e:?}"))),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Real iceoryx2 loopback in one process (pub+sub, distinct roles, same
    // service). Needs live IPC resources: Miri-excluded like the SHM tests.
    #[test]
    #[cfg_attr(miri, ignore)]
    fn loopback_ordered() {
        let svc = format!("{}/{}", SERVICE_TEST, std::process::id());
        let mut tx = Iox2Transport::publisher(&svc).unwrap();
        let mut rx = Iox2Transport::subscriber(&svc).unwrap();
        for i in 0..100u64 {
            assert!(tx.send(&BenchMsg::new(i, i)).unwrap());
        }
        let mut out = BenchMsg::new(0, 0);
        for i in 0..100u64 {
            let mut got = false;
            for _ in 0..10000 {
                if rx.try_recv(&mut out).unwrap() {
                    got = true;
                    break;
                }
                std::hint::spin_loop();
            }
            assert!(got, "message {i} never arrived");
            assert_eq!(out.seq, i);
            assert_eq!(out.tx_ns, i);
        }
    }

    #[test]
    #[cfg_attr(miri, ignore)] // real IPC resources; same reason as loopback
    fn wrong_role_errors() {
        let svc = format!("{}/role/{}", SERVICE_TEST, std::process::id());
        let mut tx = Iox2Transport::publisher(&svc).unwrap();
        let mut m = BenchMsg::new(0, 0);
        assert!(tx.try_recv(&mut m).is_err());
    }
}
