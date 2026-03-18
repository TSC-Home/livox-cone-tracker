use std::net::UdpSocket;
use crate::hap::protocol;

/// Simulation replacement for LivoxConnection.
///
/// In `--sim` mode the Rust app skips the real LiDAR handshake and
/// just listens on UDP port 57000 for packets sent by `sim/bridge.py`
/// (or the ROS 2 bridge node). The packet format is identical to the
/// real Livox HAP data stream, so all detection/visualisation code is
/// unchanged.
pub struct SimConnection {
    data_socket: UdpSocket,
    recv_buf:    Vec<u8>,
}

impl SimConnection {
    pub fn new() -> std::io::Result<Self> {
        println!("[sim] Binding UDP data socket on 0.0.0.0:57000 …");
        let data_socket = UdpSocket::bind("0.0.0.0:57000")?;
        data_socket.set_nonblocking(true)?;
        println!("[sim] Ready. Waiting for packets from bridge.py …");
        Ok(Self {
            data_socket,
            recv_buf: vec![0u8; 65_536],
        })
    }

    /// No-op – only needed for real hardware heartbeat.
    pub fn maintain(&mut self) {}

    /// Drain the socket, decode all available Livox packets.
    pub fn receive_points(&mut self, points: &mut Vec<[f32; 4]>) -> usize {
        let before = points.len();
        loop {
            match self.data_socket.recv(&mut self.recv_buf) {
                Ok(n) if n > 0 => {
                    protocol::parse_point_data(&self.recv_buf[..n], points);
                }
                _ => break,
            }
        }
        points.len() - before
    }

    pub fn stop(&mut self) {
        println!("[sim] Simulation connection closed.");
    }
}
