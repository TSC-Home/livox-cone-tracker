use std::net::UdpSocket;
use std::time::{Duration, Instant};

use super::commands;
use super::protocol::{self, HapFrame};

const LIDAR_CMD_PORT: u16 = 56000;
const HOST_CMD_PORT: u16 = 56000;
const HOST_DATA_PORT: u16 = 57000;

pub struct LivoxConnection {
    cmd_socket: UdpSocket,
    data_socket: UdpSocket,
    lidar_addr: String,
    seq: u16,
    recv_buf: Vec<u8>,
    stopped: bool,
}

impl LivoxConnection {
    pub fn connect(lidar_ip: &str, host_ip: [u8; 4]) -> std::io::Result<Self> {
        let cmd_socket = UdpSocket::bind(format!("0.0.0.0:{HOST_CMD_PORT}"))?;
        cmd_socket.set_nonblocking(false)?;
        cmd_socket.set_read_timeout(Some(Duration::from_secs(2)))?;

        let data_socket = UdpSocket::bind(format!("0.0.0.0:{HOST_DATA_PORT}"))?;
        data_socket.set_nonblocking(true)?;

        let lidar_addr = format!("{lidar_ip}:{LIDAR_CMD_PORT}");

        let mut conn = Self {
            cmd_socket,
            data_socket,
            lidar_addr,
            seq: 0,
            recv_buf: vec![0u8; 65536],
            stopped: false,
        };

        // Discovery
        let seq = conn.next_seq();
        conn.cmd_socket.send_to(&commands::discovery(seq), &conn.lidar_addr)?;
        conn.wait_ack("Discovery");

        // Set point data destination (raw IP format - confirmed working)
        let seq = conn.next_seq();
        conn.cmd_socket.send_to(
            &commands::set_point_data_dest(seq, host_ip, HOST_DATA_PORT),
            &conn.lidar_addr,
        )?;
        conn.wait_ack("Point data dest");

        // Set work mode to normal
        let seq = conn.next_seq();
        conn.cmd_socket.send_to(
            &commands::set_work_mode_normal(seq),
            &conn.lidar_addr,
        )?;
        conn.wait_ack("Work mode");

        conn.cmd_socket.set_nonblocking(true)?;
        Ok(conn)
    }

    fn next_seq(&mut self) -> u16 {
        let s = self.seq;
        self.seq = self.seq.wrapping_add(1);
        s
    }

    fn wait_ack(&mut self, label: &str) {
        let start = Instant::now();
        while start.elapsed() < Duration::from_secs(2) {
            match self.cmd_socket.recv_from(&mut self.recv_buf) {
                Ok((n, _)) => {
                    if let Some(frame) = HapFrame::parse(&self.recv_buf[..n]) {
                        if frame.cmd_type == 1 {
                            let ret = frame.data.first().copied().unwrap_or(255);
                            if ret == 0 {
                                println!("  {label}: OK");
                            } else {
                                let err_key = if frame.data.len() >= 3 {
                                    u16::from_le_bytes([frame.data[1], frame.data[2]])
                                } else { 0 };
                                println!("  {label}: FAIL ret={ret} key=0x{err_key:04X}");
                            }
                            return;
                        }
                    }
                }
                Err(_) => break,
            }
        }
        println!("  {label}: no ACK");
    }

    pub fn maintain(&mut self) {
        while self.cmd_socket.recv(&mut self.recv_buf).is_ok() {}
    }

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
        if self.stopped {
            return;
        }
        self.stopped = true;
        println!("  Sending standby command to LiDAR...");
        let _ = self.cmd_socket.set_nonblocking(false);
        let _ = self.cmd_socket.set_read_timeout(Some(Duration::from_secs(2)));

        // Send standby multiple times to make sure it arrives
        for _ in 0..5 {
            let seq = self.next_seq();
            let _ = self.cmd_socket.send_to(
                &commands::set_standby(seq), &self.lidar_addr);
            std::thread::sleep(Duration::from_millis(100));
        }
        self.wait_ack("Standby");
    }
}

impl Drop for LivoxConnection {
    fn drop(&mut self) {
        self.stop();
    }
}
