use crc::{Crc, CRC_16_IBM_3740, CRC_32_ISO_HDLC};

const SOF: u8 = 0xAA;
const VERSION: u8 = 0x00;
const HEADER_LEN: usize = 24; // SOF(1)+Ver(1)+Len(2)+Seq(4)+CmdId(2)+CmdType(1)+Sender(1)+Resv(6)+CRC16(2)+CRC32(4)

const CRC16: Crc<u16> = Crc::<u16>::new(&CRC_16_IBM_3740);
const CRC32: Crc<u32> = Crc::<u32>::new(&CRC_32_ISO_HDLC);

#[derive(Debug, Clone)]
pub struct HapFrame {
    pub seq: u16,
    pub cmd_id: u16,
    pub cmd_type: u8,
    pub sender_type: u8,
    pub data: Vec<u8>,
}

impl HapFrame {
    pub fn new_request(seq: u16, cmd_id: u16, data: Vec<u8>) -> Self {
        Self {
            seq,
            cmd_id,
            cmd_type: 0, // REQ
            sender_type: 0, // Host
            data,
        }
    }

    pub fn serialize(&self) -> Vec<u8> {
        let total_len = HEADER_LEN + self.data.len();
        let mut buf = Vec::with_capacity(total_len);

        // Header up to CRC16 field (18 bytes)
        buf.push(SOF);
        buf.push(VERSION);
        buf.extend_from_slice(&(total_len as u16).to_le_bytes());
        buf.extend_from_slice(&(self.seq as u32).to_le_bytes());
        buf.extend_from_slice(&self.cmd_id.to_le_bytes());
        buf.push(self.cmd_type);
        buf.push(self.sender_type);
        buf.extend_from_slice(&[0u8; 6]); // reserved

        // CRC16 over first 18 bytes
        let header_crc = CRC16.checksum(&buf[..18]);
        buf.extend_from_slice(&header_crc.to_le_bytes());

        // CRC32 over data payload only (0 if no data)
        let data_crc = if self.data.is_empty() {
            0u32
        } else {
            CRC32.checksum(&self.data)
        };
        buf.extend_from_slice(&data_crc.to_le_bytes());

        // Data payload
        buf.extend_from_slice(&self.data);

        buf
    }

    pub fn parse(buf: &[u8]) -> Option<Self> {
        if buf.len() < HEADER_LEN {
            return None;
        }
        if buf[0] != SOF {
            return None;
        }

        let length = u16::from_le_bytes([buf[2], buf[3]]) as usize;
        if buf.len() < length {
            return None;
        }

        // Verify CRC16 over first 18 bytes
        let header_crc = u16::from_le_bytes([buf[18], buf[19]]);
        if CRC16.checksum(&buf[..18]) != header_crc {
            return None;
        }

        let data_len = length - HEADER_LEN;
        let data = if data_len > 0 {
            buf[HEADER_LEN..HEADER_LEN + data_len].to_vec()
        } else {
            Vec::new()
        };

        Some(Self {
            seq: u16::from_le_bytes([buf[4], buf[5]]),
            cmd_id: u16::from_le_bytes([buf[8], buf[9]]),
            cmd_type: buf[10],
            sender_type: buf[11],
            data,
        })
    }
}

/// Parse point cloud data packet from UDP.
/// Header (36 bytes): version(1) + length(2) + time_interval(2) + dot_num(2) +
///   udp_cnt(2) + frame_cnt(1) + data_type(1) + time_type(1) + rsvd(12) +
///   crc32(4) + timestamp(8)
/// Each point (Cartesian high, data_type=0x01):
///   x(i32) + y(i32) + z(i32) + reflectivity(u8) + tag(u8) = 14 bytes
pub fn parse_point_data(buf: &[u8], points: &mut Vec<[f32; 4]>) {
    let header_size = 36;
    if buf.len() <= header_size {
        return;
    }
    let payload = &buf[header_size..];
    const POINT_SIZE: usize = 14;

    let count = payload.len() / POINT_SIZE;
    points.reserve(count);

    for i in 0..count {
        let off = i * POINT_SIZE;
        if off + POINT_SIZE > payload.len() {
            break;
        }
        let x = i32::from_le_bytes([
            payload[off], payload[off + 1], payload[off + 2], payload[off + 3],
        ]) as f32 / 1000.0;
        let y = i32::from_le_bytes([
            payload[off + 4], payload[off + 5], payload[off + 6], payload[off + 7],
        ]) as f32 / 1000.0;
        let z = i32::from_le_bytes([
            payload[off + 8], payload[off + 9], payload[off + 10], payload[off + 11],
        ]) as f32 / 1000.0;
        let reflectivity = payload[off + 12] as f32;

        points.push([x, y, z, reflectivity]);
    }
}
