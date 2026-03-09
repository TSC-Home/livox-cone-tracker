use super::protocol::HapFrame;

const CMD_LIDAR_SEARCH: u16 = 0x0000;
const CMD_WORK_MODE_CONTROL: u16 = 0x0100;

const KEY_POINT_DATA_HOST_IP_CFG: u16 = 0x0006;
const KEY_WORK_MODE: u16 = 0x001A;

/// Build a key-value command (cmd 0x0100).
/// Format: key_num(u16) + rsvd(u16) + [key(u16) + length(u16) + value(N)]...
fn kv_command(seq: u16, pairs: &[(u16, &[u8])]) -> Vec<u8> {
    let mut data = Vec::new();
    data.extend_from_slice(&(pairs.len() as u16).to_le_bytes());
    data.extend_from_slice(&0u16.to_le_bytes()); // reserved
    for (key, value) in pairs {
        data.extend_from_slice(&key.to_le_bytes());
        data.extend_from_slice(&(value.len() as u16).to_le_bytes());
        data.extend_from_slice(value);
    }
    HapFrame::new_request(seq, CMD_WORK_MODE_CONTROL, data).serialize()
}

/// Discovery.
pub fn discovery(seq: u16) -> Vec<u8> {
    HapFrame::new_request(seq, CMD_LIDAR_SEARCH, vec![]).serialize()
}

/// Set point data destination: raw IP(4) + host_port(u16) + lidar_port(u16).
pub fn set_point_data_dest(seq: u16, host_ip: [u8; 4], data_port: u16) -> Vec<u8> {
    let mut val = Vec::with_capacity(8);
    val.extend_from_slice(&host_ip);
    val.extend_from_slice(&data_port.to_le_bytes());
    val.extend_from_slice(&data_port.to_le_bytes());
    kv_command(seq, &[(KEY_POINT_DATA_HOST_IP_CFG, &val)])
}

/// Set work mode. Try with u8 value.
pub fn set_work_mode_normal(seq: u16) -> Vec<u8> {
    kv_command(seq, &[(KEY_WORK_MODE, &[0x01])])
}

/// Set standby mode.
pub fn set_standby(seq: u16) -> Vec<u8> {
    kv_command(seq, &[(KEY_WORK_MODE, &[0x03])])
}
