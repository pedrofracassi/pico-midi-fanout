//! Shared logic for the MIDI fan-out box.
//!
//! * [`StreamParser`] turns the serial MIDI byte stream arriving on the UART
//!   back into 4-byte USB-MIDI 1.0 event packets (virtual cable 0), handling
//!   running status, real-time bytes interleaved anywhere, system common
//!   messages and SysEx.
//! * [`StateCache`] remembers the last value of every CC and pitch bend so a
//!   rebooted show PC can ask for the full fader state.
//!
//! Serial MIDI is used on the wire (rather than raw USB packets) because it
//! is self-synchronising: after a glitch the next status byte realigns the
//! stream, so there is no framing to get wrong.

#![cfg_attr(not(test), no_std)]

/// Number of data bytes that follow a status byte (0 for single-byte messages).
const fn data_len(status: u8) -> u8 {
    match status {
        0xC0..=0xDF => 1,       // program change, channel pressure
        0x80..=0xEF => 2,       // note off/on, poly AT, CC, pitch bend
        0xF1 | 0xF3 => 1,       // MTC quarter frame, song select
        0xF2 => 2,              // song position pointer
        _ => 0,
    }
}

/// Serial MIDI byte stream → USB-MIDI event packets.
pub struct StreamParser {
    /// Current status for channel messages (running status) or system common. 0 = none.
    status: u8,
    buf: [u8; 3],
    len: u8,
    in_sysex: bool,
}

impl StreamParser {
    pub const fn new() -> Self {
        Self { status: 0, buf: [0; 3], len: 0, in_sysex: false }
    }

    /// Feed one byte; returns a USB-MIDI event packet when one is complete.
    pub fn push(&mut self, byte: u8) -> Option<[u8; 4]> {
        // Real-time messages may appear anywhere, even inside other messages.
        if byte >= 0xF8 {
            return Some([0x0F, byte, 0, 0]);
        }

        if byte & 0x80 != 0 {
            return self.status_byte(byte);
        }

        // Data byte.
        if self.in_sysex {
            self.buf[self.len as usize] = byte;
            self.len += 1;
            if self.len == 3 {
                self.len = 0;
                return Some([0x04, self.buf[0], self.buf[1], self.buf[2]]);
            }
            return None;
        }
        if self.status == 0 {
            return None; // stray data byte, e.g. we started mid-message
        }
        if self.len == 0 {
            // Running status: reuse the previous status byte.
            self.buf[0] = self.status;
            self.len = 1;
        }
        self.buf[self.len as usize] = byte;
        self.len += 1;

        let status = self.status;
        if self.len < 1 + data_len(status) {
            return None;
        }
        self.len = 0;
        let cin = match status {
            0x80..=0xEF => status >> 4,
            _ if data_len(status) == 1 => 0x2,
            _ => 0x3,
        };
        if status >= 0xF0 {
            self.status = 0; // system common messages do not set running status
        }
        let (d1, d2) = (self.buf[1], if data_len(status) == 2 { self.buf[2] } else { 0 });
        Some([cin, status, d1, d2])
    }

    fn status_byte(&mut self, byte: u8) -> Option<[u8; 4]> {
        if self.in_sysex {
            self.in_sysex = false;
            let n = self.len as usize;
            self.len = 0;
            if byte == 0xF7 {
                // SysEx end: CIN 5/6/7 carries 1/2/3 bytes including the F7.
                let mut p = [0x05 + n as u8, 0, 0, 0];
                p[1..1 + n].copy_from_slice(&self.buf[..n]);
                p[1 + n] = 0xF7;
                return Some(p);
            }
            // Any other status byte aborts the SysEx; handle it normally below.
        }

        self.len = 0;
        match byte {
            0xF0 => {
                self.status = 0;
                self.in_sysex = true;
                self.buf[0] = 0xF0;
                self.len = 1;
                None
            }
            0xF6 => {
                self.status = 0;
                Some([0x05, 0xF6, 0, 0]) // tune request
            }
            0xF1 | 0xF2 | 0xF3 | 0x80..=0xEF => {
                self.status = byte;
                self.buf[0] = byte;
                self.len = 1;
                None
            }
            _ => {
                // 0xF4/0xF5 undefined, stray 0xF7
                self.status = 0;
                None
            }
        }
    }
}

impl Default for StreamParser {
    fn default() -> Self {
        Self::new()
    }
}

/// Last-known value of every CC and pitch bend, per channel.
pub struct StateCache {
    cc: [[u8; 128]; 16],
    pb: [u16; 16],
}

const UNKNOWN_CC: u8 = 0xFF;
const UNKNOWN_PB: u16 = 0xFFFF;

impl StateCache {
    pub const fn new() -> Self {
        Self { cc: [[UNKNOWN_CC; 128]; 16], pb: [UNKNOWN_PB; 16] }
    }

    /// Record a USB-MIDI event packet if it is a CC or pitch bend.
    pub fn observe(&mut self, p: &[u8; 4]) {
        let ch = (p[1] & 0x0F) as usize;
        match p[0] & 0x0F {
            0xB => self.cc[ch][(p[2] & 0x7F) as usize] = p[3] & 0x7F,
            0xE => self.pb[ch] = (p[2] & 0x7F) as u16 | ((p[3] & 0x7F) as u16) << 7,
            _ => {}
        }
    }

    /// Last CC value on channel `ch` (0-15), if one has been seen.
    pub fn cc(&self, ch: u8, controller: u8) -> Option<u8> {
        let v = self.cc[(ch & 0x0F) as usize][(controller & 0x7F) as usize];
        (v != UNKNOWN_CC).then_some(v)
    }

    /// Last 14-bit pitch bend on channel `ch` (0-15), if one has been seen.
    pub fn pitch_bend(&self, ch: u8) -> Option<u16> {
        let v = self.pb[(ch & 0x0F) as usize];
        (v != UNKNOWN_PB).then_some(v)
    }
}

impl Default for StateCache {
    fn default() -> Self {
        Self::new()
    }
}

/// Channel (1-16) and controller number of the "please replay the state" request
/// that TouchDesigner sends to the device Pico on startup.
pub const REPLAY_CHANNEL: u8 = 16;
pub const REPLAY_CC: u8 = 127;

/// True if a 4-byte USB-MIDI event packet from the PC is the replay request.
pub fn is_replay_request(p: &[u8]) -> bool {
    p.len() == 4 && p[0] & 0x0F == 0xB && p[1] == 0xB0 | (REPLAY_CHANNEL - 1) && p[2] == REPLAY_CC
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse(bytes: &[u8]) -> Vec<[u8; 4]> {
        let mut p = StreamParser::new();
        bytes.iter().filter_map(|&b| p.push(b)).collect()
    }

    #[test]
    fn control_change_and_running_status() {
        assert_eq!(
            parse(&[0xB0, 7, 100, 8, 50]),
            vec![[0x0B, 0xB0, 7, 100], [0x0B, 0xB0, 8, 50]]
        );
    }

    #[test]
    fn two_byte_channel_message() {
        assert_eq!(parse(&[0xC3, 5, 6]), vec![[0x0C, 0xC3, 5, 0], [0x0C, 0xC3, 6, 0]]);
    }

    #[test]
    fn pitch_bend() {
        assert_eq!(parse(&[0xE1, 0x00, 0x40]), vec![[0x0E, 0xE1, 0x00, 0x40]]);
    }

    #[test]
    fn realtime_inside_message() {
        assert_eq!(
            parse(&[0x90, 60, 0xF8, 100]),
            vec![[0x0F, 0xF8, 0, 0], [0x09, 0x90, 60, 100]]
        );
    }

    #[test]
    fn ignores_data_before_first_status() {
        assert_eq!(parse(&[7, 100, 0xB0, 1, 2]), vec![[0x0B, 0xB0, 1, 2]]);
    }

    #[test]
    fn system_common_cancels_running_status() {
        assert_eq!(parse(&[0xB0, 1, 2, 0xF3, 4, 5, 6]), vec![[0x0B, 0xB0, 1, 2], [0x02, 0xF3, 4, 0]]);
        assert_eq!(parse(&[0xF2, 1, 2]), vec![[0x03, 0xF2, 1, 2]]);
        assert_eq!(parse(&[0xF6]), vec![[0x05, 0xF6, 0, 0]]);
    }

    #[test]
    fn sysex_lengths() {
        // 3 bytes total -> a single end packet (CIN 7)
        assert_eq!(parse(&[0xF0, 0x7E, 0xF7]), vec![[0x07, 0xF0, 0x7E, 0xF7]]);
        assert_eq!(parse(&[0xF0, 1, 2, 0xF7]), vec![[0x04, 0xF0, 1, 2], [0x05, 0xF7, 0, 0]]);
        assert_eq!(parse(&[0xF0, 1, 2, 3, 0xF7]), vec![[0x04, 0xF0, 1, 2], [0x06, 3, 0xF7, 0]]);
        assert_eq!(parse(&[0xF0, 1, 2, 3, 4, 0xF7]), vec![[0x04, 0xF0, 1, 2], [0x07, 3, 4, 0xF7]]);
        assert_eq!(parse(&[0xF0, 0xF7]), vec![[0x06, 0xF0, 0xF7, 0]]);
    }

    #[test]
    fn sysex_aborted_by_status() {
        assert_eq!(parse(&[0xF0, 1, 0xB0, 7, 9]), vec![[0x0B, 0xB0, 7, 9]]);
    }

    #[test]
    fn round_trip_via_usb_packet_data() {
        // What the host Pico does: take packet bytes by CIN length and send them.
        fn packet_bytes(p: &[u8; 4]) -> &[u8] {
            let n = match p[0] & 0x0F {
                0x2 | 0x6 | 0xC | 0xD => 2,
                0x3 | 0x4 | 0x7 | 0x8..=0xB | 0xE => 3,
                0x5 | 0xF => 1,
                _ => 0,
            };
            &p[1..1 + n]
        }
        let original = [[0x0B, 0xB0, 1, 2], [0x0E, 0xE5, 3, 4], [0x04, 0xF0, 1, 2], [0x06, 3, 0xF7, 0], [0x0F, 0xFA, 0, 0]];
        let wire: Vec<u8> = original.iter().flat_map(|p| packet_bytes(p).to_vec()).collect();
        assert_eq!(parse(&wire), original.to_vec());
    }

    #[test]
    fn cache_and_replay_request() {
        let mut c = StateCache::new();
        assert_eq!(c.cc(0, 7), None);
        c.observe(&[0x0B, 0xB2, 7, 99]);
        c.observe(&[0x0E, 0xE2, 0x7F, 0x7F]);
        c.observe(&[0x09, 0x92, 60, 100]); // note on: ignored
        assert_eq!(c.cc(2, 7), Some(99));
        assert_eq!(c.pitch_bend(2), Some(0x3FFF));
        assert_eq!(c.pitch_bend(0), None);

        assert!(is_replay_request(&[0x0B, 0xBF, 127, 0]));
        assert!(!is_replay_request(&[0x0B, 0xB0, 127, 0]));
        assert!(!is_replay_request(&[0x0B, 0xBF, 126, 0]));
    }
}
