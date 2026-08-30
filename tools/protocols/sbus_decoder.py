"""
SBUS Protocol Decoder
Decodes SBUS (Futaba) protocol frames
"""

def unpack_channels(payload):
    """Unpack 16 channels from SBUS frame (11-bit packed)"""
    if len(payload) < 22:
        return [992] * 16
    
    ch = [0] * 16
    # SBUS uses 11-bit values (0-2047), we'll convert to CRSF range (172-1811)
    ch[0]  = (payload[0] | payload[1] << 8) & 0x07FF
    ch[1]  = (payload[1] >> 3 | payload[2] << 5) & 0x07FF
    ch[2]  = (payload[2] >> 6 | payload[3] << 2 | payload[4] << 10) & 0x07FF
    ch[3]  = (payload[4] >> 1 | payload[5] << 7) & 0x07FF
    ch[4]  = (payload[5] >> 4 | payload[6] << 4) & 0x07FF
    ch[5]  = (payload[6] >> 7 | payload[7] << 1 | payload[8] << 9) & 0x07FF
    ch[6]  = (payload[8] >> 2 | payload[9] << 6) & 0x07FF
    ch[7]  = (payload[9] >> 5 | payload[10] << 3) & 0x07FF
    ch[8]  = (payload[11] | payload[12] << 8) & 0x07FF
    ch[9]  = (payload[12] >> 3 | payload[13] << 5) & 0x07FF
    ch[10] = (payload[13] >> 6 | payload[14] << 2 | payload[15] << 10) & 0x07FF
    ch[11] = (payload[15] >> 1 | payload[16] << 7) & 0x07FF
    ch[12] = (payload[16] >> 4 | payload[17] << 4) & 0x07FF
    ch[13] = (payload[17] >> 7 | payload[18] << 1 | payload[19] << 9) & 0x07FF
    ch[14] = (payload[19] >> 2 | payload[20] << 6) & 0x07FF
    ch[15] = (payload[20] >> 5 | payload[21] << 3) & 0x07FF
    
    # Convert from SBUS (0-2047) to CRSF (172-1811)
    for i in range(16):
        ch[i] = int((ch[i] / 2047.0) * (1811 - 172) + 172)
    
    return ch

def decode_frame(ser):
    """
    Decode a single SBUS frame from serial port
    Returns: (channels, error) where channels is list of 16 values or None
    """
    # Look for SBUS header (0x0F)
    header = ser.read(1)
    if not header or header[0] != 0x0F:
        return None, "Invalid header"
    
    # Read 24 more bytes (22 data + 1 flags + 1 footer)
    rest = ser.read(24)
    if len(rest) != 24:
        return None, "Incomplete frame"
    
    # Check footer (should be 0x00)
    if rest[23] != 0x00:
        return None, "Invalid footer"
    
    # Extract channel data (22 bytes)
    payload = rest[:22]
    channels = unpack_channels(payload)
    
    return channels, None

class SBUSDecoder:
    """SBUS Protocol Decoder"""
    
    def __init__(self):
        self.name = "SBUS"
        self.baudrate = 100000
        self.channel_min = 172
        self.channel_max = 1811
        self.channel_mid = 992
    
    def decode(self, ser):
        """Decode frame and return channels or None"""
        channels, error = decode_frame(ser)
        return channels
    
    def get_config(self):
        """Get protocol configuration"""
        return {
            'name': self.name,
            'baudrate': self.baudrate,
            'channel_min': self.channel_min,
            'channel_max': self.channel_max,
            'channel_mid': self.channel_mid
        }

