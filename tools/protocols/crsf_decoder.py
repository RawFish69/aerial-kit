"""
CRSF Protocol Decoder
Decodes CRSF (Crossfire/ELRS) protocol frames
"""

def crc8(data):
    """Calculate CRC8 DVB-S2 for CRSF"""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) if (crc & 0x80) else (crc << 1)
    return crc & 0xFF

def unpack_channels(payload):
    """Unpack 16 channels from CRSF RC frame (11-bit packed)"""
    if len(payload) < 22:
        return [992] * 16
    
    ch = [0] * 16
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
    return ch

def decode_frame(ser):
    """
    Decode a single CRSF frame from serial port
    Returns: (channels, error) where channels is list of 16 values or None
    """
    # Read sync byte
    sync = ser.read(1)
    if not sync or sync[0] not in [0x00, 0xC8, 0xEA, 0xEC]:
        return None, "Invalid sync"
    
    # Read length
    length_byte = ser.read(1)
    if not length_byte:
        return None, "No length"
    length = length_byte[0]
    
    if length < 2 or length > 64:
        return None, f"Invalid length: {length}"
    
    # Read rest of frame
    rest = ser.read(length)
    if len(rest) != length:
        return None, "Incomplete frame"
    
    frame = bytes([sync[0], length]) + rest
    
    # Validate CRC
    calc_crc = crc8(frame[2:-1])
    if calc_crc != frame[-1]:
        return None, "CRC mismatch"
    
    # Check if RC channels frame
    if frame[2] == 0x16:  # RC channels type
        payload = frame[3:-1]
        channels = unpack_channels(payload)
        return channels, None
    
    return None, f"Not RC frame (type: 0x{frame[2]:02X})"

class CRSFDecoder:
    """CRSF Protocol Decoder"""
    
    def __init__(self):
        self.name = "CRSF"
        self.baudrate = 420000
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

