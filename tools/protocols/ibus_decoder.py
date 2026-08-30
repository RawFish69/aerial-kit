"""
iBus Protocol Decoder
Decodes FlySky iBus protocol frames
"""

def decode_frame(ser):
    """
    Decode a single iBus frame from serial port
    Returns: (channels, error) where channels is list of 16 values or None
    """
    # Look for iBus header (0x20 0x40)
    header1 = ser.read(1)
    if not header1 or header1[0] != 0x20:
        return None, "Invalid header1"
    
    header2 = ser.read(1)
    if not header2 or header2[0] != 0x40:
        return None, "Invalid header2"
    
    # Read 28 bytes (14 channels * 2 bytes)
    channel_data = ser.read(28)
    if len(channel_data) != 28:
        return None, "Incomplete channel data"
    
    # Read checksum (2 bytes)
    checksum_bytes = ser.read(2)
    if len(checksum_bytes) != 2:
        return None, "Incomplete checksum"
    
    # Verify checksum
    received_checksum = checksum_bytes[0] | (checksum_bytes[1] << 8)
    calculated_checksum = 0xFFFF
    calculated_checksum -= header1[0]
    calculated_checksum -= header2[0]
    for byte in channel_data:
        calculated_checksum -= byte
    calculated_checksum &= 0xFFFF
    
    if received_checksum != calculated_checksum:
        return None, "Checksum mismatch"
    
    # Unpack channels (14 channels, little-endian)
    channels = [992] * 16
    for i in range(14):
        value = channel_data[i*2] | (channel_data[i*2+1] << 8)
        # iBus uses 1000-2000 range, convert to CRSF (172-1811)
        channels[i] = int(((value - 1000) / 1000.0) * (1811 - 172) + 172)
    
    return channels, None

class IBUSDecoder:
    """iBus Protocol Decoder"""
    
    def __init__(self):
        self.name = "iBus"
        self.baudrate = 115200
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

