"""
PPM Protocol Decoder
Note: PPM is a GPIO-based protocol (pulse timing), not serial data.
This decoder would need special hardware to convert PPM to serial.

Options:
1. Use logic analyzer/oscilloscope to view PPM directly
2. Use ESP32 as PPM-to-Serial bridge (future implementation)
3. Read RC channels from flight controller telemetry
"""

class PPMDecoder:
    """
    PPM Protocol Decoder (requires hardware bridge)
    
    PPM is not serial data - it's GPIO pulses. To use this decoder,
    you would need to:
    1. Create ESP32 sketch that reads PPM on GPIO
    2. Converts PPM pulse timing to RC channels
    3. Outputs channels via serial (as CRSF or simple CSV format)
    4. Connect that ESP32's USB to computer
    
    For now, this is a placeholder. Use CRSF/SBUS/iBus for direct monitoring.
    """
    
    def __init__(self):
        self.name = "PPM"
        self.baudrate = 115200  # Would be the bridge's serial baud rate
        self.channel_min = 172
        self.channel_max = 1811
        self.channel_mid = 992
    
    def decode(self, ser):
        """
        Placeholder decoder - would need PPM-to-Serial bridge
        Returns None as PPM can't be decoded from serial directly
        """
        # Future: If you implement ESP32 PPM bridge that outputs
        # channels as simple text format like:
        # "992,992,172,992,992,992,992,992\n"
        # You could parse it here
        
        return None
    
    def get_config(self):
        """Get protocol configuration"""
        return {
            'name': self.name,
            'baudrate': self.baudrate,
            'channel_min': self.channel_min,
            'channel_max': self.channel_max,
            'channel_mid': self.channel_mid,
            'note': 'Requires hardware PPM-to-Serial bridge'
        }

