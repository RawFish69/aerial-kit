#!/usr/bin/env python3
"""
CRSF Live Dashboard - Simple real-time CRSF monitor
Usage: python crsf_live.py COM3
"""

import serial
import sys
import time

# ANSI codes
CLEAR = '\033[2J\033[H'
HOME = '\033[H'
HIDE = '\033[?25l'
SHOW = '\033[?25h'
CLEAR_LINE = '\033[2K'

def crc8(data):
    """Calculate CRC8 DVB-S2"""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) if (crc & 0x80) else (crc << 1)
    return crc & 0xFF

def unpack_channels(payload):
    """Unpack 16 channels from CRSF RC frame"""
    if len(payload) < 22:
        return [0] * 16
    
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

def to_percent(val):
    """Convert CRSF value to percentage"""
    return ((val - 172) / (1811 - 172)) * 100.0

def bar(val, width=25):
    """Create visual bar"""
    pct = to_percent(val)
    filled = int((pct / 100.0) * width)
    return ('█' * filled) + ('░' * (width - filled))

def switch_pos(val):
    """Get switch position"""
    if val < 500:
        return "LOW "
    elif val > 1400:
        return "HIGH"
    else:
        return "MID "

def display(ch, frames, rate, errors):
    """Display dashboard"""
    def p(text):
        print(f"{CLEAR_LINE}{text}")
    
    print(HOME, end='')
    
    p("╔════════════════════════════════════════════════════════════════════════════╗")
    p("║                       CRSF LIVE DASHBOARD                                  ║")
    p("╚════════════════════════════════════════════════════════════════════════════╝")
    p("")
    p(f"  Frames: {frames:<8}  Rate: {rate:6.1f} Hz  Errors: {errors}")
    p("")
    p("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
    p("")
    p(f"  Roll      {ch[0]:4d}  [{to_percent(ch[0]):5.1f}%]  {bar(ch[0])}")
    p(f"  Pitch     {ch[1]:4d}  [{to_percent(ch[1]):5.1f}%]  {bar(ch[1])}")
    p(f"  Throttle  {ch[2]:4d}  [{to_percent(ch[2]):5.1f}%]  {bar(ch[2])}")
    p(f"  Yaw       {ch[3]:4d}  [{to_percent(ch[3]):5.1f}%]  {bar(ch[3])}")
    p("")
    p("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
    p("")
    p(f"  AUX1: {ch[4]:4d} {switch_pos(ch[4])}    AUX2: {ch[5]:4d} {switch_pos(ch[5])}")
    p(f"  AUX3: {ch[6]:4d} {switch_pos(ch[6])}    AUX4: {ch[7]:4d} {switch_pos(ch[7])}")
    p("")
    p("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
    p("")
    p(f"  Ch 1-4:   {ch[0]:4d}  {ch[1]:4d}  {ch[2]:4d}  {ch[3]:4d}")
    p(f"  Ch 5-8:   {ch[4]:4d}  {ch[5]:4d}  {ch[6]:4d}  {ch[7]:4d}")
    p(f"  Ch 9-12:  {ch[8]:4d}  {ch[9]:4d}  {ch[10]:4d}  {ch[11]:4d}")
    p(f"  Ch13-16:  {ch[12]:4d}  {ch[13]:4d}  {ch[14]:4d}  {ch[15]:4d}")
    p("")
    p("────────────────────────────────────────────────────────────────────────────")
    
    sys.stdout.flush()

def main():
    if len(sys.argv) < 2:
        print("Usage: python crsf_live.py <PORT>")
        print("Example: python crsf_live.py COM3")
        sys.exit(1)
    
    port = sys.argv[1]
    
    try:
        ser = serial.Serial(port, 420000, timeout=1)
        print(f"{HIDE}{CLEAR}Connecting to {port}...", flush=True)
        time.sleep(1)
        
        channels = [992] * 16
        frame_count = 0
        error_count = 0
        last_time = time.time()
        rate = 0
        first = True
        last_display = 0
        
        while True:
            # Read sync byte
            sync = ser.read(1)
            if not sync or sync[0] not in [0x00, 0xC8, 0xEA, 0xEC]:
                continue
            
            # Read length
            length_byte = ser.read(1)
            if not length_byte:
                continue
            length = length_byte[0]
            
            if length < 2 or length > 64:
                continue
            
            # Read rest of frame
            rest = ser.read(length)
            if len(rest) != length:
                continue
            
            frame = bytes([sync[0], length]) + rest
            
            # Validate CRC
            calc_crc = crc8(frame[2:-1])
            if calc_crc != frame[-1]:
                error_count += 1
                continue
            
            # Check if RC channels frame
            if frame[2] == 0x16:  # RC channels
                payload = frame[3:-1]
                channels = unpack_channels(payload)
                frame_count += 1
                
                # Calculate rate
                now = time.time()
                if now - last_time > 0:
                    rate = 1.0 / (now - last_time)
                last_time = now
                
                # Update display (max 20 Hz)
                if first or (now - last_display) > 0.05:
                    if first:
                        print(CLEAR, end='')
                        first = False
                    display(channels, frame_count, rate, error_count)
                    last_display = now
    
    except KeyboardInterrupt:
        print(f"\n{SHOW}\nStopped.")
    except Exception as e:
        print(f"\n{SHOW}Error: {e}")
    finally:
        print(SHOW)

if __name__ == "__main__":
    main()

