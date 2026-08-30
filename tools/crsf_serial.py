#!/usr/bin/env python3
"""
CRSF Serial Monitor - Simple real-time RC channel monitor
Usage: python crsf_serial.py COM3
"""

import serial
import sys
import time
from protocols.crsf_decoder import CRSFDecoder

# ANSI codes for terminal UI
CLEAR = '\033[2J\033[H'
HOME = '\033[H'
HIDE = '\033[?25l'
SHOW = '\033[?25h'
CLEAR_LINE = '\033[2K'

def to_percent(val, min_val=172, max_val=1811):
    """Convert channel value to percentage"""
    return ((val - min_val) / (max_val - min_val)) * 100.0

def bar(val, width=25, min_val=172, max_val=1811):
    """Create visual bar"""
    pct = to_percent(val, min_val, max_val)
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
    p("║                       CRSF SERIAL MONITOR                                  ║")
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
        print("Usage: python crsf_serial.py <PORT>")
        print("Example: python crsf_serial.py COM3")
        sys.exit(1)
    
    port = sys.argv[1]
    decoder = CRSFDecoder()
    
    try:
        ser = serial.Serial(port, decoder.baudrate, timeout=1)
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
            result = decoder.decode(ser)
            
            if result:
                channels = result
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
            else:
                error_count += 1
    
    except KeyboardInterrupt:
        print(f"\n{SHOW}\nStopped.")
    except Exception as e:
        print(f"\n{SHOW}Error: {e}")
    finally:
        print(SHOW)

if __name__ == "__main__":
    main()

