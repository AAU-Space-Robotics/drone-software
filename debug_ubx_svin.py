#!/usr/bin/env python3
"""
Debug script to decode UBX-NAV-SVIN messages and find correct byte offsets.
Run this while the RTK base station is surveying to see raw message data.
"""

import serial
import struct
import sys

def find_ubx_svin(data):
    """Find and parse UBX-NAV-SVIN (0xB5 0x62 0x01 0x3B) messages"""
    i = 0
    while i < len(data) - 50:
        if data[i:i+4] == b'\xB5\x62\x01\x3B':
            # Found NAV-SVIN message
            length = struct.unpack('<H', data[i+4:i+6])[0]
            if i + 6 + length + 2 <= len(data):
                payload = data[i+6:i+6+length]
                print(f"\n{'='*80}")
                print(f"UBX-NAV-SVIN found at offset {i}, payload length: {length} bytes")
                print(f"{'='*80}")
                
                # Print hex dump of payload
                print("\nPayload hex dump (offset: bytes):")
                for offset in range(0, len(payload), 16):
                    hex_str = ' '.join(f'{b:02X}' for b in payload[offset:offset+16])
                    ascii_str = ''.join(chr(b) if 32 <= b < 127 else '.' for b in payload[offset:offset+16])
                    print(f"  {offset:3d}: {hex_str:<48s} | {ascii_str}")
                
                # Try to parse known fields
                if length >= 40:
                    print("\n" + "="*80)
                    print("Attempting to parse fields at different offsets:")
                    print("="*80)
                    
                    # Try version at offset 0
                    version = payload[0]
                    print(f"\nOffset 0 (version): {version}")
                    
                    # Try different offsets for iTOW (should be reasonable GPS time)
                    for offset in [0, 4]:
                        if offset + 4 <= len(payload):
                            itow = struct.unpack('<I', payload[offset:offset+4])[0]
                            print(f"Offset {offset:2d} as iTOW: {itow} ({itow/1000:.1f}s)")
                    
                    # Try different offsets for duration (should be < 1000s typically)
                    for offset in [4, 8]:
                        if offset + 4 <= len(payload):
                            dur = struct.unpack('<I', payload[offset:offset+4])[0]
                            print(f"Offset {offset:2d} as duration: {dur}s")
                    
                    # Try different offsets for ECEF coordinates (should be large numbers)
                    for offset in [8, 12]:
                        if offset + 12 <= len(payload):
                            x = struct.unpack('<i', payload[offset:offset+4])[0]
                            y = struct.unpack('<i', payload[offset+4:offset+8])[0]
                            z = struct.unpack('<i', payload[offset+8:offset+12])[0]
                            print(f"Offset {offset:2d} as ECEF X,Y,Z: {x}, {y}, {z} (cm)")
                    
                    # Try different offsets for accuracy (should be reasonable mm value)
                    for offset in [24, 28, 32]:
                        if offset + 4 <= len(payload):
                            acc = struct.unpack('<I', payload[offset:offset+4])[0]
                            print(f"Offset {offset:2d} as accuracy: {acc} (0.1mm = {acc/10.0}mm)")
                    
                    # Try different offsets for observations (should be < 10000 typically)
                    for offset in [28, 32, 36]:
                        if offset + 4 <= len(payload):
                            obs = struct.unpack('<I', payload[offset:offset+4])[0]
                            print(f"Offset {offset:2d} as observations: {obs}")
                    
                    # Try different offsets for flags
                    for offset in [36, 37, 38, 39, 40]:
                        if offset < len(payload):
                            flags = payload[offset]
                            valid = (flags & 0x01) != 0
                            active = (flags & 0x02) != 0
                            print(f"Offset {offset:2d} as flags: 0x{flags:02X} (valid={valid}, active={active})")
                
                return True
        i += 1
    return False

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
    
    print(f"Opening {port} at 115200 baud...")
    print("Waiting for UBX-NAV-SVIN messages...")
    print("Press Ctrl+C to exit\n")
    
    with serial.Serial(port, 115200, timeout=1) as ser:
        buffer = b''
        while True:
            try:
                chunk = ser.read(1024)
                if chunk:
                    buffer += chunk
                    if find_ubx_svin(buffer):
                        # Keep last 1KB in buffer
                        buffer = buffer[-1024:]
                    elif len(buffer) > 4096:
                        buffer = buffer[-1024:]
            except KeyboardInterrupt:
                print("\n\nExiting...")
                break

if __name__ == '__main__':
    main()
