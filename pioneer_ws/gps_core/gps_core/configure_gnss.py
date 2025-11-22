#!/usr/bin/env python3
"""
GNSS Configuration Utility for ZED-F9P
Configures GPS module to use:
- 10Hz measurement rate
- GPS + Galileo constellations only
Run this script once to configure the module, settings will be saved to flash
"""

from pyubx2 import UBXReader, UBXMessage, SET
import serial
import time
import sys

def configure_gnss(port='/dev/gps', baudrate=115200):
    """Configure GNSS constellations: GPS + Galileo only"""
    
    print(f"Connecting to {port} at {baudrate} baud...")
    
    try:
        ser = serial.Serial(port, baudrate=baudrate, timeout=1)
        print("Connected successfully")
        
        # Clear input buffer
        ser.reset_input_buffer()
        time.sleep(0.5)
        
        print("\nConfiguring measurement rate to 10Hz...")
        msg_rate = UBXMessage('CFG', 'CFG-RATE', SET,
                            measRate=100,  # 100ms = 10Hz
                            navRate=1,
                            timeRef=0)
        ser.write(msg_rate.serialize())
        time.sleep(0.3)
        print("✓ Measurement rate set to 10Hz (100ms)")
        
        print("\nConfiguring GNSS constellations...")
        print("This will enable GPS and Galileo, disable all others")
        
        # CFG-GNSS: Configure all GNSS systems at once
        # For ZED-F9P, we need to configure the complete GNSS setup
        
        # Build GNSS configuration blocks
        # Format: gnssId, resTrkCh, maxTrkCh, reserved1, flags (4 bytes)
        
        gnss_blocks = []
        
        # GPS (gnssId=0): Enable, 8-16 channels
        gnss_blocks.extend([
            0,      # gnssId: GPS
            8,      # resTrkCh: reserved tracking channels
            16,     # maxTrkCh: maximum tracking channels
            0,      # reserved
            0x01, 0x00, 0x01, 0x01  # flags: enable=1, sigCfgMask for L1C/A
        ])
        
        # SBAS (gnssId=1): Disable
        gnss_blocks.extend([
            1,      # gnssId: SBAS
            1,      # resTrkCh
            3,      # maxTrkCh
            0,      # reserved
            0x00, 0x00, 0x01, 0x01  # flags: enable=0
        ])
        
        # Galileo (gnssId=2): Enable, 4-8 channels
        gnss_blocks.extend([
            2,      # gnssId: Galileo
            4,      # resTrkCh
            8,      # maxTrkCh
            0,      # reserved
            0x01, 0x00, 0x01, 0x01  # flags: enable=1, sigCfgMask for E1
        ])
        
        # BeiDou (gnssId=3): Disable
        gnss_blocks.extend([
            3,      # gnssId: BeiDou
            2,      # resTrkCh
            16,     # maxTrkCh
            0,      # reserved
            0x00, 0x00, 0x01, 0x01  # flags: enable=0
        ])
        
        # QZSS (gnssId=5): Disable
        gnss_blocks.extend([
            5,      # gnssId: QZSS
            0,      # resTrkCh
            3,      # maxTrkCh
            0,      # reserved
            0x00, 0x00, 0x05, 0x01  # flags: enable=0
        ])
        
        # GLONASS (gnssId=6): Disable
        gnss_blocks.extend([
            6,      # gnssId: GLONASS
            8,      # resTrkCh
            14,     # maxTrkCh
            0,      # reserved
            0x00, 0x00, 0x01, 0x01  # flags: enable=0
        ])
        
        # Create CFG-GNSS message
        # Note: pyubx2 expects specific parameter names for CFG-GNSS
        # We'll send raw bytes instead
        
        # UBX header: 0xB5 0x62
        # Class: 0x06 (CFG)
        # ID: 0x3E (GNSS)
        # Length: variable
        
        payload = bytes([
            0,      # msgVer
            0,      # numTrkChHw (0 = read-only)
            32,     # numTrkChUse (max tracking channels to use)
            6,      # numConfigBlocks
        ]) + bytes(gnss_blocks)
        
        # Calculate checksum
        ck_a = 0
        ck_b = 0
        msg_data = bytes([0x06, 0x3E]) + len(payload).to_bytes(2, 'little') + payload
        
        for byte in msg_data:
            ck_a = (ck_a + byte) & 0xFF
            ck_b = (ck_b + ck_a) & 0xFF
        
        full_msg = bytes([0xB5, 0x62]) + msg_data + bytes([ck_a, ck_b])
        
        print("Sending GNSS configuration...")
        ser.write(full_msg)
        time.sleep(1.0)
        
        print("GNSS configuration sent successfully")
        
        # Save configuration to flash
        print("\nSaving configuration to flash memory...")
        msg_save = UBXMessage('CFG', 'CFG-CFG', SET,
                            clearMask=b'\x00\x00\x00\x00',
                            saveMask=b'\x1F\x1F\x00\x00',
                            loadMask=b'\x00\x00\x00\x00')
        ser.write(msg_save.serialize())
        time.sleep(1.0)
        
        print("Configuration saved successfully!")
        print("\nGNSS Configuration Complete:")
        print("  ✓ Measurement Rate: 10Hz (100ms)")
        print("  ✓ GPS: ENABLED (8-16 channels)")
        print("  ✓ Galileo: ENABLED (4-8 channels)")
        print("  ✗ GLONASS: DISABLED")
        print("  ✗ BeiDou: DISABLED")
        print("  ✗ QZSS: DISABLED")
        print("  ✗ SBAS: DISABLED")
        print("\nSettings have been saved to flash memory.")
        print("You can now use the main GPS node.")
        
        ser.close()
        return True
        
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == '__main__':
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/gps'
    baudrate = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    
    print("=" * 60)
    print("ZED-F9P GNSS Configuration Utility")
    print("Configures: 10Hz rate + GPS + Galileo")
    print("=" * 60)
    print(f"Port: {port}")
    print(f"Baudrate: {baudrate}")
    print("=" * 60)
    
    success = configure_gnss(port, baudrate)
    
    sys.exit(0 if success else 1)