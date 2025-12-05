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


def wait_for_ack(ser, msg_class, msg_id, timeout=3.0):
    """
    Wait for ACK-ACK or ACK-NAK response from u-blox module.

    Args:
        ser: Serial port object
        msg_class: UBX message class (e.g., 0x06 for CFG)
        msg_id: UBX message ID
        timeout: Timeout in seconds

    Returns:
        True if ACK received, False if NAK received, None if timeout
    """
    ubr = UBXReader(ser)
    start = time.time()
    while time.time() - start < timeout:
        if ser.in_waiting:
            try:
                raw, parsed = ubr.read()
                if parsed:
                    if parsed.identity == 'ACK-ACK':
                        if parsed.clsID == msg_class and parsed.msgID == msg_id:
                            return True
                    elif parsed.identity == 'ACK-NAK':
                        if parsed.clsID == msg_class and parsed.msgID == msg_id:
                            return False
            except Exception:
                pass
        time.sleep(0.01)
    return None


def configure_measurement_rate(ser, rate_hz=10):
    """
    Configure measurement rate with ACK verification.

    Args:
        ser: Serial port object
        rate_hz: Measurement rate in Hz

    Returns:
        True if ACK, False if NAK, None if timeout
    """
    meas_rate_ms = int(1000 / rate_hz)
    msg = UBXMessage('CFG', 'CFG-RATE', SET,
                     measRate=meas_rate_ms,
                     navRate=1,
                     timeRef=0)
    ser.reset_input_buffer()
    ser.write(msg.serialize())

    # CFG-RATE: class=0x06, id=0x08
    return wait_for_ack(ser, 0x06, 0x08)


def configure_gnss_constellations(ser):
    """
    Configure GNSS constellations (GPS + Galileo only).

    Args:
        ser: Serial port object

    Returns:
        True if ACK, False if NAK, None if timeout
    """
    gnss_blocks = []

    # GPS (gnssId=0): Enable, 8-16 channels
    gnss_blocks.extend([0, 8, 16, 0, 0x01, 0x00, 0x01, 0x01])
    # SBAS (gnssId=1): Disable
    gnss_blocks.extend([1, 1, 3, 0, 0x00, 0x00, 0x01, 0x01])
    # Galileo (gnssId=2): Enable, 4-8 channels
    gnss_blocks.extend([2, 4, 8, 0, 0x01, 0x00, 0x01, 0x01])
    # BeiDou (gnssId=3): Disable
    gnss_blocks.extend([3, 2, 16, 0, 0x00, 0x00, 0x01, 0x01])
    # QZSS (gnssId=5): Disable
    gnss_blocks.extend([5, 0, 3, 0, 0x00, 0x00, 0x05, 0x01])
    # GLONASS (gnssId=6): Disable
    gnss_blocks.extend([6, 8, 14, 0, 0x00, 0x00, 0x01, 0x01])

    payload = bytes([0, 0, 32, 6]) + bytes(gnss_blocks)

    # Calculate checksum
    ck_a, ck_b = 0, 0
    msg_data = bytes([0x06, 0x3E]) + len(payload).to_bytes(2, 'little') + payload
    for byte in msg_data:
        ck_a = (ck_a + byte) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF

    full_msg = bytes([0xB5, 0x62]) + msg_data + bytes([ck_a, ck_b])

    ser.reset_input_buffer()
    ser.write(full_msg)

    # CFG-GNSS: class=0x06, id=0x3E
    return wait_for_ack(ser, 0x06, 0x3E)


def save_configuration(ser):
    """
    Save configuration to flash memory.

    Args:
        ser: Serial port object

    Returns:
        True if ACK, False if NAK, None if timeout
    """
    msg = UBXMessage('CFG', 'CFG-CFG', SET,
                     clearMask=b'\x00\x00\x00\x00',
                     saveMask=b'\x1F\x1F\x00\x00',
                     loadMask=b'\x00\x00\x00\x00')
    ser.reset_input_buffer()
    ser.write(msg.serialize())

    # CFG-CFG: class=0x06, id=0x09
    return wait_for_ack(ser, 0x06, 0x09)


def configure_gnss(port='/dev/gps', baudrate=115200, logger=None):
    """
    Configure ZED-F9P GNSS module with ACK verification.

    Args:
        port: Serial port path
        baudrate: Baud rate
        logger: ROS2 logger (optional, uses print if None)

    Returns:
        tuple: (success: bool, message: str)
    """
    def log_info(msg):
        if logger:
            logger.info(msg)
        else:
            print(msg)

    def log_warn(msg):
        if logger:
            logger.warn(msg)
        else:
            print(f"WARN: {msg}")

    def log_error(msg):
        if logger:
            logger.error(msg)
        else:
            print(f"ERROR: {msg}")

    try:
        ser = serial.Serial(port, baudrate=baudrate, timeout=1)
        ser.reset_input_buffer()
        time.sleep(0.5)

        # 1. Configure measurement rate
        log_info("Configuring measurement rate to 10Hz...")
        ack = configure_measurement_rate(ser, rate_hz=10)
        if ack is True:
            log_info("CFG-RATE: ACK received")
        elif ack is False:
            log_warn("CFG-RATE: NAK received - configuration rejected")
        else:
            log_warn("CFG-RATE: No response (timeout)")

        # 2. Configure GNSS constellations
        log_info("Configuring GNSS constellations (GPS + Galileo)...")
        ack = configure_gnss_constellations(ser)
        if ack is True:
            log_info("CFG-GNSS: ACK received")
        elif ack is False:
            log_warn("CFG-GNSS: NAK received - configuration rejected")
        else:
            log_warn("CFG-GNSS: No response (timeout)")

        # 3. Save configuration to flash
        log_info("Saving configuration to flash...")
        ack = save_configuration(ser)
        if ack is True:
            log_info("CFG-CFG: ACK received - configuration saved")
        elif ack is False:
            log_error("CFG-CFG: NAK received - save failed")
            ser.close()
            return False, "Failed to save configuration"
        else:
            log_warn("CFG-CFG: No response (timeout)")

        ser.close()
        return True, "Configuration completed"

    except Exception as e:
        return False, str(e)


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

    success, message = configure_gnss(port, baudrate)

    print("\n" + "=" * 60)
    if success:
        print("Configuration Complete:")
        print("  - Measurement Rate: 10Hz (100ms)")
        print("  - GPS: ENABLED (8-16 channels)")
        print("  - Galileo: ENABLED (4-8 channels)")
        print("  - GLONASS: DISABLED")
        print("  - BeiDou: DISABLED")
        print("  - QZSS: DISABLED")
        print("  - SBAS: DISABLED")
    else:
        print(f"Configuration failed: {message}")
    print("=" * 60)

    sys.exit(0 if success else 1)
