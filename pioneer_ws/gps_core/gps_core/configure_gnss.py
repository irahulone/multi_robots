#!/usr/bin/env python3
"""
ZED-F9P one-shot configuration script.

- Navigation rate: 10 Hz (100 ms)
- Constellations: GPS + Galileo only (US use case)
- UART1:
    - UBX-NAV-PVT at 10 Hz
    - NMEA disabled (to reduce bandwidth)
- Save to non-volatile memory (flash/BBR)

Run this once (or whenever you want to reconfigure the receiver).
"""

import sys
import time
import serial
from pyubx2 import UBXReader, UBXMessage, SET


def wait_for_ack(ser, msg_class, msg_id, timeout=3.0):
    """Wait for ACK-ACK / ACK-NAK for given (class, id)."""
    ubr = UBXReader(ser)
    start = time.time()
    while time.time() - start < timeout:
        if ser.in_waiting:
            try:
                raw, parsed = ubr.read()
            except Exception:
                continue

            if parsed is None:
                continue

            if parsed.identity == "ACK-ACK":
                if parsed.clsID == msg_class and parsed.msgID == msg_id:
                    return True
            elif parsed.identity == "ACK-NAK":
                if parsed.clsID == msg_class and parsed.msgID == msg_id:
                    return False

        time.sleep(0.01)
    return None


def cfg_rate_10hz(ser):
    """Configure measurement rate to 10 Hz."""
    # 100 ms = 10 Hz
    msg = UBXMessage(
        "CFG",
        "CFG-RATE",
        SET,
        measRate=100,  # ms
        navRate=1,
        timeRef=0,     # GPS time
    )
    ser.reset_input_buffer()
    ser.write(msg.serialize())
    return wait_for_ack(ser, 0x06, 0x08)  # CFG-RATE


def cfg_gnss_gps_galileo_only(ser):
    """Enable only GPS + Galileo (dual band), disable others."""

    # flags helper
    def make_flags(enable, sig_mask):
        # bit 0 = enable, bits 23..16 = sigCfgMask
        return (1 if enable else 0) | ((sig_mask & 0xFF) << 16)

    # GPS: L1C/A + L2C (0x01 + 0x10 = 0x11)
    flags_gps = make_flags(True, 0x11)
    # Galileo: E1 + E5b (0x01 + 0x20 = 0x21)
    flags_gal = make_flags(True, 0x21)

    # Header
    # msgVer = 0
    # numTrkChHw is read-only on F9P, value is ignored in recent protocol versions
    # numTrkChUse: 0xFF => "use all HW channels"
    msg_ver = 0
    num_trk_ch_hw = 0
    num_trk_ch_use = 0xFF
    num_config_blocks = 6

    header = bytes([
        msg_ver,
        num_trk_ch_hw,
        num_trk_ch_use,
        num_config_blocks
    ])

    blocks = bytearray()

    def add_block(gnss_id, res_trk_ch, max_trk_ch, flags):
        # resTrkCh / maxTrkCh values are typical examples
        blocks.extend([
            gnss_id,
            res_trk_ch,
            max_trk_ch,
            0  # reserved0
        ])
        blocks.extend(flags.to_bytes(4, "little"))

    # 0: GPS (enable, dual band)
    add_block(0, 8, 16, flags_gps)
    # 1: SBAS (disabled)
    add_block(1, 1, 3, 0)
    # 2: Galileo (enable, dual band)
    add_block(2, 4, 8, flags_gal)
    # 3: BeiDou (disabled)
    add_block(3, 2, 16, 0)
    # 5: QZSS (disabled)
    add_block(5, 0, 3, 0)
    # 6: GLONASS (disabled)
    add_block(6, 8, 14, 0)

    payload = header + blocks

    # Build UBX frame
    length = len(payload).to_bytes(2, "little")
    msg_body = bytes([0x06, 0x3E]) + length + payload

    ck_a = 0
    ck_b = 0
    for b in msg_body:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF

    frame = bytes([0xB5, 0x62]) + msg_body + bytes([ck_a, ck_b])

    ser.reset_input_buffer()
    ser.write(frame)

    # Wait for ACK / NAK
    return wait_for_ack(ser, 0x06, 0x3E)


def cfg_msgs_nav_pvt_10hz_uart1(ser):
    """
    Configure message rates:

    - UBX-NAV-PVT on UART1: 10 Hz
    - Disable main NMEA on UART1 (GGA, GLL, GSA, GSV, RMC, VTG)
    """
    # Helper to send CFG-MSG
    def cfg_msg(msg_class, msg_id, rate_uart1):
        msg = UBXMessage(
            "CFG",
            "CFG-MSG",
            SET,
            msgClass=msg_class,
            msgID=msg_id,
            rateDDC=0,
            rateUART1=rate_uart1,
            rateUART2=0,
            rateUSB=0,
            rateSPI=0,
            rateReserved=0,
        )
        ser.reset_input_buffer()
        ser.write(msg.serialize())
        return wait_for_ack(ser, 0x06, 0x01)  # CFG-MSG

    # UBX-NAV-PVT (0x01 0x07) at 10 Hz
    res = cfg_msg(0x01, 0x07, 1)
    if res is False:
        return False

    # Disable NMEA on UART1 (optional, comment out if NMEA needed)
    nmea_msgs = [
        (0xF0, 0x00),  # GGA
        (0xF0, 0x01),  # GLL
        (0xF0, 0x02),  # GSA
        (0xF0, 0x03),  # GSV
        (0xF0, 0x04),  # RMC
        (0xF0, 0x05),  # VTG
        (0xF0, 0x08),  # ZDA
    ]

    for cls_id, msg_id in nmea_msgs:
        res = cfg_msg(cls_id, msg_id, 0)
        if res is False:
            return False

    return True


def save_to_flash(ser):
    """Save configuration to non-volatile memory (flash/BBR)."""
    msg = UBXMessage(
        "CFG",
        "CFG-CFG",
        SET,
        clearMask=b"\x00\x00\x00\x00",
        saveMask=b"\x1F\x1F\x00\x00",  # save most config blocks
        loadMask=b"\x00\x00\x00\x00",
    )
    ser.reset_input_buffer()
    ser.write(msg.serialize())
    return wait_for_ack(ser, 0x06, 0x09)  # CFG-CFG


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/gps"
    baudrate = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

    print("=" * 60)
    print("ZED-F9P one-shot configuration")
    print("  - 10 Hz navigation rate")
    print("  - GPS + Galileo only")
    print("  - UBX-NAV-PVT @ 10 Hz on UART1, NMEA off")
    print("=" * 60)
    print(f"Port    : {port}")
    print(f"Baudrate: {baudrate}")
    print("=" * 60)

    try:
        ser = serial.Serial(port, baudrate=baudrate, timeout=1)
        time.sleep(0.5)
        ser.reset_input_buffer()
    except Exception as e:
        print(f"ERROR: Failed to open serial port: {e}")
        sys.exit(1)

    # 1. Navigation rate
    print("\n[1/4] Setting navigation rate to 10 Hz (CFG-RATE)...")
    r = cfg_rate_10hz(ser)
    print(f"  -> ACK={r}")

    # 2. GNSS constellations
    print("\n[2/4] Setting GNSS to GPS + Galileo only (CFG-GNSS)...")
    r = cfg_gnss_gps_galileo_only(ser)
    print(f"  -> ACK={r}")

    # 3. Message configuration
    print("\n[3/4] Setting UBX-NAV-PVT @ 10 Hz on UART1 and disabling NMEA (CFG-MSG)...")
    r = cfg_msgs_nav_pvt_10hz_uart1(ser)
    print(f"  -> result={r}")

    # 4. Save configuration
    print("\n[4/4] Saving configuration to flash (CFG-CFG)...")
    r = save_to_flash(ser)
    print(f"  -> ACK={r}")

    ser.close()

    print("\n" + "=" * 60)
    print("DONE. Please power-cycle the ZED-F9P and confirm:")
    print("  - NAV-PVT is output at 10 Hz")
    print("  - Only GPS+Galileo are used (GNSS status / u-center etc.)")
    print("  - Settings persist after power cycle")
    print("=" * 60)


if __name__ == "__main__":
    main()
