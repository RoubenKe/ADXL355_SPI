#!/usr/bin/env python3
"""
Minimal ADXL355 sensor module using SPI.

This module:
- Opens the SPI device using spidev
- Reads 20-bit acceleration data (X, Y, Z)
- Converts to signed raw values, g and m/s^2

It does NOT depend on RPi.GPIO or board detection.
"""

from typing import Tuple
import spidev

# Gravity constant
G0 = 9.80665  # [m/s^2]


class ADXL355:
    """
    Minimal driver for the ADXL355 using SPI.

    Assumptions:
    - Sensor is connected to SPI bus 0, chip select 0 (CE0)
    - SPI mode 0
    - Default range after power-up is ±2 g
    """

    # Register addresses (7-bit, from datasheet)
    REG_DEVID_AD   = 0x00
    REG_DEVID_MST  = 0x01
    REG_PARTID     = 0x02
    REG_REVID      = 0x03

    REG_XDATA3     = 0x08  # start of X/Y/Z data block

    REG_RANGE      = 0x2C  # range register
    REG_POWER_CTL  = 0x2D  # power control register

    # Sensitivity for ±2 g range (typical, from datasheet)
    LSB_PER_G_2G   = 256000.0

    def __init__(self, bus: int = 0, device: int = 0, max_speed_hz: int = 1_000_000):
        """
        Initialize the SPI device and configure the sensor.
        """
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = max_speed_hz
        self.spi.mode = 0b00  # SPI mode 0

        # Track current sensitivity in LSB/g
        # Default power-up range is ±2 g, so start with that.
        self.lsb_per_g = self.LSB_PER_G_2G

        # Ensure we are in measurement mode:
        # Bit0 = 0 → measurement mode, Bit0 = 1 → standby.
        # Here we simply write 0x00 (measurement) and rely on default ODR/filter.
        self._write_reg(self.REG_POWER_CTL, 0x00)

    # ---------- low-level SPI helpers ----------

    def _read_reg(self, addr: int) -> int:
        """
        Read a single 8-bit register via SPI.
        """
        cmd = (addr << 1) | 0x01  # bit0=1 → read
        resp = self.spi.xfer2([cmd, 0x00])
        return resp[1]

    def _write_reg(self, addr: int, value: int) -> None:
        """
        Write a single 8-bit register via SPI.
        """
        cmd = (addr << 1) & 0xFE  # bit0=0 → write
        self.spi.xfer2([cmd, value & 0xFF])

    def _read_block(self, start_addr: int, length: int) -> bytes:
        """
        Read a block of 'length' bytes starting at 'start_addr'.
        """
        cmd = (start_addr << 1) | 0x01
        resp = self.spi.xfer2([cmd] + [0x00] * length)
        return bytes(resp[1:])  # drop first dummy byte

    @staticmethod
    def _convert_20bit(b3: int, b2: int, b1: int) -> int:
        """
        Convert three bytes (20-bit left-justified two's complement)
        to a signed integer.

        Layout (from datasheet):
        - b3: bits [19:12]
        - b2: bits [11:4]
        - b1: bits [7:4] = bits [3:0] of the 20-bit value
        - b1: bits [3:0] reserved
        """
        raw = ((b3 << 12) | (b2 << 4) | (b1 >> 4)) & 0xFFFFF  # keep 20 bits
        # Two's complement sign extension for 20 bits
        if raw & 0x80000:  # if bit 19 is set → negative
            raw -= 1 << 20
        return raw

    # ---------- public API ----------

    def set_range(self, range_g: int) -> None:
        """
        Set measurement range to ±2 g, ±4 g or ±8 g.

        range_g: 2, 4 or 8
        """
        if range_g == 2:
            reg_val = 0x01
            self.lsb_per_g = 256000.0
        elif range_g == 4:
            reg_val = 0x02
            self.lsb_per_g = 128000.0
        elif range_g == 8:
            reg_val = 0x03
            self.lsb_per_g = 64000.0
        else:
            raise ValueError("range_g must be 2, 4 or 8.")

        # Write to RANGE register
        self._write_reg(self.REG_RANGE, reg_val)

    def read_raw(self) -> Tuple[int, int, int]:
        """
        Read raw 20-bit acceleration data for X, Y, Z.

        Returns:
            (x_raw, y_raw, z_raw) as signed integers.
        """
        block = self._read_block(self.REG_XDATA3, 9)  # 3 bytes per axis
        x_raw = self._convert_20bit(block[0], block[1], block[2])
        y_raw = self._convert_20bit(block[3], block[4], block[5])
        z_raw = self._convert_20bit(block[6], block[7], block[8])
        return x_raw, y_raw, z_raw

    def read_g(self) -> Tuple[float, float, float]:
        """
        Read acceleration in units of g (using current range / sensitivity).

        Returns:
            (x_g, y_g, z_g) as floats.
        """
        x_raw, y_raw, z_raw = self.read_raw()
        scale = 1.0 / self.lsb_per_g  # g per LSB
        return x_raw * scale, y_raw * scale, z_raw * scale

    def read_ms2(self) -> Tuple[float, float, float]:
        """
        Read acceleration in units of m/s^2 (using current range / sensitivity).

        Returns:
            (x_ms2, y_ms2, z_ms2) as floats.
        """
        x_g, y_g, z_g = self.read_g()
        return x_g * G0, y_g * G0, z_g * G0

    def read_ids(self) -> Tuple[int, int, int, int]:
        """
        Read device identification registers.

        Returns:
            (DEVID_AD, DEVID_MST, PARTID, REVID)
        """
        devid_ad  = self._read_reg(self.REG_DEVID_AD)
        devid_mst = self._read_reg(self.REG_DEVID_MST)
        partid    = self._read_reg(self.REG_PARTID)
        revid     = self._read_reg(self.REG_REVID)
        return devid_ad, devid_mst, partid, revid

    def close(self) -> None:
        """
        Close the SPI device.
        """
        self.spi.close()
