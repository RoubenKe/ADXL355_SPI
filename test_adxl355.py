#!/usr/bin/env python3
"""
Simple ADXL355 reader using the adxl355_sensor module.

- Initializes the sensor
- Sets range to ±4 g by default
- Prints raw values, values in g and m/s^2 in a loop
"""

import time
from adxl355_sensor import ADXL355


def main():
    # Initialize sensor (SPI0, CE0)
    sensor = ADXL355(bus=0, device=0, max_speed_hz=1_000_000)

    # Set measurement range to ±4 g (default for this script)
    sensor.set_range(4)

    try:
        # Read and print device IDs
        devid_ad, devid_mst, partid, revid = sensor.read_ids()
        print(f"DEVID_AD  = 0x{devid_ad:02X}")
        print(f"DEVID_MST = 0x{devid_mst:02X}")
        print(f"PARTID    = 0x{partid:02X}")
        print(f"REVID     = 0x{revid:02X}\n")

        print("Reading X/Y/Z (raw, g, m/s^2) – Ctrl+C to stop\n")

        while True:
            x_raw, y_raw, z_raw = sensor.read_raw()
            x_g, y_g, z_g = sensor.read_g()
            x_ms2, y_ms2, z_ms2 = sensor.read_ms2()

            print(
                f"RAW   x={x_raw:8d}  y={y_raw:8d}  z={z_raw:8d}  |  "
                f"g   x={x_g:+1.4f}  y={y_g:+1.4f}  z={z_g:+1.4f}  |  "
                f"m/s^2 x={x_ms2:+6.3f}  y={y_ms2:+6.3f}  z={z_ms2:+6.3f}"
            )

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user.")
    finally:
        sensor.close()


if __name__ == "__main__":
    main()
