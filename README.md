# ADXL355_SPI

A lightweight Python setup for using an **ADXL355 accelerometer** on a Raspberry Pi 3 B+,  
streaming acceleration data via **UDP** to a Linux host.

## Overview

This repository contains a minimal driver and UDP daemon for the ADXL355 connected via SPI.
It allows real-time data acquisition and remote configuration (range, sampling rate, streaming state)
from any networked PC.

**Main components:**

- `adxl355_sensor.py` — minimal SPI driver for the ADXL355  
- `read_adxl355.py` — simple console reader for debugging  
- `udp_sender.py` — background service sending acceleration data via UDP and listening for control commands

## Features

- SPI communication without external dependencies (just `spidev`)
- Adjustable measurement range (±2 g / ±4 g / ±8 g)
- Configurable sampling rate (up to ~1 kHz)
- UDP streaming of raw acceleration data (`x_raw`, `y_raw`, `z_raw`)
- Remote control via UDP (`set_range`, `set_rate`, `set_streaming`, `get_status`)

## Installation

Clone the repository and install dependencies:

```bash
sudo apt install python3-pip python3-spidev -y
git clone https://github.com/RoubenKe/ADXL355_SPI.git
cd ADXL355_SPI
pip install -r requirements.txt
