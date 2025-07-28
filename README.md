# QuSpin QTFM Gen-2 and GPS Simulator

A hardware emulator for QuSpin QTFM Gen-2 magnetometers and GPS receivers, designed for testing and development on Raspberry Pi platforms without physical hardware.

## Overview

This simulator creates virtual serial ports that emulate:
- **GPS Receiver** on `/dev/ttyAMA0` (NMEA 0183 protocol)
- **QuSpin Magnetometer 1** on `/dev/ttyAMA2` (QuSpin proprietary protocol)
- **QuSpin Magnetometer 2** on `/dev/ttyAMA4` (QuSpin proprietary protocol)

The simulator accurately replicates the data formats, timing, and protocols of real hardware devices.

## Features

- **Accurate Protocol Implementation**: Exact replication of QuSpin and NMEA data formats
- **Y-Splitter Mode**: Option to make both magnetometers output identical data
- **Real-time Simulation**: Proper timing for data rates (GPS at 10Hz, Magnetometers at 250Hz)
- **Safe Port Management**: Automatic backup and restoration of existing hardware devices
- **Interactive Console**: Runtime control of simulation parameters

## Requirements

- Raspberry Pi (tested on Pi 5) or Linux system
- C++11 compiler (g++)
- Root privileges (for creating devices in `/dev/`)
- PTY support (pseudo-terminal)

## Installation

### 1. Install Dependencies

```bash
# Update system
sudo apt update
sudo apt upgrade -y

# Install C++ development tools
sudo apt install build-essential -y

# Install screen for testing (optional)
sudo apt install screen -y
```

### 2. Compile the Simulator

```bash
# Clone or download the source code
# Navigate to the project directory

# Compile with PTY support
g++ -std=c++11 -pthread quspin_gps_simulator.cpp -o quspin_simulator -lutil

# If -lutil fails, try without it
g++ -std=c++11 -pthread quspin_gps_simulator.cpp -o quspin_simulator
```

## Usage

### Running the Simulator

```bash
# Must run with root privileges
sudo ./quspin_simulator
```

### Interactive Commands

- `i` - Toggle identical magnetometers mode (Y-splitter)
- `m` - Show menu
- `q` - Quit simulator

### Testing the Output

Open separate terminals to view the data streams:

```bash
# Terminal 1 - GPS Data
screen /dev/ttyAMA0 9600

# Terminal 2 - Magnetometer 1
screen /dev/ttyAMA2 115200

# Terminal 3 - Magnetometer 2
screen /dev/ttyAMA4 115200

# Exit screen with Ctrl+A, then K
```

## Protocol Specifications

### QuSpin QTFM Gen-2 Protocol

**Serial Configuration:**
- Baud rate: 115200
- Data bits: 8
- Parity: None
- Stop bits: 1
- Flow control: None

**Data Format:**

Each data line follows this structure:
```
!{scalar}{validation}{axis}{vector}{validation}@{counter}>{timestamp}s{s_sens}v{v_sens}
```

**Example:**
```
!52926.706_Y53998.412=@394>86341532s139v107
```

**Field Descriptions:**

1. **Scalar Field** (`!52926.706_`)
    - `!` - Start delimiter
    - `52926.706` - Magnetic field magnitude in nanoTesla (3 decimal places)
    - `_` - Validation character (`_` = valid, `*` = invalid)

2. **Vector Field** (`Y53998.412=`)
    - `Y` - Axis identifier (X, Y, or Z)
    - `53998.412` - Vector component in nanoTesla (3 decimal places, can be negative)
    - `=` - Validation character (`=` = valid, `?` = invalid)

3. **Data Counter** (`@394`)
    - `@` - Counter delimiter
    - `394` - 3-digit counter (000-498, increments by 2)
    - Rolls over from 498 to 000

4. **Timestamp** (`>86341532`)
    - `>` - Timestamp delimiter
    - `86341532` - Milliseconds since power-on (increments by 4)
    - 32-bit value (rolls over ~every 50 days)

5. **Sensitivities** (`s139v107`)
    - `s139` - Scalar sensitivity (000-999, higher is better, 50+ optimal)
    - `v107` - Vector sensitivity (000-999, higher is better, 10+ optimal)

**Data Rate:** ~250Hz (4ms between samples)

**Axis Rotation:** The vector component rotates through X→Y→Z→X with each sample

### GPS NMEA 0183 Protocol

**Serial Configuration:**
- Baud rate: 9600
- Data bits: 8
- Parity: None
- Stop bits: 1
- Flow control: None

**Primary Sentence: GNGGA**

Format:
```
$GNGGA,HHMMSS.SS,DDMM.MMMMM,N,DDDMM.MMMMM,W,Q,NN,H.HH,AAA.A,M,GG.G,M,,*CC
```

**Example:**
```
$GNGGA,165732.50,4350.00141,N,07918.61979,W,1,09,0.57,208.7,M,-36.0,M,,*7E
```

**Field Descriptions:**
- `$GNGGA` - Sentence identifier (GN = GPS+GLONASS)
- `165732.50` - UTC time (16:57:32.50)
- `4350.00141,N` - Latitude (43° 50.00141' North)
- `07918.61979,W` - Longitude (079° 18.61979' West)
- `1` - Fix quality (0=invalid, 1=GPS fix, 2=DGPS)
- `09` - Number of satellites
- `0.57` - Horizontal dilution of precision
- `208.7,M` - Altitude in meters
- `-36.0,M` - Geoid separation
- `*7E` - Checksum

**Secondary Sentence: GNZDA**

Occasionally sent (~every 50 GNGGA sentences):
```
$GNZDA,165737.50,22,07,2025,00,00*7E
```

Fields: UTC time, day, month, year, timezone offset

**Data Rate:** 10Hz (100ms between samples)

## Technical Implementation

### Virtual Port Creation

The simulator uses PTY (pseudo-terminal) to create virtual serial ports:
1. Opens a master/slave PTY pair
2. Creates symbolic links in `/dev/` pointing to the slave PTY
3. Configures appropriate baud rates and permissions

### Thread Architecture

- **Main Thread**: User interface and control
- **GPS Thread**: Generates NMEA sentences at 10Hz
- **Magnetometer Thread 1**: QuSpin data for `/dev/ttyAMA2`
- **Magnetometer Thread 2**: QuSpin data for `/dev/ttyAMA4`

### Y-Splitter Mode

When enabled, both magnetometers output identical data:
- Magnetometer 1 generates the data
- Magnetometer 2 copies all values including timestamps
- Simulates a hardware Y-splitter configuration

## Safety Features

1. **Automatic Backup**: Real hardware devices are renamed to `.backup`
2. **Cleanup on Exit**: All virtual ports are removed
3. **Restoration**: Original devices are restored from backups
4. **Signal Handling**: Proper cleanup on Ctrl+C

## Troubleshooting

### Permission Denied
```bash
# Must run with sudo
sudo ./quspin_simulator
```

### Ports Already Exist
```bash
# Manual cleanup if needed
sudo rm -f /dev/ttyAMA0 /dev/ttyAMA2 /dev/ttyAMA4
```

### Compilation Errors
```bash
# Try without -lutil flag
g++ -std=c++11 -pthread quspin_gps_simulator.cpp -o quspin_simulator
```

## Data Format Precision

- **GPS Coordinates**: 5 decimal places (sub-centimeter precision)
- **GPS Altitude**: 1 decimal place (decimeter precision)
- **Magnetometer Fields**: 3 decimal places (sub-nanoTesla precision)
- **All timing**: Millisecond precision

## License

This simulator is provided as-is for testing and development purposes.

## Author

Developed for Ramon Freire