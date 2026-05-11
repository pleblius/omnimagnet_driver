# Omnimagnet Driver

A ROS 2 hardware driver for controlling multiple electromagnetic coil systems ("Omnimagnets") through an Advantech PCI-1724 DAQ using `comedilib`.

This package provides:

- Real-time current control of multiple omnimagnets
- Constant dipole generation
- Rotating dipole generation
- Multi-magnet synchronized control
- Safety shutdown handling
- Timeout protection
- Error/status publishing

---

## Features

The driver supports:

### Single Magnet Operations
- Constant dipole field generation
- Rotating dipole field generation

### Multi Magnet Operations
- Multiple synchronized constant dipoles
- Multiple synchronized rotating dipoles
- Shared or independent parameters

### Safety Features
- Automatic hardware shutdown on node exit
- Command timeout shutdown
- Magnet current zeroing after experiment completion
- Error publishing for hardware faults

---

## Hardware Requirements

This driver currently assumes:

### DAQ Board
- **Advantech PCI-1724**
- Accessed through `comedilib`

### Device Node
Expected DAQ device:

```bash
/dev/comedi0
```

### Amplifier Inhibit Pins

The driver automatically configures:

| Channel | Function |
|---------|----------|
| 25 | Amplifier inhibit |
| 26 | Amplifier inhibit |

These are initialized to **75% output** during startup.

---

## Dependencies

### ROS Dependencies

- ROS 2 (tested with Humble)
- `rclcpp`

### System Dependencies

- `comedilib`
- Eigen3
- pthread

Install comedi:

```bash
sudo apt install libcomedi-dev
```

---

## Package Structure

The main node is:

```cpp
OmnimagnetDriverNode
```

Node name:

```bash
omnimagnet_driver
```

---

## Running

Launch the driver:

```bash
ros2 run omnimagnet_driver omnimagnet_driver
```

---

# Published Topics

## Driver Errors

Topic:

```bash
/driver_errors
```

Type:

`omnimagnet_interfaces/msg/ErrorMessage`

Used for:

- Hardware initialization failures
- Magnet shutdown failures
- Timeout shutdowns
- Runtime driver faults

---

## Driver Finished

Topic:

```bash
/driver_finished
```

Type:

`omnimagnet_interfaces/msg/FinishedMessage`

Published when an experiment completes successfully.

Contains:

- Completion timestamp

---

# Services

---

# 1. Single Magnet Constant

Service:

```bash
/single_magnet_constant
```

Type:

`SingleMagnetConstant`

## Request

| Field | Type | Description |
|------|------|-------------|
| omnimagnet | uint64 | Magnet ID |
| dipole_vec | Vector3 | Desired dipole direction |
| dipole_strength | float64 | Field strength |
| duration | float64 | Optional runtime (seconds) |

## Notes

If duration ≤ 0:

Default:

```cpp
30 seconds
```

### Example

```bash
ros2 service call /single_magnet_constant omnimagnet_interfaces/srv/SingleMagnetConstant "
{
  omnimagnet: 0,
  dipole_vec: {x: 1.0, y: 0.0, z: 0.0},
  dipole_strength: 40.0,
  duration: 10.0
}"
```

---

# 2. Single Magnet Rotation

Service:

```bash
/single_magnet_rotation
```

Type:

`SingleMagnetRotation`

## Request

| Field | Type |
|------|------|
| omnimagnet | uint64 |
| rotation_vector | Vector3 |
| dipole_strength | float64 |
| rotation_freq | float64 |
| phase_offset | float64 |
| duration | float64 |

### Notes

Phase offset is specified in:

```text
degrees
```

Internally converted to radians.

### Example

```bash
ros2 service call /single_magnet_rotation omnimagnet_interfaces/srv/SingleMagnetRotation "
{
  omnimagnet: 0,
  rotation_vector: {x: 0.0, y: 0.0, z: 1.0},
  dipole_strength: 40.0,
  rotation_freq: 5.0,
  phase_offset: 90.0,
  duration: 20.0
}"
```

---

# 3. Multi Magnet Constant

Service:

```bash
/multi_magnet_constant
```

Type:

`MultiMagnetConstant`

## Request

Supports:

- One vector for all magnets
- One strength for all magnets
- Per-magnet vectors/strengths

### Example

```bash
ros2 service call /multi_magnet_constant omnimagnet_interfaces/srv/MultiMagnetConstant "
{
  omnimagnets: [0,1,2],
  dipole_vecs: [
    {x: 1.0, y: 0.0, z: 0.0}
  ],
  dipole_strengths: [40.0],
  duration: 10.0
}"
```

---

# 4. Multi Magnet Rotation

Service:

```bash
/multi_magnet_rotation
```

Type:

`MultiMagnetRotation`

Supports:

- Shared rotation vector
- Shared frequency
- Shared phase offset
- Per-magnet overrides

### Example

```bash
ros2 service call /multi_magnet_rotation omnimagnet_interfaces/srv/MultiMagnetRotation "
{
  omnimagnets: [0,1],
  rotation_vectors: [
    {x: 0.0, y: 0.0, z: 1.0}
  ],
  dipole_strengths: [40.0],
  rotation_freqs: [5.0],
  phase_offsets: [0.0,180.0],
  duration: 20.0
}"
```

---

# 5. Driver Reset

Service:

```bash
/reset_driver
```

Type:

`DriverReset`

Immediately:

- Stops active experiment
- Turns off all magnets
- Cancels duration timer
- Restarts timeout timer

### Example

```bash
ros2 service call /reset_driver omnimagnet_interfaces/srv/DriverReset "{}"
```

---

# Magnet IDs

Currently configured:

| ID | Description |
|----|-------------|
| 0 | Left Upper |
| 1 | Center Upper |
| 2 | Right Upper |
| 3 | Right Lower |
| 4 | Left Lower |

ID 5 is reserved but disabled.

---

# Real-Time Control

A dedicated control thread runs at:

```text
1200 Hz
```

The thread:

1. Copies active commands under mutex protection
2. Computes target dipole
3. Converts dipole to coil currents
4. Writes currents to hardware

This keeps ROS callbacks separate from hardware timing.

---

# Safety Behavior

## Node Exit

On shutdown:

- Control thread stops
- All magnets set to zero current
- Amplifier pins reset

---

## Timeout

If no controller command is received for:

```text
30 seconds
```

The driver:

1. Publishes error
2. Safely shuts down

---

# Validation Checks

Requests are rejected if:

- Magnet ID is invalid
- Vector magnitude is zero
- Vector contains NaN
- Parameter array sizes mismatch
- Another experiment is already running

---

# Important Notes

## Hardware Access

User must have access to:

```bash
/dev/comedi0
```

If permission errors occur:

```bash
sudo usermod -aG iocard $USER
```

Then log out and back in.

---

## Current Configuration

Magnet calibration values are currently hardcoded.

Future improvement:

Move calibration to YAML config.

---

## Thread Safety

Active commands are protected with:

- `std::mutex`
- `std::atomic`

Do not bypass these protections when modifying control logic.

---

# TODOs

Current code still has several planned improvements:

- [ ] Move magnet config to YAML
- [ ] Parameterize timeout values
- [ ] Parameterize control loop frequency
- [ ] Add launch file
- [ ] Add diagnostics topic
- [ ] Add hardware self-test service

---

# Authors

Tyler Wilcox
University of Utah
tyler.c.wilcox@utah.edu