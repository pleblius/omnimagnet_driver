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

## Setup

This code is built to run using:

### OS
- Ubuntu Jammy 22.04

### DAQ Board
- **Advantech PCI-1724**
- Accessed using `comedilib`

### Device Access
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

### Operating Pins

| Omnimagnet | Inner Pin | Middle Pin | Outer Pin |
|------------|-----------|------------|-----------|
|     1      |     2     |      0     |     18    |
|     2      |     3     |     11     |     19    |
|     3      |     4     |     12     |     20    |
|     4      |     5     |     13     |     21    |
|     5      |     6     |     14     |     22    |

---

## Dependencies

### ROS Dependencies

- ROS 2 (tested with Humble)
- `rclcpp`

### System Dependencies

- `comedilib`
- Eigen3
- pthread

### Code Dependencies
- omnimagnet
- omnimagnet_interfaces

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

Contained in:
```
omnimagnet_driver/src/omnimagnet_driver.cpp
```

It relies on code contained in:
```
omnimagnet_driver/include/omnimagnet_driver/omnimagnet_driver.hpp
omnimagnet_driver/include/omnimagnet_driver/omnimagnet.hpp
omnimagnet_driver/src/omnimagnet.cpp
```

---

## Running

Launch the driver:

```bash
ros2 run omnimagnet_driver omnimagnet_driver
```

### Domain ID

Defaults to domain ID 
`2`

Will timeout after 60 seconds of not receiving any topics.

---

# Vector3
A copy of ROS2's Vector3 interface type to simplify dependencies

| Field | Type |
|------|------|
| x | float64 |
| y | float64 |
| z | float64 |

# Published Topics

## Driver Errors

Topic:

```bash
/driver_errors
```


Type:

`omnimagnet_interfaces/msg/ErrorMessage`

| Field | Type | Description |
|------|------|-------------|
| error_desc | string | Brief description of cause of error |
| shutdown | bool | True if the driver initiated a shutdown |

Reports errors that prevent proper driver operation and whether a shutdown has been initiated.

Specifically errors:

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

| Field | Type | Description |
|------|------|-------------|
| msg | string | Completion timestamp |

Published when an experiment completes successfully.

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
| dipole_vec | Vector3 | Desired dipole unit vector |
| dipole_strength | float64 | Field strength (Am^2)|
| duration | float64 | (Optional) Runtime (sec) |

## Response

| Field | Type | Description |
|------|------|-------------|
| error | bool | If an error prevented service execution |
| error_desc | string | Brief description of cause of error |

Default duration:

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

| Field | Type | Description |
|------|------|--------------|
| omnimagnet | uint64 | Magnet ID |
| rotation_vector | Vector3 | Rotation unit vector |
| dipole_strength | float64 | Dipole strength (Am^2) |
| rotation_freq | float64 | Rotation Frequency (Hz) |
| phase_offset | float64 | Initial Rotation Phase Shift (deg) |
| duration | float64 | (Optional) Runtime (sec) |

## Response
| Field | Type | Description |
|------|------|-------------|
| error | bool | If an error prevented service execution |
| error_desc | string | Brief description of cause of error |

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

| Field | Type | Description |
|------|------|--------------|
| omnimagnets | uint64[] | List of Magnet IDs |
| dipole_vecs | Vector3[] | Index-associated dipole unit vectors |
| dipole_strengths | float64[] | Index-associated dipole strengths (Am^2) |
| duration | float64 | (Optional) Runtime (sec) |

## Response
| Field | Type | Description |
|------|------|-------------|
| error | bool | If an error prevented service execution |
| error_desc | string | Brief description of cause of error |

### Supports

- One dipole vector for all magnets
- One strength for all magnets
- Per-magnet vectors/strengths

### Notes
- `dipole_vecs` must be either length 1 or the same length as `omnimagnets`
- `dipole_strengths` must be either length 1 or the same length as `omnimagnets`

### Examples

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

```bash
ros2 service call /multi_magnet_constant omnimagnet_interfaces/srv/MultiMagnetConstant "
{
  omnimagnets: [0,1,2],
  dipole_vecs: [
    {x: 1.0, y: 0.0, z: 0.0},
    {x: 0.3, y: 0.4, z: 0.5},
    {x: 0.5, y: 0.0, z: 0.5}
  ],
  dipole_strengths: [40.0, 15.0, 25.0],
  duration: 45.0
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

## Request

| Field | Type | Description |
|------|------|--------------|
| omnimagnets | uint64[] | List of Magnet IDs |
| rotation_vectors | Vector3[] | Index-associated rotation unit vectors |
| rotation_freqs | float64[] | Index-associated rotation frequencies (Hz) |
| dipole_strengths | float64[] | Index-associated dipole strengths (Am^2) |
| phase_offsets | float64[] | Index-associated rotation offset (deg) |
| duration | float64 | (Optional) Runtime (sec) |

## Response
| Field | Type | Description |
|------|------|-------------|
| error | bool | If an error prevented service execution |
| error_desc | string | Brief description of cause of error |


### Supports:

- Synchronous rotation
- Shared rotation vector
- Shared frequency
- Shared phase offset
- Shared strength
- Per-magnet overrides
- Negative frequencies

### Notes
-`rotation_vectors`, `rotation_freqs`, `dipole_strengths`, and `phase_offsets` must be either length 1 or the same length as `omnimagnets`

### Phase Calculation
- Initial dipole vector is calculated by crossing the `x` vector with the rotation unit vector `omega`
 - If `x` and `omega` are nearly parallel, the `y` vector is used instead
- Phase shift represents movement in the direction of rotation from this initial vector by `phase_offset` degrees
- If rotation is in the same direction, use identical rotation vectors with same-signed frequencies
- For opposed rotation:
 - Use identical rotation vectors with opposite-signed frequencies (Both vectors will have the same 0-phase angle)
 - Use opposed rotation vectors with same-signed frequencies (Vectors will have opposite 0-phase angle)
  - Identical to using identical rotation vectors with opposite-signed frequencies and a phase-shift of 180 degrees

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

## Request

None

## Response
| Field | Type | Description |
|------|------|-------------|
| status | bool | True if driver successfully reset |

Immediately:

- Stops active experiment
- Turns off all magnets
- Cancels duration timer
- Restarts timeout timer

Operations cannot be run simultaneously; if a new operation is desired before the previous run finishes, `/reset_driver` must be invoked first.

### Example

```bash
ros2 service call /reset_driver omnimagnet_interfaces/srv/DriverReset "{}"
```

---

# Magnet IDs

Currently configured:

| ID | Description |
|----|-------------|
| 1 | Left Upper |
| 2 | Center Upper |
| 3 | Right Upper |
| 4 | Right Lower |
| 5 | Left Lower |

(Upper corresponds to north)

ID 6 is reserved but disabled.

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
60 seconds
```

The driver:

1. Publishes error
2. Safely shuts down magnets
3. Terminates operation

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
Add magnet frame transformations.

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
- [ ] Add individual magnet frame transformations
- [ ] Parameterize timeout values
- [ ] Parameterize control loop frequency
- [ ] Add launch file

---

# Authors

Tyler Wilcox
University of Utah
tyler.c.wilcox@utah.edu