# Omnimagnet Driver

A ROS 2 hardware driver for controlling multiple electromagnetic coil systems ("Omnimagnets") through an Advantech PCI-1724U D2A using `comedilib`.

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

### D2A Card
- **Advantech PCI-1724U**
- Accessed using `comedilib`

### Device Access
Expected D2A device:

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


### Magnet IDs

Currently configured:

| ID | Description |
|----|-------------|
| 1 | Left Upper |
| 2 | Center Upper |
| 3 | Right Upper |
| 4 | Right Lower |
| 5 | Left Lower |

(Upper/Lower corresponds to North/South, respectively. Left/Right corresponds to West/East, respectively.)

ID 6 is reserved but currently disabled.

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

Launch setup is contained in
```omnimagnet_bringup/```
with parameters in 
```omnimagnet_bringup/config/omnimagnet_params.yaml```
and the launch file in 
```omnimagnet_bringup/launch/omnimagnet_driver.launch.xml```

---

## Building

To build the project, make sure the repository in ```~/ros2_ws/src``` is up-to-date. From ```~/ros2_ws```, use the bash command
```bash
colcon build
```

If done correctly, there should be four sub-folders:
```
~/ros2_ws/build
~/ros2_ws/install
~/ros2_ws/log
~/ros2_ws/src
```

If errors occur due to a previous build, deleting the build folders may be necessary. This can be done from ```~/ros2_ws``` using
```bash
rm -rf build install log
```
Invoking ```colcon build``` will then re-build all packages from the source directory.
Note: If the source directory in ```ros2_ws/src``` is deleted, it will need to be reacquired before the project can be built.

## Running

Launch the driver using launch file with configuration parameters:

```bash
ros2 launch omnimagnet_bringup omnimagnet_driver.launch.xml
```

To run the driver directly:
```bash
ros2 run omnimagnet_driver omnimagnet_driver
```

Driver will timeout after 300 seconds of not receiving any requests.

---

## Parameters

While default values for all parameters are provided in the source code, the defaults are implemented such that the driver will not run. The proper parameter values from the ```omnimagnet_bringup``` package are required to operate the driver. These parameters can be adjusted in ```omnimagnet_bringup/config/omnimagnet_params.yaml```.

### Hardware Configuration

The hardware configuration parameters control the D2A setup within the code and should not be adjusted unless the D2A is reconfigured.

### Timing Configuration

The timing configuration parameters are used to control the operation of the driver. There are three parameters:

* timeout_seconds: How long the driver will wait for a new request before shutting off. (Default value: 300.0)
* default_duration_seconds: The default time for a given service request if none is provided. (Default value: 30.0)
* control_frequency_hz: The default update frequency for the control loop to send new current commands to the omnimagnets. (Default value: 1000.0)

### Magnet Configuration

The magnet configuration parameters relate the omnimagnet hardware to the omnimagnet objects, including identification, construction and wiring configuration, and local reference frame. There is one general magnet configuration parameter, ```magnet_count```, which specifies the number of omnimagnets that are connected to the D2A. The rest of the parameters are used for the configuration of the individual magnets; if any given magnet changes, or if a magnet is added or removed, its parameterization must be adjusted accordingly; additionally, the source code will need to be updated to account for the addition or removal of that magnet.

* id: the local identifier used for the given magnet
* enabled: whether or not the magnet is active and should be loaded by the driver
* wire_width: the thickness of the wire used in the magnet's construction
* wire_lengths:
  * inner: length of wire used for the innermost coil
  * mid: length of wire used for the middle coil
  * outer: length of wire used for the outermost coil
* core_size: the diameter of the magnetic core at the center of the omnimagnet
* channels:
  * inner: D2A channel for innermost coil
  * mid: D2A channel for the middle coil
  * outer: D2A channel for the outermost coil
* estimate: whether the dipole-current relationship should be estimated (currently *needs* to be set to true, no other method is provided) 
* frame: the rotation matrix of the magnet's actual dipole output with respect to its local frame. The local frame is standardized as described in ```frames/default.png```

---

# Interfaces

## Vector 3

A copy of ros2's Vector3 interface type to simplify dependencies

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
30.0 seconds
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

Default duration:

```cpp
30.0 seconds
```

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

Default duration:

```cpp
30.0 seconds
```

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

Default duration:

```cpp
30.0 seconds
```

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

Operations *cannot* be run simultaneously; if a new operation is desired before the previous run finishes, `/reset_driver` must be invoked first, either from another ros2 program or from the terminal.

To manually command a driver reset, you can use the terminal command
```bash
ros2 service call /reset_driver omnimagnet_interfaces/srv/DriverReset "{}"
```

---

# Real-Time Control

A dedicated control thread runs at:

```text
1000 Hz
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
300 seconds
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

Then log out and back in to the computer.

---

## Thread Safety

Active commands are protected with:

- `std::mutex`
- `std::atomic`

Do not bypass these protections when modifying control logic.

---

# TODOs

Current code still has several planned improvements:

- [x] Move magnet config to YAML
- [x] Add individual magnet frame transformations
- [x] Parameterize timeout values
- [x] Parameterize control loop frequency
- [x] Add launch file
- [ ] Add transform subscriber for controller world-frame transformations
- [ ] Add current-based alternative drive requests

---

# Authors

Tyler Wilcox, 
University of Utah,
tyler.c.wilcox@utah.edu
