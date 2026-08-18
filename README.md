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

### Single-Magnet Operations
- Constant dipole generation
- Rotating dipole generation
- Constant current generation
- Rotating current generation

### Multi-Magnet Operations
- Multiple synchronized constant dipoles
- Multiple synchronized rotating dipoles
- Multiple synchronized constant currents
- Multiple synchronized rotating currents
- Shared or independent parameters between magnets

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

These are initialized to **5 V** during startup.

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

- ROS 2 Humble
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
omnimagnet_driver/include/omnimagnet_driver/commands.hpp
omnimagnet_driver/src/omnimagnet_driver.cpp
omnimagnet_driver/src/omnimagnet.cpp
omnimagnet_driver/src/commands.cpp
```

Launch setup is contained in
```omnimagnet_bringup/```
with parameters in 
```omnimagnet_bringup/config/omnimagnet_params.yaml```
and the launch file in 
```omnimagnet_bringup/launch/omnimagnet_driver.launch.xml```

---

## Building

To build the project, make sure the repository in ```~/ros2_ws/src``` is up-to-date with the repository from GitHub. From ```~/ros2_ws```, use the bash command
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
Invoking ```colcon build``` will then rebuild all packages from the source directory.
Note: If the source directory in ```ros2_ws/src``` is deleted, it will need to be reacquired before the project can be built.

## Launching

Launch the driver using launch file with configuration parameters:

```bash
ros2 launch omnimagnet_bringup omnimagnet_driver.launch.xml
```

To run the driver directly:
```bash
ros2 run omnimagnet_driver omnimagnet_driver
```
Note: Running the driver directly will not load the parameter file, so all parameters will be default-configured; by default, all hardware is disabled, so the driver will do nothing if passed a command.

The driver will time out after 300 seconds of not receiving any requests.

---

## Parameters

While default values for all parameters are provided in the source code, the defaults are implemented such that the driver will not run. The proper parameter values from the ```omnimagnet_bringup``` package are required to operate the driver. These parameters can be adjusted in ```omnimagnet_bringup/config/omnimagnet_params.yaml```.

### Hardware Configuration

The hardware configuration parameters control the D2A setup within the code and should not be adjusted unless the D2A is reconfigured.

### Timing Configuration

The timing configuration parameters are used to control the operation of the driver. There are two parameters:

* timeout_seconds: How long the driver will wait for a new request before shutting off, in seconds. (Default value: 300.0)
* control_frequency_hz: The default update frequency (in Hertz) for the control loop to send new current commands to the omnimagnets. (Default value: 1000.0)

### Magnet Configuration

The magnet configuration parameters relate the omnimagnet hardware to the omnimagnet objects, including identification, construction and wiring configuration, and local reference frame. The rest of the parameters are used for the configuration of the individual magnets; if any given magnet changes, or if a magnet is added or removed, its parameterization must be adjusted accordingly. (Additionally, the number of magnets set in ```omnimagnet_driver.hpp``` will need to be updated to the appropriate count.)

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

A copy of ROS 2's Vector3 interface type to simplify dependencies

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
| shutdown | bool | True if the driver initiated a shutdown due to error |

Reports errors that prevent proper driver operation. Errors that are considered unrecoverable will initiate a driver shutdown.

Specifically errors:

- Hardware initialization failures
- Magnet write failures
- Runtime driver faults
- Timeout shutdowns

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

## 1. Driver Reset

Service:

```bash
/reset_driver
```

Type:

`DriverReset`

### Request

None

### Response
| Field | Type | Description |
|------|------|-------------|
| status | bool | True if driver successfully reset |

Immediately:

- Stops active experiment
- Turns off all magnets
- Cancels duration timer
- Restarts timeout timer

Operations *cannot* be run simultaneously; if a new operation is desired before the previous run finishes, `/reset_driver` must be invoked first, either from another ROS 2 program or from the terminal.

To manually command a driver reset, you can use the terminal command
```bash
ros2 service call /reset_driver omnimagnet_interfaces/srv/DriverReset "{}"
```

---

## Dipole Requests
For requests that desire a specified magnetic dipole, the driver will use the omnimagnets' internal frames to calculate the necessary currents.
Note: All vectors are converted to unit vectors; strength will be determined by dipole_strength fields.

### 2. Single Magnet Constant

Service:

```bash
/single_magnet_constant
```

Type:

`SingleMagnetConstant`

#### Request

| Field | Type | Description |
|------|------|-------------|
| omnimagnet | uint64 | Magnet ID |
| dipole_vec | Vector3 | Dipole vector (direction) |
| dipole_strength | float64 | Dipole strength (Am^2)|
| duration | float64 | Runtime (sec) |

#### Response

| Field | Type | Description |
|------|------|-------------|
| error | bool | If an error prevented service execution |
| error_desc | string | Brief description of the error |

#### Example

```bash
ros2 service call /single_magnet_constant omnimagnet_interfaces/srv/SingleMagnetConstant 
"{
  omnimagnet: 5,
  dipole_vec: {x: 1.0, y: 0.0, z: 0.0},
  dipole_strength: 40.0,
  duration: 10.0
}"
```

---

### 3. Single Magnet Rotation

Service:

```bash
/single_magnet_rotation
```

Type:

`SingleMagnetRotation`

#### Request

| Field | Type | Description |
|------|------|--------------|
| omnimagnet | uint64 | Magnet ID |
| rotation_vector | Vector3 | Rotation vector |
| dipole_strength | float64 | Dipole strength (Am^2) |
| rotation_freq | float64 | Rotation Frequency (Hz) |
| phase_offset | float64 | Initial Rotation Phase Shift (deg) |
| duration | float64 | Runtime (sec) |

#### Response
| Field | Type | Description |
|------|------|-------------|
| error | bool | If an error prevented service execution |
| error_desc | string | Brief description of error |

#### Example

```bash
ros2 service call /single_magnet_rotation omnimagnet_interfaces/srv/SingleMagnetRotation 
"{
  omnimagnet: 2,
  rotation_vector: {x: 0.0, y: 0.0, z: 1.0},
  dipole_strength: 40.0,
  rotation_freq: 5.0,
  phase_offset: 90.0,
  duration: 20.0
}"
```

#### Phase Calculation
- Initial dipole vector is calculated by crossing the rotation vector with the `x` or `y` axis
  - Uses whichever axis `omega` is less parallel to
- Rotation uses the right-hand rule for positive rotation frequency
- Phase shift will be in the direction of rotation by `phase_offset` degrees
---

### 4. Multi Magnet Constant

Service:

```bash
/multi_magnet_constant
```

Type:

`MultiMagnetConstant`

#### Request

| Field | Type | Description |
|------|------|--------------|
| omnimagnets | uint64[] | List of Magnet IDs |
| dipole_vecs | Vector3[] | Index-associated dipole vectors |
| dipole_strengths | float64[] | Index-associated dipole strengths (Am^2) |
| duration | float64 | Runtime (sec) |

#### Response
| Field | Type | Description |
|------|------|-------------|
| error | bool | If an error prevented service execution |
| error_desc | string | Brief description of error |

#### Supports

- Shared dipole vector
- Shared dipole strength
- Per-magnet overrides
  - Applies to each list independently

#### Notes
- `dipole_vecs` and `dipole_strengths` must be either length 1 or the same length as `omnimagnets`
- Entries are index-associated, meaning `omnimagnets[i]` will have vector `dipole_vecs[i]` and strength `dipole_strengths[i]`
  - Disregarded for any lists that have length 1 (Value is applied universally)

#### Examples
##### Single vector, single strength:
```bash
ros2 service call /multi_magnet_constant omnimagnet_interfaces/srv/MultiMagnetConstant
"{
  omnimagnets: [1, 4, 5],
  dipole_vecs: [
    {x: 1.0, y: 0.0, z: 0.0}
  ],
  dipole_strengths: [40.0],
  duration: 10.0
}"
```

##### Independent vectors, independent strengths:
```bash
ros2 service call /multi_magnet_constant omnimagnet_interfaces/srv/MultiMagnetConstant
"{
  omnimagnets: [2,3,4],
  dipole_vecs: [
    {x: 1.0, y: 0.0, z: 0.0},
    {x: 0.3, y: 0.4, z: 0.5},
    {x: 0.5, y: 0.0, z: 0.5}
  ],
  dipole_strengths: [40.0, 15.0, 25.0],
  duration: 45.0
}"
```

##### Shared vector, independent strengths:
```bash
ros2 service call /multi_magnet_constant omnimagnet_interfaces/srv/MultiMagnetConstant
"{
  omnimagnets: [1, 5],
  dipole_vecs: [
    {x: 1.0, y: 1.0, z: 1.0}
  ],
  dipole_strengths: [5.0, 7.0],
  duration: 17
}"
```
---

### 5. Multi Magnet Rotation

Service:

```bash
/multi_magnet_rotation
```

Type:

`MultiMagnetRotation`

#### Request

| Field | Type | Description |
|------|------|--------------|
| omnimagnets | uint64[] | List of Magnet IDs |
| rotation_vectors | Vector3[] | Index-associated rotation vectors |
| rotation_freqs | float64[] | Index-associated rotation frequencies (Hz) |
| dipole_strengths | float64[] | Index-associated dipole strengths (Am^2) |
| phase_offsets | float64[] | Index-associated rotation offsets (deg) |
| duration | float64 | Runtime (sec) |

#### Response
| Field | Type | Description |
|------|------|-------------|
| error | bool | If an error prevented service execution |
| error_desc | string | Brief description of error |

#### Supports:

- Synchronous rotation
- Shared rotation vector
- Shared frequency
- Shared phase offset
- Shared strength
- Per-magnet overrides
  - Applies to each list independently
- Negative frequencies
- Arbitrary phase shifts

#### Notes
- `rotation_vectors`, `rotation_freqs`, `dipole_strengths`, and `phase_offsets` must be either length 1 or the same length as `omnimagnets`

#### Phase Calculation
- Initial dipole vector is calculated by crossing the rotation vector with the `x` or `y` axis
  - Uses whichever axis `omega` is less parallel to
- Rotation uses the right-hand rule for positive rotation frequency
- Phase shift will be in the direction of rotation by `phase_offset` degrees
- If rotation is in the same direction, use identical rotation vectors with same-signed frequencies
- For opposed rotation:
  - Same starting dipole:
    - Use identical rotation vectors with opposite-signed frequencies
  - Opposite starting dipole:
    - Use opposite rotation vectors with same-signed frequencies
    - Alt: Use identical rotation vectors with opposite-signed frequencies and one 180-degree phase shift

#### Examples
##### Identical Rotation:
```bash
ros2 service call /multi_magnet_rotation omnimagnet_interfaces/srv/MultiMagnetRotation
"{
  omnimagnets: [1, 2],
  rotation_vectors: [
    {x: 0.0, y: 0.0, z: 1.0}
  ],
  dipole_strengths: [40.0],
  rotation_freqs: [5.0],
  phase_offsets: [0.0],
  duration: 20.0
}"
```

##### Opposed Rotation, Same Starting Dipole:
```bash
ros2 service call /multi_magnet_rotation omnimagnet_interfaces/srv/MultiMagnetRotation
"{
  omnimagnets: [4, 5],
  rotation_vectors: [
    {x: 0.5, y: 0.5, z: 0.0}
  ],
  dipole_strengths: [20.0],
  rotation_freqs: [10.0, -10.0],
  phase_offsets: [0.0],
  duration: 10.0
}"
```

##### Opposed Rotation, Opposite Starting Dipole:
```bash
ros2 service call /multi_magnet_rotation omnimagnet_interfaces/srv/MultiMagnetRotation
"{
  omnimagnets: [2, 3],
  rotation_vectors: [
    {x: 1.0, y: 0.0, z: 0.0},
    {x: -1.0, y: 0.0, z: 0.0}
  ],
  dipole_strengths: [15.0],
  rotation_freqs: [10.0],
  phase_offsets: [0.0],
  duration: 15.0
}"
```

##### Opposed Rotation, Opposite Starting Dipole (Alternative):
```bash
ros2 service call /multi_magnet_rotation omnimagnet_interfaces/srv/MultiMagnetRotation
"{
  omnimagnets: [1, 4],
  rotation_vectors: [
    {x: 1.0, y: 0.0, z: 0.0}
  ],
  dipole_strengths: [15.0],
  rotation_freqs: [10.0, -10.0],
  phase_offsets: [0.0, 180.0],
  duration: 15.0
}"
```
---

---

## Current Requests
For requests that desire a specified current through the magnet.
Note: All vectors are converted to unit vectors; strength will be determined by current_strength fields.

### 6. Single Current Constant

Service:

```bash
/single_current_constant
```

Type:

`SingleCurrentConstant`

#### Request

| Field | Type | Description |
|------|------|-------------|
| omnimagnet | uint64 | Magnet ID |
| current_vec | Vector3 | Current vector (direction) |
| current_strength | float64 | Current strength (A)|
| duration | float64 | Runtime (sec) |

#### Response

| Field | Type | Description |
|------|------|-------------|
| error | bool | If an error prevented service execution |
| error_desc | string | Brief description of the error |

#### Example

```bash
ros2 service call /single_current_constant omnimagnet_interfaces/srv/SingleCurrentConstant 
"{
  omnimagnet: 5,
  current_vec: {x: 1.0, y: 0.0, z: 0.0},
  current_strength: 5.0,
  duration: 10.0
}"
```

---

### 7. Single Current Rotation

Service:

```bash
/single_current_rotation
```

Type:

`SingleCurrentRotation`

#### Request

| Field | Type | Description |
|------|------|--------------|
| omnimagnet | uint64 | Magnet ID |
| rotation_vector | Vector3 | Rotation vector |
| current_strength | float64 | Current strength (A) |
| rotation_freq | float64 | Rotation Frequency (Hz) |
| phase_offset | float64 | Initial Rotation Phase Shift (deg) |
| duration | float64 | Runtime (sec) |

#### Response
| Field | Type | Description |
|------|------|-------------|
| error | bool | If an error prevented service execution |
| error_desc | string | Brief description of error |

#### Example

```bash
ros2 service call /single_current_rotation omnimagnet_interfaces/srv/SingleCurrentRotation 
"{
  omnimagnet: 2,
  rotation_vector: {x: 0.0, y: 0.0, z: 1.0},
  current_strength: 2.0,
  rotation_freq: 5.0,
  phase_offset: 90.0,
  duration: 20.0
}"
```

#### Phase Calculation
- Initial current vector is calculated by crossing the rotation vector with the `x` or `y` axis
  - Uses whichever axis `omega` is less parallel to
- Rotation uses the right-hand rule for positive rotation frequency
- Phase shift will be in the direction of rotation by `phase_offset` degrees
---

### 8. Multi Current Constant

Service:

```bash
/multi_current_constant
```

Type:

`MultiCurrentConstant`

#### Request

| Field | Type | Description |
|------|------|--------------|
| omnimagnets | uint64[] | List of Magnet IDs |
| current_vecs | Vector3[] | Index-associated current vectors |
| current_strengths | float64[] | Index-associated current strengths (A) |
| duration | float64 | Runtime (sec) |

#### Response
| Field | Type | Description |
|------|------|-------------|
| error | bool | If an error prevented service execution |
| error_desc | string | Brief description of error |

#### Supports

- Shared current vector
- Shared current strength
- Per-magnet overrides
  - Applies to each list independently

#### Notes
- `current_vecs` and `current_strengths` must be either length 1 or the same length as `omnimagnets`
- Entries are index-associated, meaning `omnimagnets[i]` will have vector `current_vecs[i]` and strength `current_strengths[i]`
  - Disregarded for any lists that have length 1 (Value is applied universally)

#### Examples
##### Single vector, single strength:
```bash
ros2 service call /multi_current_constant omnimagnet_interfaces/srv/MultiCurrentConstant
"{
  omnimagnets: [1, 4, 5],
  current_vecs: [
    {x: 1.0, y: 0.0, z: 0.0}
  ],
  current_strengths: [8.0],
  duration: 10.0
}"
```

##### Independent vectors, independent strengths:
```bash
ros2 service call /multi_current_constant omnimagnet_interfaces/srv/MultiCurrentConstant
"{
  omnimagnets: [2, 3, 4],
  current_vecs: [
    {x: 1.0, y: 0.0, z: 0.0},
    {x: 0.3, y: 0.4, z: 0.5},
    {x: 0.5, y: 0.0, z: 0.5}
  ],
  current_strengths: [2.0, 6.0, 8.0],
  duration: 45.0
}"
```

##### Shared vector, independent strengths:
```bash
ros2 service call /multi_current_constant omnimagnet_interfaces/srv/MultiCurrentConstant
"{
  omnimagnets: [1, 5],
  current_vecs: [
    {x: 1.0, y: 1.0, z: 1.0}
  ],
  current_strengths: [5.0, 7.0],
  duration: 17.0
}"
```
---

### 9. Multi Current Rotation

Service:

```bash
/multi_current_rotation
```

Type:

`MultiCurrentRotation`

#### Request

| Field | Type | Description |
|------|------|--------------|
| omnimagnets | uint64[] | List of Magnet IDs |
| rotation_vectors | Vector3[] | Index-associated rotation vectors |
| rotation_freqs | float64[] | Index-associated rotation frequencies (Hz) |
| current_strengths | float64[] | Index-associated current strengths (A) |
| phase_offsets | float64[] | Index-associated rotation offsets (deg) |
| duration | float64 | Runtime (sec) |

#### Response
| Field | Type | Description |
|------|------|-------------|
| error | bool | If an error prevented service execution |
| error_desc | string | Brief description of error |

#### Supports:

- Synchronous rotation
- Shared rotation vector
- Shared frequency
- Shared phase offset
- Shared strength
- Per-magnet overrides
  - Applies to each list independently
- Negative frequencies
- Arbitrary phase shifts

#### Notes
- `rotation_vectors`, `rotation_freqs`, `current_strengths`, and `phase_offsets` must be either length 1 or the same length as `omnimagnets`

#### Phase Calculation
- Initial current vector is calculated by crossing the rotation vector with the `x` or `y` axis
  - Uses whichever axis `omega` is less parallel to
- Rotation uses the right-hand rule for positive rotation frequency
- Phase shift will be in the direction of rotation by `phase_offset` degrees
- If rotation is in the same direction, use identical rotation vectors with same-signed frequencies
- For opposed rotation:
  - Same starting current:
    - Use identical rotation vectors with opposite-signed frequencies
  - Opposite starting current:
    - Use opposite rotation vectors with same-signed frequencies
    - Alt: Use identical rotation vectors with opposite-signed frequencies and one 180-degree phase shift

#### Examples
##### Identical Rotation:
```bash
ros2 service call /multi_current_rotation omnimagnet_interfaces/srv/MultiCurrentRotation
"{
  omnimagnets: [1, 2],
  rotation_vectors: [
    {x: 0.0, y: 0.0, z: 1.0}
  ],
  current_strengths: [10.0],
  rotation_freqs: [5.0],
  phase_offsets: [0.0],
  duration: 20.0
}"
```

##### Opposed Rotation, Same Starting Current:
```bash
ros2 service call /multi_current_rotation omnimagnet_interfaces/srv/MultiCurrentRotation
"{
  omnimagnets: [2, 3],
  rotation_vectors: [
    {x: 1.0, y: 0.0, z: 0.0}
  ],
  current_strengths: [5.0],
  rotation_freqs: [10.0, -10.0],
  phase_offsets: [0.0],
  duration: 10.0
}"
```

##### Opposed Rotation, Opposite Starting Current:
```bash
ros2 service call /multi_current_rotation omnimagnet_interfaces/srv/MultiCurrentRotation
"{
  omnimagnets: [2, 3],
  rotation_vectors: [
    {x: 1.0, y: 0.0, z: 0.0},
    {x: -1.0, y: 0.0, z: 0.0}
  ],
  current_strengths: [5.0],
  rotation_freqs: [10.0],
  phase_offsets: [0.0],
  duration: 15.0
}"
```

##### Opposed Rotation, Opposite Starting Current (Alternative):
```bash
ros2 service call /multi_current_rotation omnimagnet_interfaces/srv/MultiCurrentRotation
"{
  omnimagnets: [2, 3],
  rotation_vectors: [
    {x: 1.0, y: 0.0, z: 0.0}
  ],
  current_strengths: [5.0],
  rotation_freqs: [10.0, -10.0],
  phase_offsets: [0.0, 180.0],
  duration: 15.0
}"
```

--- 

# Real-Time Control

A dedicated control thread runs at:

```text
1000 Hz
```

The thread:
1. Checks driver state
2. Loads or resets commands if necessary
3. Calculates delta-t for the current experiment
4. Determines necessary current for each magnet
5. Sends current commands to D2A

This keeps ROS callbacks separate from hardware timing.

---

# Safety Behavior

## Node Exit

On shutdown:

- All magnets are set to zero current
- Amplifier pins are set to zero

---

## Timeout

If no controller command is received within the timeout duration (default 300 seconds), the driver will:

1. Publish timeout message
2. Safely shut down magnets
3. Terminate ROS node

---

# Command Validation Checks

Service requests are rejected if:

- Magnet ID(s) are invalid
- Vector magnitude is zero or not finite
- Command duration is not a positive value
- Array size mismatch
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
- [x] Add current-based alternative drive requests
- [ ] Split DriverNode setup and control logic into separate classes
- [ ] Add saturation detection and warning
- [ ] Replace dipole estimation method with calibrated values
- [ ] Add READMEs to subfolders
- [ ] (Maybe) Add transform subscriber for controller world-frame transformations
- [ ] (Maybe) Consider adding command enqueueing

---

# Authors

Tyler Wilcox
University of Utah
Magnetic and Medical Robotics Laboratory
tyler.c.wilcox@utah.edu
