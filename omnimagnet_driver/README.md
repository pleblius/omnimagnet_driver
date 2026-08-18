# omnimagnet_driver

This package contains the source code and header files for the omnimagnet driver and omnimagnet objects, along with the build files for the ros2 package builder.

# Build Files

There are two build files:

* `CMakeLists.txt` contains the CMake build specifications for the package
* `package.xml` contains the package build specifications for ros2

These files should only be altered if code dependencies change.

# Source Code

There are two source files in this package, located in the `src` directory: 

* `omnimagnet.cpp` contains the source code for the omnimagnet objects
* `omnimagnet_driver.cpp` contains the source code for the ros2 driver node

There are two associated header files located in the `include/omnimagnet_driver` directory:

* `omnimagnet_driver.hpp`
* `omnimagnet.hpp`

## Omnimagnet

The source code for the operation of the omnimagnets is contained in `omnimagnet.cpp`.

## Omnimagnet Driver

The source code for the ros2 driver is contained in `omnimagnet_driver.cpp`.