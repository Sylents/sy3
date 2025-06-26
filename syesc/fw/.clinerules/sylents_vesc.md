# Sylents Vesc Firmware
Embedded Systems Motor Control Firmware for the sylents electric jet drive forked from github vedderb/bldc library

## Purpose:
- control bldc motor
- retrieve steering signal from handle attached via ppm-interface
- provide access via bluetooth module BLE, BLE module is attached via UART Port
- Provide User Status on custom LED-Display
    - Battery Status
    - Current Throttle and or Power

## Current Development Status
The firmware has been customized for the Sylents electric jet drive with the following key features implemented:
- **Custom LED Display Driver**: Integration of a custom driver (`syled`) for user status display, showing battery level, current throttle, and power output (wattage) on a TM1640-based LED display.
- **I2C Communication for Display**: Software-based I2C communication (`swi2c`) to control the LED display, including functions for transmitting data and managing display digits for upper (wattage) and lower (duty cycle) segments.
- **Power Calculation**: Actual power output is derived using a formula, rather than predefined power mapping curves, for accurate wattage calculation based on current conditions.
- **Version Information**: The firmware supports version tracking with major and minor version numbers for both firmware (FW_VERSION_MAJOR, FW_VERSION_MINOR) and hardware (HW_MAJOR, HW_MINOR), displayed during initialization on the LED display.

## Top Level Directory Tree
with focus on sylents related firmware modifications

├───applications                # Application-specific code and configurations
├───blackmagic                  # Blackmagic debug probe integration
├───build                       # Build output and artifacts
│   ├───60                      # Build for specific version or model 60
│   └───s60                     # Build for SYLENTS specific version or model s60
├───ChibiOS_3.0.5               # ChibiOS RTOS version 3.0.5
├───comm                        # Communication protocols and interfaces
├───documentation               # Project documentation and guides
├───downloads                   # Downloaded resources or dependencies
├───driver                      # Hardware drivers
│   └───syled                   # Sylents custom LED display driver
│               syled.c
│               syled.h
├───encoder                     # Encoder-related code for motor position sensing
├───hwconf                      # Hardware configuration files
│   └───sylents
│       ├───s60                 # Config for model s60, includes custom display and power curve logic
│       │       hw_s60.h
│       │       hw_s60_core.c   # Custom implementation for LED display control and power mapping
│       │       hw_s60_core.h
│       │
│       └───s75_300             # Config for model s75_300
│               hw_s75_300.h
│               hw_s75_300_core.c
│               hw_s75_300_core.h
│               hw_s75_300_r2.h
│
├───imu                         # Inertial Measurement Unit code for orientation
├───libcanard                   # CAN bus protocol library
├───lispBM                      # LispBM scripting engine integration
├───make                        # Build scripts and makefiles
├───motor                       # Motor control logic and algorithms
├───Project                     # Main project files or configurations
├───qmlui                       # QML-based user interface components
├───tests                       # Test suites and scripts
├───tools                       # Utility tools and scripts
└───util                        # General utility functions and helpers


# PC & Smartphone Applications
The Firmware provides the API for external 3rd party applications. These external applications are not the scope of this project.

# Sylents Context

For Vesc Library related questions, use the Context7 MCP server with the following command to retrieve up-to-date documentation:
- **Tool Name**: get-library-docs
- **Argument**:
  {
    "context7CompatibleLibraryID": "/vedderb/bldc",
    "tokens": 10000
  }
- **Note**: Adjust the 'tokens' parameter if more or less documentation context is needed. Higher values provide more detailed information but consume more tokens.
