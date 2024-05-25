# STM32F103C8T6 Secure Boot and Secure Firmware Update on STM32F103C8T6

## Overview

This project implements a secure bootloader for the STM32F103C8T6 microcontroller. The bootloader is designed to ensure that only authenticated and unaltered firmware is executed on the microcontroller, enhancing the security of the embedded system.

## Features

- **Secure Boot**: Verifies the integrity and authenticity of the firmware using SHA-256 hashing and digital signatures.
- **Firmware Update**: Supports secure firmware updates via UART.
- **Version Control**: Includes version management for the bootloader.
- **GPIO and UART Initialization**: Configures essential peripherals for communication and control.

## Getting Started

### Prerequisites

- STM32F103C8T6 microcontroller
- STM32CubeIDE or any compatible IDE
- USB to UART converter for serial communication
- Cryptographic library for SHA-256 and signature verification (e.g., mbed TLS)

### Cloning the Repository

```bash
git clone https://github.com/yourusername/stm32-secure-bootloader.git
cd stm32-secure-bootloader
```

### Building the Project

1. Open the project in STM32CubeIDE.
2. Configure the project settings as needed.
3. Build the project to generate the bootloader binary.

### Flashing the Bootloader

Use the ST-Link Utility or STM32CubeProgrammer to flash the bootloader binary to the STM32F103C8T6 microcontroller.

## Code Structure

### main.c

Contains the main function and initialization code. It sets up the system clock, initializes peripherals, and handles the firmware update and application jump.

### Functions

#### `write_data_to_flash_app`

Writes data to the flash memory of the microcontroller.

#### `verify_firmware_signature`

Verifies the integrity and authenticity of the firmware using a hash and digital signature.

#### `goto_application`

Transfers execution from the bootloader to the main application.

### Example Functions

```c
static HAL_StatusTypeDef write_data_to_flash_app(uint8_t *data, uint16_t data_len, bool is_first_block);
int verify_firmware_signature(uint8_t* firmware, uint32_t size, uint8_t* signature, uint8_t* public_key);
static void goto_application(void);
```

## Detailed Function Descriptions

### `write_data_to_flash_app`

Handles the secure writing of data to the microcontroller's flash memory. It unlocks the flash, erases necessary pages if it's the first block, writes the data, and then locks the flash again.

### `verify_firmware_signature`

Calculates the SHA-256 hash of the firmware and verifies its digital signature using a stored public key, ensuring that the firmware is authentic and unaltered.

### `goto_application`

Verifies the application's validity, sets the stack pointer, and jumps to the application's reset handler, effectively handing over control from the bootloader to the application.

## Contributing

Contributions are welcome! Please fork the repository and submit pull requests with your changes. Ensure your code follows the project's coding standards and includes necessary documentation.

## License

This project is licensed under the BSD 3-Clause License. See the LICENSE file for details.

## Contact

For questions or issues, please open an issue on GitHub or contact [your email].

---

Feel free to customize this README file as needed to better suit your project's specific details and requirements.
