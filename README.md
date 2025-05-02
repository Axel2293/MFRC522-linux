# MFRC522-linux (Work in progress)
This project is focused on bringign an interface to interact with the MFRC522 chip on a PICO-PI-IMX8M-MINI running on a Kirkstone Linux distro.

## Key implementation concepts
For this implementation, the objective is to use the Linux API to interact through the SPI protocol using the "linux/spi/spidev.h" interface. Also, the gpio pins are managed through the gpiod.h interface, which makes it simpler than using the "linux/gpio.h" interface.

## Pin layout
Connected to the expansion IO on the board.
| MFRC522 Pin | PICO-PI-IMX8M-MINI Pin | Description |
|-------------|-------------|-------------|
| SDA/CS      | ECSPI1_SS0       | SPI Chip Select |
| SCK         | ECSPI1_SCLK       | SPI Clock |
| MOSI        | ECSPI1_MOSI       | SPI Master Out Slave In |
| MISO        | ECSPI1_MISO       | SPI Master In Slave Out |
| GND         | GND         | Ground |
| RST         | GPIO_P30       | Reset |
| VCC         | 3.3V        | Power Supply |

## Project prototype
Here is a project I developed for my IOT class using this library, a mfrc522 reader and the PICO-PI-IMX8M-MINI. 
The project consists of an access control system that works through the communication of a python app with a sub-process C app with sockets.
## Credits
Full credits go to the miguelbalboa/rfid library for providing the implementation to interact with the mfrc522 and RFID cards.

# Datasheet
This is the link for the MFRC522 datasheet that was used for the implementation:
https://www.nxp.com/docs/en/data-sheet/MFRC522.pdf
