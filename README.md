# MFRC522-linux (Work in progress)
This project is focused on bringign an interface to interact with the MFRC522 chip on a PICO-PI-IMX8M-MINI running on a Kirkstone Linux distro.

## Key implementation concepts
For this implementation the objective is to use the linux API to interact through the SPI protocol using the "linux/spi/spidev.h" interface.

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

# Datasheet
This is the link for the MFRC522 datasheet that was used for the implementation:
https://www.nxp.com/docs/en/data-sheet/MFRC522.pdf
