#ifndef MFRC522_H
#define MFRC522_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <gpiod.h>
#include <time.h>

/**
 * MFRC522 registers. Described in chapter 9.2 of the datasheet.
 *  When using SPI all addresses are shifted one bit left in the "SPI address byte" (8.1.2.3)
 *  The MSB of the register address is set to 1 for read operations and 0 for write operations.
 *  The LSB of the register address is set to 0 for all operations.
 */
//Page 0: Command and status
#define CommandReg              (0x01 << 1)    // starts and stops command execution
#define ComIEnReg              (0x02 << 1)    // enable and disable interrupt request control bits
#define DivIEnReg              (0x03 << 1)    // enable and disable interrupt request control bits
#define ComIrqReg              (0x04 << 1)    // interrupt request bits
#define DivIrqReg              (0x05 << 1)    // interrupt request bits
#define ErrorReg               (0x06 << 1)    // error bits showing the error status of the last command executed 
#define Status1Reg             (0x07 << 1)    // communication status bits
#define Status2Reg             (0x08 << 1)    // receiver and transmitter status bits
#define FIFODataReg            (0x09 << 1)    // input and output of 64 byte FIFO buffer
#define FIFOLevelReg           (0x0A << 1)    // number of bytes stored in the FIFO buffer
#define WaterLevelReg          (0x0B << 1)    // level for FIFO underflow and overflow warning
#define ControlReg             (0x0C << 1)    // miscellaneous control registers
#define BitFramingReg          (0x0D << 1)    // adjustments for bit-oriented frames
#define CollReg                (0x0E << 1)    // bit position of the first bit-collision detected on the RF interface

// Page 1: Command
#define ModeReg                (0x11 << 1)    // defines general modes for transmitting and receiving 
#define TxModeReg              (0x12 << 1)    // defines transmission data rate and framing
#define RxModeReg              (0x13 << 1)    // defines reception data rate and framing
#define TxControlReg           (0x14 << 1)    // controls the logical behavior of the antenna driver pins TX1 and TX2
#define TxASKReg               (0x15 << 1)    // controls the setting of the transmission modulation
#define TxSelReg               (0x16 << 1)    // selects the internal sources for the antenna driver
#define RxSelReg               (0x17 << 1)    // selects internal receiver settings
#define RxThresholdReg         (0x18 << 1)    // selects thresholds for the bit decoder
#define DemodReg               (0x19 << 1)    // defines demodulator settings
#define MfTxReg                (0x1C << 1)    // controls some MIFARE communication transmit parameters
#define MfRxReg                (0x1D << 1)    // controls some MIFARE communication receive parameters
#define SerialSpeedReg         (0x1F << 1)    // selects the speed of the serial UART interface

// Page 2: Configuration
#define CRCResultReg_1        (0x21 << 1)    // shows the MSB values of the CRC calculation
#define CRCResultReg_2        (0x22 << 1)    // shows the LSB values of the CRC calculation
#define ModWidthReg           (0x24 << 1)    // controls the ModWidth setting
#define RFCfgReg              (0x26 << 1)    // configures the receiver gain
#define GsNReg                (0x27 << 1)    // selects the conductance of the antenna driver pins TX1 and TX2 for modulation
#define CWGsPReg              (0x28 << 1)    // defines the conductance of the p-driver output during periods of no modulation
#define ModGsPReg             (0x29 << 1)    // defines the conductance of the p-driver output during periods of modulation
#define TModeReg              (0x2A << 1)    // defines settings for the internal timer
#define TPrescalerReg         (0x2B << 1)    // defines timer prescaler value
#define TReloadReg_1          (0x2C << 1)    // defines 16-bit timer reload value (high)
#define TReloadReg_2          (0x2D << 1)    // defines 16-bit timer reload value (low)
#define TCounterValReg_1      (0x2E << 1)    // shows 16-bit timer value (high)
#define TCounterValReg_2      (0x2F << 1)    // shows 16-bit timer value (low)

// Page 3: Test registers
#define TestSel1Reg           (0x31 << 1)    // general test signal configuration
#define TestSel2Reg           (0x32 << 1)    // general test signal configuration and PRBS control
#define TestPinEnReg          (0x33 << 1)    // enables pin output driver on pins D1 to D7
#define TestPinValueReg       (0x34 << 1)    // defines the values for D1 to D7 when used as I/O bus
#define TestBusReg            (0x35 << 1)    // shows the status of the internal test bus
#define AutoTestReg           (0x36 << 1)    // controls the digital self test
#define VersionReg            (0x37 << 1)    // shows the software version
#define AnalogTestReg         (0x38 << 1)    // controls the pins AUX1 and AUX2
#define TestDAC1Reg           (0x39 << 1)    // defines the test value for TestDAC1
#define TestDAC2Reg           (0x3A << 1)    // defines the test value for TestDAC2
#define TestADCReg            (0x3B << 1)    // shows the value of ADC I and Q channels

// Use logicar OR to combine the register address with the read bit
#define MFRC522_READ_MSB        0x80          // MSB for read operation
// Use logical AND to combine the register address with the write bit
#define MFRC522_WRITE_MSB       0x7F          // MSB for write operation

// GPIO Configuration
#define GPIO_CHIP_NAME "gpiochip2"          // GPIO chip name that contains the reset pin (under /dev/gpiochipN)
#define RESET_PIN 30                        // GPIO pin number for the MFRC522 reset line

// SPI Configuration
#define SPI_DEVICE "/dev/spidev1.0"
#define SPI_SPEED 500000                    // 1MHz
#define SPI_BITS_PER_WORD 8                 // 8 bits per word
#define SPI_MODE SPI_MODE_0                 // SPI mode 0 (CPOL=0, CPHA=0)

typedef struct {
    int spi_fd;
    struct gpiod_chip *gpio_chip;
    struct gpiod_line *reset_line;
} mfrc522;

/**
 * Initialize mfrc522 device
 */

int mfrc522_init_gpio(mfrc522 *dev);
int mfrc522_init_spi(mfrc522 *dev);
void mfrc522_reset(mfrc522 *dev);

/**
 * Register operations
 */
// Read one byte from the specified register in the MFRC522 chip
uint8_t mfrc522_read_register(mfrc522 *dev, uint8_t reg);
// Read multiple bytes from the specified register in the MFRC522 chip
int mfrc522_read_register_multiple(mfrc522 *dev, uint8_t reg, uint8_t count, uint8_t *values, uint8_t rxAlign);
// Write one byte to the specified register in the MFRC522 chip
void mfrc522_write_register(mfrc522 *dev, uint8_t reg, uint8_t value);
// Write multiple bytes to the specified register in the MFRC522 chip
void mfrc522_write_register_multiple(mfrc522 *dev, uint8_t reg, uint8_t count, uint8_t *values);

/**
 * Clean up resources
 */
void mfrc522_cleanup(mfrc522 *dev);

#endif /* MFRC522_H */
