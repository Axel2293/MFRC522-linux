/**
 * MFRC522 Library
 *
 * MFRC522 RFID reader connected to a PICO-PI-IMX8MM via SPI.
 */

#include "../include/mfrc522.h"

/**
 * Initialize the MFRC522 chip
 *  Init GPIO
 *  Init SPI
 *  Do reset
 *  Reset baud rates
 *  Reset ModWidthReg
 *  Set timeout config
 */
int mfrc522_init(mfrc522 *dev)
{
    // TODO
}

/**
 * Initialize GPIO for MFRC522
 */
int mfrc522_init_gpio(mfrc522 *dev)
{
    // Open GPIO chip
    dev->gpio_chip = gpiod_chip_open_by_name(GPIO_CHIP_NAME);
    if (!dev->gpio_chip)
    {
        perror("Error opening GPIO chip");
        return -1;
    }

    // Get reset line
    dev->reset_line = gpiod_chip_get_line(dev->gpio_chip, RESET_PIN);
    if (!dev->reset_line)
    {
        perror("Error getting GPIO line");
        gpiod_chip_close(dev->gpio_chip);
        return -1;
    }

    // Configure reset line as output with initial high state
    if (gpiod_line_request_output(dev->reset_line, "mfrc522_reset", 1) < 0)
    {
        perror("Error configuring GPIO line");
        gpiod_chip_close(dev->gpio_chip);
        return -1;
    }

    return 0;
}

/**
 * Reset the MFRC522 using GPIO
 */
void mfrc522_reset(mfrc522 *dev)
{
    // Reset sequence
    gpiod_line_set_value(dev->reset_line, 0); // Active low reset
    usleep(10000);                            // Hold for 10ms
    gpiod_line_set_value(dev->reset_line, 1); // Release reset
    usleep(50000);                            // Wait 50ms for stabilization
}

/**
 * Initialize SPI communication
 */
int mfrc522_init_spi(mfrc522 *dev)
{
    int ret;
    uint8_t mode = SPI_MODE;
    uint8_t bits = SPI_BITS_PER_WORD;
    uint32_t speed = SPI_SPEED;

    // Open SPI device
    dev->spi_fd = open(SPI_DEVICE, O_RDWR);
    if (dev->spi_fd < 0)
    {
        perror("Error opening SPI device");
        return EXIT_FAILURE;
    }

    // Set SPI mode
    ret = ioctl(dev->spi_fd, SPI_IOC_WR_MODE, &mode);
    if (ret < 0)
    {
        perror("Error setting SPI mode");
        close(dev->spi_fd);
        return EXIT_FAILURE;
    }

    // Set bits per word
    ret = ioctl(dev->spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    if (ret < 0)
    {
        perror("Error setting SPI bits per word");
        close(dev->spi_fd);
        return EXIT_FAILURE;
    }

    // Set max speed
    ret = ioctl(dev->spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    if (ret < 0)
    {
        perror("Error setting SPI speed");
        close(dev->spi_fd);
        return EXIT_FAILURE;
    }

    return 0;
}

/**
 * Read one byte from the specified register in the MFRC522 chip
 *  See 8.1.2.1 in the MFRC522 datasheet for details.
 *  - The MSB of the register address is set to 1 to indicate a read operation (8.1.2.3).
 */
uint8_t mfrc522_read_register(mfrc522 *dev, uint8_t reg)
{
    uint8_t tx[2] = {MFRC522_READ_MSB | reg, 0};
    uint8_t rx[2] = {0, 0};

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = 2,
        .speed_hz = SPI_SPEED,
        .bits_per_word = SPI_BITS_PER_WORD,
        .delay_usecs = 0,
    };

    int ret = ioctl(dev->spi_fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 0)
    {
        perror("Error in SPI transfer");
        exit(EXIT_FAILURE);
    }

    // Second byte of the response contains the register byte
    return rx[1];
}

/**
 * Write a byte to the specified register
 */
void mfrc522_write_register(mfrc522 *dev, uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {MFRC522_WRITE_MSB & reg, value};
    uint8_t rx[2] = {0, 0};

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = 2,
        .speed_hz = SPI_SPEED,
        .bits_per_word = SPI_BITS_PER_WORD,
        .delay_usecs = 0};

    if (ioctl(dev->spi_fd, SPI_IOC_MESSAGE(1), &tr) < 0)
    {
        perror("Error in SPI transfer");
        exit(EXIT_FAILURE);
    }
}

/**
 * Clear bits in the specified register
 *  - The bits to clear are specified by the mask.
 */
void mfrc522_clearRegisterBitMask(mfrc522 *dev, uint8_t reg, uint8_t mask) {
    uint8_t tmp = mfrc522_read_register(dev, reg);

    // Clear bits in the register
    tmp &= (~mask);
    mfrc522_write_register(dev, reg, tmp);
}

/**
 * Set bits in the specified register
 */
void mfrc522_setRegisterBitMask(mfrc522 *dev, uint8_t reg, uint8_t mask) {
    uint8_t tmp = mfrc522_read_register(dev, reg);

    mfrc522_write_register(dev, reg, tmp | mask);
}

uint8_t mfrc522_communicateWithPICC( //TODO: Add descriptions to arguments
    mfrc522 *dev,
    uint8_t command,
    uint8_t waitIrq,
    uint8_t *sendData,
    uint8_t sendLen,
    uint8_t *backData,
    uint8_t *backLen,
    uint8_t *validBits,
    uint8_t rxAlign,
    bool checkCRC
) {
    uint8_t txLastBits = validBits ? *validBits : 0;
    uint8_t bitFraming = (rxAlign << 4) + txLastBits;       // RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

    mfrc522_write_register(CommandReg, Idle);               // Stop any active command.
    mfrc522_write_register(ComIrqReg, 0x7F);                // Clear all seven interrupt request bits.
    mfrc522_write_register(FIFOLevelReg, 0x80);             // FlushBuffer = 1, FIFO init.
    mfrc522_write_register(FIFODataReg, sendLen, sendData); // TODO: Implement multy byte write to register
    mfrc522_write_register(BitFramingReg, bitFraming);      // Bit adjustments.
    mfrc522_write_register(CommandReg, command);            // Execute command.

    if (command == Transceive) {
        mfrc522_setRegisterBitMask(BitFramingReg, 0x80);    // StartSend=1, transmission of data starts
    }

    // Wait for the command to complete (max 36ms)
    uint32_t start_time = time(NULL) * 1000;  // Current time in milliseconds
    uint32_t deadline = start_time + 36;
    bool completed = false;

    do {
        uint8_t n = mfrc522_read_register(dev, ComIrqReg);  // ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
        if (n & waitIrq) {                    // One of the interrupts that signal success has been set.
            completed = true;
            break;
        }
        if (n & 0x01) {                       // Timer interrupt - nothing received in 25ms
            return STATUS_TIMEOUT;
        }
        usleep(1000);  // Sleep for 1ms instead of yield()
    } while ((time(NULL) * 1000) < deadline);
    
    if (!completed) {
        return STATUS_TIMEOUT;
    }

    // Stop if errors
    uint8_t errorRegVal = mfrc522_read_register(dev, ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
    if (errorRegVal & 0x13) {               // Check BufferOvfl ParityErr ProtocolErr
        return STATUS_ERROR;
    }

    // Valid bits
    uint8_t _valBits = 0;

    // If the caller wants data back, get it from the MFRC522.
    if (backData && backLen) {
        uint8_t n = mfrc522_read_register(dev, FIFOLevelReg);       // Number of bytes in the FIFO
        if (n > *backLen) {
            return STATUS_NO_ROOM;
        }
        *backLen = n;
        mfrc522_read_register(dev, n, backData, rxAlign);           // TODO: mult byte read
        _valBits = mfrc522_read_register(dev, ControlReg) & 0x07;   // RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.
        if (validBits) {
            *validBits = _valBits;
        }
    }

    // Tell collisions
    if (errorRegVal & 0x08) {
        return STATUS_COLLISION;
    }

    // Perform CRC_A validation if requested.

    return STATUS_OK;
}

uint8_t mfrc522_transceive_data(
        mfrc522 *dev,           
        uint8_t *sendData,      // Pointer to the data to transfer to the FIFO.
        uint8_t sendLen,        // Number of bytes to transfer to the FIFO.
        uint8_t *backData,      // null ptr or pointer to the buffer if data should be read back after executing the command.
        uint8_t *backLen,       // In: Max number of bytes to write to *backData. Out: The number of bytes returned.
        uint8_t *validBits,     // In/Out: Number of valid bits in the last byte. 0 for 8 valid bits. Default nullptr.
        uint8_t rxAlign,        // In: Defines the bit position in backData[0] for the first bit received. Default 0.
        bool    checkCRC        // In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
    )
{
    uint8_t waitIRq = 0x30;
    return mfrc522_communicateWithPICC();
}

bool mfrc522_isNewCardPresent(mfrc522 *dev)
{
    uint8_t buff[2];
    uint8_t size = sizeof(buff);

    // Reset antena
    mfrc522_write_register(TxModeReg, 0x00);
    mfrc522_write_register(RxModeReg, 0x00);

    // Reset ModWidthreg
    mfrc522_write_register(ModWidthReg, 0x26);

    // Send request
    uint8_t res = PICC_RequestA()
}

uint8_t PICC_RequestA(mfrc522 *dev, uint8_t *bufferATQA, uint8_t *bufferSize) {
    return PICC_REQA_or_WUPA(dev, PICC_CMD_REQA, bufferATQA, bufferSize);
}

uint8_t PICC_REQA_or_WUPA(mfrc522 *dev, uint8_t command, uint8_t *bufferATQA, uint8_t *bufferSize)
{
    uint8_t validBits;
    int status;

    if (bufferATQA == NULL || *bufferSize < 2)
    {
        return STATUS_NO_ROOM;
    }

    // Clear collision register
    mfrc522_clearRegisterBitMask(dev, CollReg, 0x80);

    // Set valid bits for REQA and WUPA
    validBits = 7;

    // Transceive data
    status = mfrc522_transceive_data(dev, &command, 1, bufferATQA, bufferSize, &validBits);
    if (status != STATUS_OK)
    {
        return status;
    }

    if (*bufferSize != 2 || validBits != 0)
    {
        return STATUS_ERROR;
    }

    return STATUS_OK;
}

/**
 * Release resources
 *  - Release the GPIO line and close the GPIO chip.
 *  - Close the SPI device file descriptor.
 */
void mfrc522_cleanup(mfrc522 *dev)
{
    if (dev->reset_line)
        gpiod_line_release(dev->reset_line);

    if (dev->gpio_chip)
        gpiod_chip_close(dev->gpio_chip);

    if (dev->spi_fd >= 0)
        close(dev->spi_fd);
}
