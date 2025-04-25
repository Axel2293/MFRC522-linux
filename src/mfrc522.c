/**
 * MFRC522 Library
 *
 * MFRC522 RFID reader connected to a PICO-PI-IMX8MM via SPI.
 */
#include "../include/mfrc522.h"
#include <stdbool.h>

/**
 * Initialize the MFRC522 chip
 *  Init GPIO
 *  Init SPI
 *  Do reset
 *  Reset baud rates
 *  Reset ModWidthReg
 *  Set timeout config
 */
int mfrc522_init(Mfrc522 *dev)
{
    // Initialize GPIO and SPI
    if (mfrc522_init_gpio(dev) != 0)
    {
        fprintf(stderr, "Failed to initialize GPIO\n");
        return -1;
    }
    if (mfrc522_init_spi(dev) != 0)
    {
        fprintf(stderr, "Failed to initialize SPI\n");
        gpiod_chip_close(dev->gpio_chip);
        return -1;
    }
    mfrc522_reset(dev);

    // Reset baud rates
    mfrc522_write_register(dev, TxModeReg, 0x00);
    mfrc522_write_register(dev, RxModeReg, 0x00);
    // Reset ModWidthReg
    mfrc522_write_register(dev, ModWidthReg, 0x26);
    // Set timeout config
    mfrc522_write_register(dev, TModeReg, 0x80);      // TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
    mfrc522_write_register(dev, TPrescalerReg, 0xA9); // TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25μs.
    mfrc522_write_register(dev, TReloadReg_1, 0x03);  // Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
    mfrc522_write_register(dev, TReloadReg_2, 0xE8);
    mfrc522_write_register(dev, TxASKReg, 0x40); // Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
    mfrc522_write_register(dev, ModeReg, 0x3D);  // Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
    mfrc522_antenna_on(dev);                     // Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)
    return 0;
}

/**
 * Initialize GPIO for MFRC522
 */
int mfrc522_init_gpio(Mfrc522 *dev)
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

    return 0;
}

/**
 * Turn on the antenna
 */
void mfrc522_antenna_on(Mfrc522 *dev)
{
    uint8_t value = mfrc522_read_register(dev, TxControlReg);
    if ((value & 0x03) != 0x03)
    {
        mfrc522_setRegisterBitMask(dev, TxControlReg, value | 0x03);
    }
}

/**
 * Reset the MFRC522 using GPIO
 */
void mfrc522_reset(Mfrc522 *dev)
{
    bool hardReset = false;

    // Set the reset line as input
    gpiod_line_release(dev->reset_line);
    if (gpiod_line_request_input(dev->reset_line, "mfrc522_reset") < 0)
    {
        perror("Error requesting GPIO line as input");
        return;
    }

    // Check if reset line gpio is low
    int value = gpiod_line_get_value(dev->reset_line);
    if (value == 0)
    {
        // Set reset pin as output
        gpiod_line_release(dev->reset_line);
        if (gpiod_line_request_output(dev->reset_line, "mfrc522_reset", 0) < 0)
        {
            perror("Error requesting GPIO line as output");
            return;
        }
        // Set reset line to low
        gpiod_line_set_value(dev->reset_line, 0); // Active low reset
        usleep(10000);                            // Hold for 10ms
        gpiod_line_set_value(dev->reset_line, 1); // Release reset
        usleep(50000);                            // Wait 50ms for stabilization
        hardReset = true;
        printf("Hard reset performed\n");
    }

    if (!hardReset)
    {
        mfrc522_write_register(dev, CommandReg, SoftReset); // Issue the SoftReset command.
                                                            // The datasheet does not mention how long the SoftRest command takes to complete.
                                                            // But the MFRC522 might have been in soft power-down mode (triggered by bit 4 of CommandReg)
                                                            // Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74μs. Let us be generous: 50ms.
        uint8_t count = 0;
        do
        {
            // Wait for the PowerDown bit in CommandReg to be cleared (max 3x50ms)
            usleep(50000);
        } while ((mfrc522_read_register(dev, CommandReg) & (1 << 4)) && (++count) < 3);
    }
}

/**
 * Initialize SPI communication
 */
int mfrc522_init_spi(Mfrc522 *dev)
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
uint8_t mfrc522_read_register(Mfrc522 *dev, uint8_t reg)
{
    uint8_t tx[2] = {MFRC522_READ_MSB | (reg & MFRC522_WRITE_MSB), 0};
    uint8_t rx[2] = {0};

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
void mfrc522_write_register(Mfrc522 *dev, uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {MFRC522_WRITE_MSB & reg, value};

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)NULL,
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

void mfrc522_clearRegisterBitMask(Mfrc522 *dev, uint8_t reg, uint8_t mask)
{
    uint8_t tmp = mfrc522_read_register(dev, reg);

    // Clear bits in the register
    tmp &= (~mask);
    mfrc522_write_register(dev, reg, tmp);
}

void mfrc522_setRegisterBitMask(Mfrc522 *dev, uint8_t reg, uint8_t mask)
{
    uint8_t tmp = mfrc522_read_register(dev, reg);

    mfrc522_write_register(dev, reg, tmp | mask);
}

uint8_t mfrc522_communicateWithPICC(
    Mfrc522 *dev,
    uint8_t command,
    uint8_t waitIrq,
    uint8_t *sendData,
    uint8_t sendLen,
    uint8_t *backData,
    uint8_t *backLen,
    uint8_t *validBits,
    uint8_t rxAlign,
    bool checkCRC)
{
    uint8_t txLastBits = validBits ? *validBits : 0;
    uint8_t bitFraming = (rxAlign << 4) + txLastBits; // RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

    mfrc522_write_register(dev, CommandReg, Idle);   // Stop any active command.
    mfrc522_write_register(dev, ComIrqReg, 0x7F);    // Clear all seven interrupt request bits.
    mfrc522_write_register(dev, FIFOLevelReg, 0x80); // FlushBuffer = 1, FIFO init.
    for (size_t i = 0; i < sendLen; i++)
    {
        mfrc522_write_register(dev, FIFODataReg, sendData[i]); // Write sendData to the FIFO
    }
    mfrc522_write_register(dev, BitFramingReg, bitFraming); // Bit adjustments.
    mfrc522_write_register(dev, CommandReg, command);       // Execute command.

    if (command == Transceive)
    {
        mfrc522_setRegisterBitMask(dev, BitFramingReg, 0x80); // StartSend=1, transmission of data starts
    }

    // In mfrc522_init() we set the TAuto flag in TModeReg. This means the timer
    // automatically starts when the PCD stops transmitting.
    //
    // Wait here for the command to complete. The bits specified in the
    // `waitIRq` parameter define what bits constitute a completed command.
    // When they are set in the ComIrqReg register, then the command is
    // considered complete. If the command is not indicated as complete in
    // ~36ms, then consider the command as timed out.

    time_t start_time = time(NULL) * 1000; // Current time in milliseconds
    time_t deadline = start_time + 50;
    bool completed = false;

    do
    {
        uint8_t n = mfrc522_read_register(dev, ComIrqReg); // ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
        if (n & waitIrq)
        { // One of the interrupts that signal success has been set.
            completed = true;
            break;
        }
        if (n & 0x01)
        { // Timer interrupt - nothing received in 25ms
            return STATUS_TIMEOUT;
        }
    } while ((time(NULL) * 1000) < deadline);

    if (!completed)
    {
        return STATUS_TIMEOUT;
    }

    // Stop if errors
    uint8_t errorRegVal = mfrc522_read_register(dev, ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
    if (errorRegVal & 0x13)
    { // Check BufferOvfl ParityErr ProtocolErr
        return STATUS_ERROR;
    }

    // Valid bits
    uint8_t _valBits = 0;

    // If the caller wants data back, get it from the MFRC522.
    if (backData && backLen)
    {
        uint8_t n = mfrc522_read_register(dev, FIFOLevelReg); // Number of bytes in the FIFO
        if (n > *backLen)
        {
            return STATUS_NO_ROOM;
        }
        *backLen = n;
        for (size_t i = 0; i < n; i++)
        {
            backData[i] = mfrc522_read_register(dev, FIFODataReg); // Get received data from FIFO
        }
        _valBits = mfrc522_read_register(dev, ControlReg) & 0x07; // RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.
        if (validBits)
        {
            *validBits = _valBits;
        }
    }

    // Tell collisions
    if (errorRegVal & 0x08)
    {
        return STATUS_COLLISION;
    }

    // TODO
    // // Perform CRC_A validation if requested.
    // if (backData && backLen && checkCRC) {
    //     // In this case a MIFARE Classic NAK is not OK.
    //     if (*backLen == 1 && _valBits == 4) {
    //         return STATUS_MIFARE_NACK;
    //     }
    //     // We need at least the CRC_A value and all 8 bits of the last byte must be received.
    //     if (*backLen < 2 || _valBits != 0) {
    //         return STATUS_CRC_WRONG;
    //     }
    //     uint8_t controlBuffer[2];
    //     uint8_t status = mfrc522_CalculateCRC(dev, backData, *backLen - 2, controlBuffer);
    //     if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1])) {
    //         return STATUS_CRC_WRONG;
    //     }
    // }

    return STATUS_OK;
}

uint8_t mfrc522_transceive_data(
    Mfrc522 *dev,
    uint8_t *sendData,  // Pointer to the data to transfer to the FIFO.
    uint8_t sendLen,    // Number of bytes to transfer to the FIFO.
    uint8_t *backData,  // null ptr or pointer to the buffer if data should be read back after executing the command.
    uint8_t *backLen,   // In: Max number of bytes to write to *backData. Out: The number of bytes returned.
    uint8_t *validBits, // In/Out: Number of valid bits in the last byte. 0 for 8 valid bits. Default nullptr.
    uint8_t rxAlign,    // In: Defines the bit position in backData[0] for the first bit received. Default 0.
    bool checkCRC       // In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
)
{
    uint8_t waitIRq = 0x30; // RxIRq and IdleIRq
    return mfrc522_communicateWithPICC(dev, Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
}

uint8_t mfrc522_CalculateCRC(Mfrc522 *dev, uint8_t *data, uint8_t length, uint8_t *result)
{
    mfrc522_write_register(dev, CommandReg, Idle);       // Stop any active command.
    mfrc522_write_register(dev, DivIrqReg, 0x04);        // Clear the CRCIRq interrupt request bit
    mfrc522_setRegisterBitMask(dev, FIFOLevelReg, 0x80); // FlushBuffer = 1, FIFO init.
    for (size_t i = 0; i < length; i++)
    {
        mfrc522_write_register(dev, FIFODataReg, data[i]); // Write data to the FIFO
    }
    mfrc522_write_register(dev, CommandReg, CalcCRC); // Start the CRC coprocessor

    // Wait for the CRC calculation to complete. Check for the register to
    // indicate that the CRC calculation is complete in a loop. If the
    // calculation is not indicated as complete in ~90ms, then time out
    // the operation.
    const time_t deadline = time(NULL) * 1000 + 150;

    do
    {
        // DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
        uint8_t n = mfrc522_read_register(dev, DivIrqReg);
        if (n & 0x04)
        {
            mfrc522_write_register(dev, CommandReg, Idle); // Stop calculating CRC for new content in the FIFO.
            // Transfer the result from the registers to the result buffer
            result[0] = mfrc522_read_register(dev, CRCResultReg_2);
            result[1] = mfrc522_read_register(dev, CRCResultReg_1);
            return STATUS_OK;
        }
        usleep(1000); // Sleep for 1ms instead
    } while ((time(NULL) * 1000) < deadline);

    // Timeout
    return STATUS_TIMEOUT;
}

bool mfrc522_isNewCardPresent(Mfrc522 *dev)
{
    uint8_t buff[2] = {0, 0};
    uint8_t size = sizeof(buff);

    // Reset antena
    mfrc522_write_register(dev, TxModeReg, 0x00);
    mfrc522_write_register(dev, RxModeReg, 0x00);

    // Reset ModWidthreg
    mfrc522_write_register(dev, ModWidthReg, 0x26);

    // Send request
    uint8_t res = PICC_RequestA(dev, buff, &size);

    if (DEBUG)
    {
        printf("IS NEW CARD PRESENT: %02X %02X\n", buff[0], buff[1]);
    }
    return (res == STATUS_OK || res == STATUS_COLLISION);
}

void mfrc522_cleanup(Mfrc522 *dev)
{
    if (dev->reset_line)
        gpiod_line_release(dev->reset_line);

    if (dev->gpio_chip)
        gpiod_chip_close(dev->gpio_chip);

    if (dev->spi_fd >= 0)
        close(dev->spi_fd);
}

uint8_t PICC_RequestA(Mfrc522 *dev, uint8_t *bufferATQA, uint8_t *bufferSize)
{
    return PICC_REQA_or_WUPA(dev, PICC_CMD_REQA, bufferATQA, bufferSize);
}

uint8_t PICC_REQA_or_WUPA(Mfrc522 *dev, uint8_t command, uint8_t *bufferATQA, uint8_t *bufferSize)
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
    status = mfrc522_transceive_data(dev, &command, 1, bufferATQA, bufferSize, &validBits, 0, false);
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

uint8_t PICC_Select(Mfrc522 *dev, Uid *uid, uint8_t validBits)
{
    bool uidComplete;
    bool selectDone;
    bool useCascadeTag;
    uint8_t cascadeLevel = 1;
    uint8_t result;
    uint8_t count;
    uint8_t checkBit;
    uint8_t index;
    uint8_t uidIndex;             // The first index in uid->uidByte[] that is used in the current Cascade Level.
    int8_t currentLevelKnownBits; // The number of known UID bits in the current Cascade Level.
    uint8_t buffer[9];            // The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
    uint8_t bufferUsed;           // The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO.
    uint8_t rxAlign;              // Used in BitFramingReg. Defines the bit position for the first bit received.
    uint8_t txLastBits;           // Used in BitFramingReg. The number of valid bits in the last transmitted byte.
    uint8_t *responseBuffer;
    uint8_t responseLength;

    // Description of buffer structure:
    //		Byte 0: SEL 				Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
    //		Byte 1: NVB					Number of Valid Bits (in complete command, not just the UID): High nibble: complete bytes, Low nibble: Extra bits.
    //		Byte 2: UID-data or CT		See explanation below. CT means Cascade Tag.
    //		Byte 3: UID-data
    //		Byte 4: UID-data
    //		Byte 5: UID-data
    //		Byte 6: BCC					Block Check Character - XOR of bytes 2-5
    //		Byte 7: CRC_A
    //		Byte 8: CRC_A
    // The BCC and CRC_A are only transmitted if we know all the UID bits of the current Cascade Level.
    //
    // Description of bytes 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
    //		UID size	Cascade level	Byte2	Byte3	Byte4	Byte5
    //		========	=============	=====	=====	=====	=====
    //		 4 bytes		1			uid0	uid1	uid2	uid3
    //		 7 bytes		1			CT		uid0	uid1	uid2
    //						2			uid3	uid4	uid5	uid6
    //		10 bytes		1			CT		uid0	uid1	uid2
    //						2			CT		uid3	uid4	uid5
    //						3			uid6	uid7	uid8	uid9

    // Sanity checks
    if (validBits > 80)
    {
        return STATUS_INVALID;
    }

    // Prepare the MFRC522
    mfrc522_clearRegisterBitMask(dev, CollReg, 0x80); // ValuesAfterColl=1 => Bits received after collision are cleared.

    // Repeat Cascade Level loop until we have a complete UID.
    uidComplete = false;
    while (!uidComplete)
    {
        // Set the Cascade Level in the SEL byte, find out if we need to use the Cascade Tag in byte 2.
        switch (cascadeLevel)
        {
        case 1:
            buffer[0] = PICC_CMD_SEL_CL1;
            uidIndex = 0;
            useCascadeTag = validBits && uid->size > 4; // When we know that the UID has more than 4 bytes
            break;
        case 2:
            buffer[0] = PICC_CMD_SEL_CL2;
            uidIndex = 3;
            useCascadeTag = validBits && uid->size > 7; // When we know that the UID has more than 7 bytes
        case 3:
            buffer[0] = PICC_CMD_SEL_CL3;
            uidIndex = 6;
            useCascadeTag = false; // Never used in CL3.
        default:
            return STATUS_INTERNAL_ERROR;
            break;
        }

        // How many UID bits are known in this Cascade Level?
        currentLevelKnownBits = validBits - (8 * uidIndex);
        if (currentLevelKnownBits < 0)
        {
            currentLevelKnownBits = 0;
        }
        // Copy the known bits from uid->uidByte[] to buffer[]
        index = 2; // destination index in buffer[]
        if (useCascadeTag)
        {
            buffer[index++] = PICC_CMD_CT;
        }
        uint8_t bytesToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0);
        if (bytesToCopy)
        {
            uint8_t maxBytes = useCascadeTag ? 3 : 4; // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
            if (bytesToCopy > maxBytes)
            {
                bytesToCopy = maxBytes;
            }
            for (count = 0; count < bytesToCopy; count++)
            {
                buffer[index++] = uid->uidByte[uidIndex + count];
            }
        }
        // Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
        if (useCascadeTag)
        {
            currentLevelKnownBits += 8;
        }

        // Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
        selectDone = false;
        while (!selectDone)
        {
            // Find out how many bits and bytes to send and receive.
            if (currentLevelKnownBits >= 32)
            {                     // All UID bits in this Cascade Level are known. This is a SELECT.
                buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole bytes
                // Calculate BCC - Block Check Character
                buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
                // Calculate CRC_A
                result = mfrc522_CalculateCRC(dev, buffer, 7, &buffer[7]); // TODO
                if (result != STATUS_OK)
                {
                    return result;
                }
                txLastBits = 0; // 0 => All 8 bits are valid.
                bufferUsed = 9;
                // Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
                responseBuffer = &buffer[6];
                responseLength = 3;
            }
            else
            { // This is an ANTICOLLISION.
                txLastBits = currentLevelKnownBits % 8;
                count = currentLevelKnownBits / 8;     // Number of whole bytes in the UID part.
                index = 2 + count;                     // Number of whole bytes: SEL + NVB + UIDs
                buffer[1] = (index << 4) + txLastBits; // NVB - Number of Valid Bits
                bufferUsed = index + (txLastBits ? 1 : 0);
                // Store response in the unused part of buffer
                responseBuffer = &buffer[index];
                responseLength = sizeof(buffer) - index;
            }

            // Set bit adjustments
            rxAlign = txLastBits;                                                    // Having a separate variable is overkill. But it makes the next line easier to read.
            mfrc522_write_register(dev, BitFramingReg, (rxAlign << 4) + txLastBits); // RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

            // Transmit the buffer and receive the response.

            result = mfrc522_transceive_data(dev, buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign, false);
            if (result == STATUS_COLLISION)
            {                                                                 // More than one PICC in the field => collision.
                uint8_t valueOfCollReg = mfrc522_read_register(dev, CollReg); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
                if (valueOfCollReg & 0x20)
                {                            // CollPosNotValid
                    return STATUS_COLLISION; // Without a valid collision position we cannot continue
                }
                uint8_t collisionPos = valueOfCollReg & 0x1F; // Values 0-31, 0 means bit 32.
                if (collisionPos == 0)
                {
                    collisionPos = 32;
                }
                if (collisionPos <= currentLevelKnownBits)
                { // No progress - should not happen
                    return STATUS_INTERNAL_ERROR;
                }
                // Choose the PICC with the bit set.
                currentLevelKnownBits = collisionPos;
                count = currentLevelKnownBits % 8; // The bit to modify
                checkBit = (currentLevelKnownBits - 1) % 8;
                index = 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First byte is index 0.
                buffer[index] |= (1 << checkBit);
            }
            else if (result != STATUS_OK)
            {
                return result;
            }
            else
            { // STATUS_OK
                if (currentLevelKnownBits >= 32)
                {                      // This was a SELECT.
                    selectDone = true; // No more anticollision
                                       // We continue below outside the while.
                }
                else
                { // This was an ANTICOLLISION.
                    // We now have all 32 bits of the UID in this Cascade Level
                    currentLevelKnownBits = 32;
                    // Run loop again to do the SELECT.
                }
            }
        } // End of while (!selectDone)

        // We do not check the CBB - it was constructed by us above.

        // Copy the found UID bytes from buffer[] to uid->uidByte[]
        index = (buffer[2] == PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
        bytesToCopy = (buffer[2] == PICC_CMD_CT) ? 3 : 4;
        for (count = 0; count < bytesToCopy; count++)
        {
            uid->uidByte[uidIndex + count] = buffer[index++];
        }

        // Check response SAK (Select Acknowledge)
        if (responseLength != 3 || txLastBits != 0)
        { // SAK must be exactly 24 bits (1 byte + CRC_A).
            return STATUS_ERROR;
        }
        // Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed anymore.
        result = mfrc522_CalculateCRC(dev, responseBuffer, 1, &buffer[2]);
        if (result != STATUS_OK)
        {
            return result;
        }
        if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2]))
        {
            return STATUS_CRC_WRONG;
        }
        if (responseBuffer[0] & 0x04)
        { // Cascade bit set - UID not complete yes
            cascadeLevel++;
        }
        else
        {
            uidComplete = true;
            uid->sak = responseBuffer[0];
        }
    } // End of while (!uidComplete)

    // Set correct uid->size
    uid->size = 3 * cascadeLevel + 1;

    return STATUS_OK;
}

bool PICC_ReadCardSerial(Mfrc522 *dev, Uid *uid)
{
    uint8_t result = PICC_Select(dev, uid, 0);
    printf("PICC_Select: %02X\n", result);
    return (result == STATUS_OK);
}
