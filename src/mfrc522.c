/**
 * MFRC522 Library
 * 
 * MFRC522 RFID reader connected to a PICO-PI-IMX8MM via SPI.
 */

#include "../include/mfrc522.h"

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
uint8_t mfrc522_read_register(mfrc522 *dev, uint8_t reg) {
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
    if (ret < 0) {
        perror("Error in SPI transfer");
        return EXIT_FAILURE;
    }

    // Second byte of the response contains the register byte
    return rx[1];
}

/**
 * Write a byte to the specified register
 */
void mfrc522_write_register(mfrc522 *dev, uint8_t reg, uint8_t value) {
  uint8_t tx[2] = {MFRC522_WRITE_MSB & reg, value};
  uint8_t rx[2] = {0, 0};

  struct spi_ioc_transfer tr = {
      .tx_buf = (unsigned long) tx,
      .rx_buf = (unsigned long) rx,
      .len = 2,
      .speed_hz = SPI_SPEED,
      .bits_per_word = SPI_BITS_PER_WORD,
      .delay_usecs = 0
  };

  if (ioctl(dev->spi_fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
      perror("Error in SPI transfer");
      return EXIT_FAILURE;
  }
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
