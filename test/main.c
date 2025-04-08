#include <stdio.h>
#include "../include/mfrc522.h" 

int main()
{
    mfrc522 dev = {
        .spi_fd = -1,
        .gpio_chip = NULL,
        .reset_line = NULL
    };

    printf("MFRC522\n");
    printf("----------------------\n");

    // Initialize GPIO
    if (mfrc522_init_gpio(&dev) != 0)
    {
        fprintf(stderr, "Failed to initialize GPIO\n");
        return 1;
    }

    // Initialize SPI
    if (mfrc522_init_spi(&dev) != 0)
    {
        fprintf(stderr, "Failed to initialize SPI\n");
        mfrc522_cleanup(&dev);
        return 1;
    }

    // Reset the device
    printf("Resetting MFRC522...\n");
    mfrc522_reset(&dev);

    // Read version register
    printf("Reading version register...\n");
    uint8_t version = mfrc522_read_register(&dev, VersionReg);

    // Display result
    printf("Version register value: 0x%02X\n", version);

    // Clean up
    mfrc522_cleanup(&dev);
    printf("Done.\n");

    return 0;
}