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
    // Initialize the MFRC522 device
    if (mfrc522_init(&dev) != 0)
    {
        fprintf(stderr, "Failed to initialize MFRC522\n");
        return -1;
    }

    // Reset the device
    printf("Resetting MFRC522...\n");
    mfrc522_reset(&dev);

    // Read version register
    printf("Reading version register...\n");
    uint8_t version = mfrc522_read_register(&dev, VersionReg);

    // Display result
    printf("Version register value: 0x%02X\n", version);

    printf("----------------------\n");
    // Check if new card is present
    printf("Checking for new card...\n");
    while (true)
    {
        if (mfrc522_isNewCardPresent(&dev))
        {
            printf("----------------------\n");
            printf("New card detected!\n");
            // print UID
            Uid uid = {
                .uidByte = {0},
                .size = 0,
                .sak = 0
            };
            if (PICC_ReadCardSerial(&dev, &uid))
            {   

                printf("UID: ");
                for (int i = 0; i < uid.size; i++)
                {
                    printf("%02X ", uid.uidByte[i]);
                }
                printf("\n");
            }
            else
            {
                printf("Failed to read card serial\n");
            }
        }
    }
    

    // Clean up
    mfrc522_cleanup(&dev);
    printf("Done.\n");

    return 0;
}