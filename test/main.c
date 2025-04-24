#include <stdio.h>
#include "../include/mfrc522.h"

uint8_t validUID[4] = {0x23, 0x1A, 0x1A, 0x10};

void validateReadWrite(mfrc522 *dev) {
    // Check if the register operations work
    printf("////////////////////////////////\n");
    printf("[+] Testing register operations...\n");
    uint8_t test_value = 0xAD;
    mfrc522_write_register(dev, FIFODataReg, test_value);
    uint8_t read_value = mfrc522_read_register(dev, FIFODataReg);
    if (read_value != test_value)
    {
        perror("    [*] Register operations failed!\n");
        exit(EXIT_FAILURE);
    }
    // Write multiple data to FIFO
    uint8_t data[16] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                        0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10};
    for (size_t i = 0; i < sizeof(data); i++)
    {
        mfrc522_write_register(dev, FIFODataReg, data[i]);
    }
    // Read back the data
    uint8_t read_data[16] = {0};
    for (size_t i = 0; i < sizeof(read_data); i++)
    {
        read_data[i] = mfrc522_read_register(dev, FIFODataReg);
    }
    //Print the read data
    printf("[+] Read data from FIFO:\n");
    if (memcmp(data, read_data, 16) != 0) 
    {
        perror("    [*] Multiple read/write operation failed\n");
        exit(EXIT_FAILURE);
    }
    printf("[+] All operation were succesful :)\n");
    printf("////////////////////////////////\n");
}

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

    // Read version register
    printf("Reading version register...\n");
    uint8_t version = mfrc522_read_register(&dev, VersionReg);

    // Display result
    printf("Version register value: 0x%02X\n", version);


    validateReadWrite(&dev);

    printf("----------------------\n");
    // Check if new card is present
    printf("Checking for new card...\n");
    while (true)
    {
        sleep(3); // Sleep for 1 second
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
                printf("----------------------\n");
                printf("UID: ");
                for (int i = 0; i < uid.size; i++)
                {
                    printf("%02X ", uid.uidByte[i]);
                }
                printf("\n");
            }
            else
            {   
                // Print UID data
                printf("UID: %02X ", uid.uidByte[0]);
                for (int i = 1; i < uid.size; i++)
                {
                    printf("%02X ", uid.uidByte[i]);
                }
                printf("\n");
                // Print SAK
                printf("SAK: %02X\n", uid.sak);
                // Print UID size
                printf("UID size: %d\n", uid.size);
                printf("Failed to read card serial\n");
            }
        }
    }
    

    // Clean up
    mfrc522_cleanup(&dev);
    printf("Done.\n");

    return 0;
}