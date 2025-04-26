#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include "../include/mfrc522.h" //  Include the mfrc522 driver header file

// Define error codes
typedef enum
{
    ERROR_INIT = 1, // Main errors
    ERROR_TO_FEW_OR_MANY_ARGS,
    ERROR_SOCKET_INIT, // Socket errors
    ERROR_SOCKET_CONNECTION,
    ERROR_SOCKET_CLEANUP,
    ERROR_PICC_UID, // MFRC522 reader errors
    ERROR_MFRC522_INIT,
    ERROR_MFRC522_READ,
    ERROR_MFRC522_WRITE,
    ERROR_MFRC522_TEST
} StatusCodes;

//////////////////////////////////////////////////////
// Type definitions
//////////////////////////////////////////////////////
typedef struct
{
    char *logsFile;  // Path to the log file
    int *logsFileFd; // File descriptor for the log file
} SlaveConfig;

typedef struct
{
    int socketFD;
} Socket;

//////////////////////////////////////////////////////
// Module utils
//////////////////////////////////////////////////////
void print_to_log(FILE *log_file, const char *message);        // Print the given string to the logs file descriptor
void free_resources(Mfrc522 *dev, Socket *sock, FILE *logFile);     // Free up resources

//////////////////////////////////////////////////////
// Module style
//////////////////////////////////////////////////////

//////////////////////////////////////////////////////
// Functions to use te mfrc522 reader  
//////////////////////////////////////////////////////
uint8_t get_mfrc522_version(Mfrc522 *dev);

//////////////////////////////////////////////////////
//  Functions to test/validate the MFRC522 module
//////////////////////////////////////////////////////
bool validate_mfrc522_operations(Mfrc522 *dev); // Function to validate read/write operations
bool validate_mfrc522_version(Mfrc522 *dev);    // Function to print the MFRC522 reader version

//////////////////////////////////////////////////////
//  Socket communication functions
//////////////////////////////////////////////////////
int socket_init(void);
int socket_connect_master(Socket *sock, const char *master_host, const char *master_port);
int socket_send_critial_error(Socket *sock); // Send critical error message to master
int socket_cleanup(Socket *socketFd);

//=======================================================================================================================================================================//
/**
 * Main logic for the mfrc522_module
 *  - Socket Workflow
 *      - Signal OK to master(init done)
 *      - Send mfrc522 version to master
 *      - Start checking for proximity cards. Send the UID if detected (String-HEX format).
 */
int main(int argc, char *argv[])
{
    // Check if the user provided the correct number of arguments
    if (argc != 2)
    {
        fprintf(stderr, "Usage: %s <master_port>\n", argv[0]);
        exit(ERROR_TO_FEW_OR_MANY_ARGS);
    }

    // Define main variables
    const char *MASTER_PORT = argv[1];     // Port for the master process
    const char *MASTER_HOST = "127.0.0.1"; // Host for the master process on localhost
    pid_t pid = getpid();
    Mfrc522 *dev = calloc(1, sizeof(Mfrc522));
    Socket *sock = calloc(1, sizeof(Socket));

    // Open log file and override the default log file
    FILE *logFile = fopen("mfrc_module_logs.txt", "w");
    if (logFile == NULL)
    {
        fprintf(logFile, "[-] **Failed to open log file\n");
        fflush(logFile);
        exit(ERROR_INIT);
    }
    fprintf(logFile, "==============================\n");
    fprintf(logFile, "==== MFRC522 MODULE C.A.S ====\n");
    fprintf(logFile, "==============================\n\n");
    fflush(logFile);

    // Show PID
    fprintf(logFile, "[*] PID: %d\n", pid);
    fflush(logFile);

    // Initialize the MFRC522 device
    if (mfrc522_init(dev) != 0)
    {
        fprintf(logFile, "[-] Failed to initialize MFRC522\n");
        fflush(logFile);
        exit(ERROR_MFRC522_INIT);
    }
    fprintf(logFile, "[+] MFRC522 initialized\n");
    fflush(logFile);

    // Validate the MFRC522 operations
    if (!validate_mfrc522_operations(dev))
    {
        fprintf(logFile, "[-] Failed to validate MFRC522 operations\n");
        fflush(logFile);
        exit(ERROR_MFRC522_TEST);
    }

    // Validate the MFRC522 version
    if (!validate_mfrc522_version(dev))
    {
        fprintf(logFile, "[-] Failed to validate MFRC522 version\n");
        fflush(logFile);
        exit(ERROR_MFRC522_TEST);
    }
    fprintf(logFile, "[+] MFRC522 checks: OK\n");
    fflush(logFile);

    // Open socket and connect to master
    if ((sock->socketFD = socket_init()) < 0)
    {
        fprintf(logFile, "[-] Failed to initialize socket\n");
        fflush(logFile);
        exit(ERROR_SOCKET_INIT);
    }
    fprintf(logFile, "[+] Socket initialized\n");
    fflush(logFile);

    // Connect to the master process
    if (socket_connect_master(sock, MASTER_HOST, MASTER_PORT) != 0)
    {
        fprintf(logFile, "[-] Failed to connect to master\n");
        fflush(logFile);
        exit(ERROR_SOCKET_CONNECTION);
    }
    fprintf(logFile, "[+] Connected to master process\n");
    fflush(logFile);

    // Send the OK message to the master
    char *ok_message = "OK Xd"; // This means where starting to check for cards
    if (send(sock->socketFD, ok_message, strlen(ok_message), 0) < 0)
    {
        fprintf(logFile, "[-] Failed to send OK message to master\n");
        fflush(logFile);
        exit(ERROR_SOCKET_CONNECTION);
    }

    // Get reader version
    uint8_t version = get_mfrc522_version(dev);
    if (version != 0x00 && version != 0xFF) {
        char data[64];
        snprintf(data, sizeof(data), "%d", version);
        if (send(sock->socketFD, data, strlen(data), 0) < 0) {
            fprintf(logFile, "[-] Failed to send reader version\n");
            fflush(logFile);
            
            // Send critical_error message to master TODO
            // if (socket_send_critial_error(sock))
            exit(ERROR_PICC_UID);
        }
    }

    // Check if new card is present
    while (true)
    {
        sleep(0.1);
        if (mfrc522_isNewCardPresent(dev))
        {
            Uid uid = {// Structure to hold the UID
                       .uidByte = {0},
                       .size = 0,
                       .sak = 0};
            fprintf(logFile, "[+] Card detected\n");
            fflush(logFile);

            if (PICC_ReadCardSerial(dev, &uid))
            {
                char message[256];
                snprintf(message, sizeof(message), "{\"uid\": \"");
                for (int i = 0; i < uid.size; i++)
                {
                    snprintf(message + strlen(message), sizeof(message) - strlen(message), "%02X", uid.uidByte[i]);
                }
                snprintf(message + strlen(message), sizeof(message) - strlen(message), "\", \"length\": %d}", uid.size);
                // Send the message to the master
                if (send(sock->socketFD, message, strlen(message), 0) < 0)
                {
                    fprintf(logFile, "[-] Failed to send UID message to master\n");
                    fflush(logFile);
                    exit(ERROR_SOCKET_CONNECTION);
                }
                fprintf(logFile, "[+] Card UID: %s\n", message);
                fflush(logFile);
            }
            else
            {
                fprintf(logFile, "[-] Failed to read card serial\n");
                fflush(logFile);
                // Send error message to master
                char *error_message = "Error reading card UID";
                if (send(sock->socketFD, error_message, strlen(error_message), 0) < 0)
                {
                    fprintf(logFile, "[-] Failed to send error message to master\n");
                    fflush(logFile);
                    exit(ERROR_SOCKET_CONNECTION);
                }
            }
        }
    }


    free_resources(dev, sock, logFile);
    exit(EXIT_SUCCESS);
}

void free_resources(Mfrc522 *dev, Socket *sock, FILE *logFile) {
    // Close the log file
    fclose(logFile);
    // Clean up
    mfrc522_cleanup(dev);
    // Close the socket
    socket_cleanup(sock);

    free(dev);
}

int socket_init(void)
{
    int sockFd = 0;
    if ((sockFd = socket(AF_INET, SOCK_STREAM, 0)) == -1)
    {
        return ERROR_SOCKET_INIT;
    }
    return sockFd;
}

int socket_connect_master(Socket *sock, const char *master_host, const char *master_port)
{
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(atoi(master_port));

    if (server_addr.sin_port == 0)
    {
        return ERROR_SOCKET_CONNECTION;
    }

    // Convert IPv4 addresses from text to binary form
    if (inet_pton(AF_INET, master_host, &server_addr.sin_addr) <= 0)
    {
        return ERROR_SOCKET_CONNECTION;
    }

    // Connect to the server
    if (connect(sock->socketFD, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        return ERROR_SOCKET_CONNECTION;
    }
    return EXIT_SUCCESS;
}

int socket_send_critial_error(Socket *sock) {
    char message[256] = "CRITICAL_ERROR";
    if (send(sock->socketFD, message, sizeof(message), strlen(message)) < 0) {
        return ERROR_SOCKET_CONNECTION;
    }
    return EXIT_SUCCESS;
}

int socket_cleanup(Socket *sock)
{
    close(sock->socketFD);
    free(sock);
    return EXIT_SUCCESS;
}

bool validate_mfrc522_operations(Mfrc522 *dev)
{
    // Check if the register operations work as expected
    uint8_t test_value = 0xAD;
    mfrc522_write_register(dev, FIFODataReg, test_value);
    uint8_t read_value = mfrc522_read_register(dev, FIFODataReg);
    if (read_value != test_value)
    {
        perror("    [-] Register operations failed!\n");
        return false;
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
    // Check read data is the same as the data we wrote
    if (memcmp(data, read_data, 16) != 0)
    {
        perror("    [*] Multiple read/write operation failed\n");
        return false;
    }
    return true;
}

bool validate_mfrc522_version(Mfrc522 *dev)
{
    // Read version register
    uint8_t version = mfrc522_read_register(dev, VersionReg);

    if (version == 0x00 || version == 0xFF)
    {
        perror("    [-] Failed to read version register\n");
        return false;
    }
    else if (version != 0xB2)
        printf("    [+] MFRC522 version: %02X\n", version);
    return true;
}

uint8_t get_mfrc522_version(Mfrc522 *dev) {
    return mfrc522_read_register(dev, VersionReg);
}
