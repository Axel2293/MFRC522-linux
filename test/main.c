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
bool validate_mfrc522_operations(Mfrc522 *dev); // Function to validate read/write operations
uint8_t mfrc522_get_version(Mfrc522 *dev);   // Function to get the MFRC522 version
uint8_t mfrc522_wait_for_card(Mfrc522 *dev, Socket *sock, Uid *uid); // Function to wait for a card to be present

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
int socket_receive(Socket *sock, char *buffer, size_t buffer_size); // Receive data from the master
int socket_send_ok(Socket *sock);   // Send OK message to master
int socket_send_error(Socket *sock); // Send critical error message to master
int socket_send_reader_version(Socket *sock, uint8_t version); // Send MFRC522 version to master
int socket_cleanup(Socket *socketFd);

//  Global variables
FILE *logFile = NULL; // Global log file pointer
const char *logFilePath = "./mfrc522.log"; // Path to the log file

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
    // Open log file or create it if it doesn't exist
    logFile = fopen(logFilePath, "w+");
    if (logFile == NULL)
    {
        printf("[-] Failed to open log file: %s\n", logFilePath);
        exit(ERROR_INIT);
    }

    // Check if the user provided the correct number of arguments
    if (argc != 2)
    {
        fprintf(logFile, "Usage: %s <master_port>\n", argv[0]);
        exit(ERROR_TO_FEW_OR_MANY_ARGS);
    }

    // Define main variables
    const char *MASTER_PORT = argv[1];     // Port for the master process
    const char *MASTER_HOST = "127.0.0.1"; // Host for the master process on localhost
    pid_t pid = getpid();
    Mfrc522 *dev = calloc(1, sizeof(Mfrc522));
    Socket *sock = calloc(1, sizeof(Socket));

    fprintf(logFile, "==============================\n");
    fprintf(logFile, "==== MFRC522 MODULE C.A.S ====\n");
    fprintf(logFile, "==============================\n\n");
    fflush(logFile);

    // Show PID
    fprintf(logFile, "[*] Slave PID: %d\n", pid);
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

    // Send the ok message to the master
    if (socket_send_ok(sock) != 0)
    {
        fprintf(logFile, "[-] Failed to send OK message to master\n");
        fflush(logFile);
        exit(ERROR_SOCKET_CONNECTION);
    }
    fprintf(logFile, "[+] Sent OK message to master\n");
    fflush(logFile);

    ///////////////////////////////////////////////////////////////////////////////////
    // Listen for incoming commands from the master
    ///////////////////////////////////////////////////////////////////////////////////
    char buffer[256];
    while (1)
    {
        // Receive command from master
        if (socket_receive(sock, buffer, sizeof(buffer)) != 0)
        {
            fprintf(logFile, "[-] Failed to receive command from master\n");
            fflush(logFile);
            exit(ERROR_SOCKET_CONNECTION);
        }
        fprintf(logFile, "[+] Received command from master: %s\n", buffer);
        fflush(logFile);

        // Send ok message to master
        if (socket_send_ok(sock) != 0)
        {
            fprintf(logFile, "[-] Failed to send OK message to master\n");
            fflush(logFile);
            exit(ERROR_SOCKET_CONNECTION);
        }

        if (strcmp(buffer, "0") == 0)
        {
            // Search for a card
            fprintf(logFile, "[+] Searching for a card...\n");
            fflush(logFile);
            // Wait for a card to be present
            Uid *uid = calloc(1, sizeof(Uid));
            uint8_t status = mfrc522_wait_for_card(dev, sock, uid);
            if (status != 0)
            {
                fprintf(logFile, "[-] Failed to read card UID\n");
                fflush(logFile);
                break;
            }

        }
        else if (strcmp(buffer, "1") == 0)
        {
            // Get the MFRC522 version
            uint8_t version = mfrc522_get_version(dev);
            if (version == 0x00 || version == 0xFF)
            {
                fprintf(logFile, "[-] Failed to read MFRC522 version\n");
                fflush(logFile);
                return ERROR_MFRC522_READ;
            }
            // Convert version to hex string
            char hex_version[5];
            snprintf(hex_version, sizeof(hex_version), "%02X", version);

            // Send the version to the master
            if (send(sock->socketFD, hex_version, sizeof(hex_version), strlen(hex_version)) < 0)
            {
                fprintf(logFile, "[-] Failed to send MFRC522 version to master\n");
                fflush(logFile);
                exit(ERROR_SOCKET_CONNECTION);
            }
            fprintf(logFile, "[+] MFRC522 version sent to master: %s\n", hex_version);
            fflush(logFile);
        }
        else
        {
            fprintf(logFile, "[-] Unknown command from master: %s\n", buffer);
            fflush(logFile);
            continue;
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

////////////////////////////////////////////////////////////////
int socket_init(void)
{
    int sockFd = 0;
    if ((sockFd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        fprintf(logFile, "[-] Socket creation failed\n");
        fflush(logFile);
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
        fprintf(logFile, "[-] Invalid port number: %s\n", master_port);
        fflush(logFile);
        return ERROR_SOCKET_CONNECTION;
    }

    // Convert IPv4 addresses from text to binary form
    if (inet_pton(AF_INET, master_host, &server_addr.sin_addr) <= 0)
    {
        fprintf(logFile, "[-] Invalid address: %s\n", master_host);
        fflush(logFile);
        return ERROR_SOCKET_CONNECTION;
    }

    // Connect to the server
    int result;
    if ((result = connect(sock->socketFD, (struct sockaddr *)&server_addr, sizeof(server_addr))) < 0)
    {
        fprintf(logFile, "[-] Connection failed to master: %s:%s:%d\n", master_host, master_port, result);
        fflush(logFile);
        return ERROR_SOCKET_CONNECTION;
    }

    // Send the OK message to the master
    socket_send_ok(sock);

    return EXIT_SUCCESS;
}

int socket_send_ok(Socket *sock) {
    char message[256] = "OK";
    if (send(sock->socketFD, message, sizeof(message), strlen(message)) < 0) {
        return ERROR_SOCKET_CONNECTION;
    }
    return EXIT_SUCCESS;
}

int socket_send_error(Socket *sock) {
    char message[256] = "ERROR";
    if (send(sock->socketFD, message, sizeof(message), strlen(message)) < 0) {
        return ERROR_SOCKET_CONNECTION;
    }
    return EXIT_SUCCESS;
}

int socket_receive(Socket *sock, char *buffer, size_t buffer_size) {
    ssize_t bytes_received = recv(sock->socketFD, buffer, buffer_size - 1, 0);
    if (bytes_received < 0) {
        return ERROR_SOCKET_CONNECTION;
    }
    buffer[bytes_received] = '\0'; // Null-terminate the received string
    return EXIT_SUCCESS;
}

int socket_cleanup(Socket *sock)
{
    close(sock->socketFD);
    free(sock);
    return EXIT_SUCCESS;
}

int socket_send_reader_version(Socket *sock, uint8_t version) {
    if (version == 0x00 || version == 0xFF) {
        return ERROR_MFRC522_READ;
    }
    // Convert version to hex string
    char hex_version[5];
    snprintf(hex_version, sizeof(hex_version), "%02X", version);

    // Send the version to the master
    if (send(sock->socketFD, hex_version, sizeof(hex_version), strlen(hex_version)) < 0) {
        return ERROR_SOCKET_CONNECTION;
    }
    fprintf(logFile, "[+] MFRC522 version sent to master: %s\n", hex_version);
    return EXIT_SUCCESS;
}

/////////////////////////////////////////////////////////////////
bool validate_mfrc522_operations(Mfrc522 *dev)
{
    // Check if the register operations work as expected
    uint8_t test_value = 0xAD;
    mfrc522_write_register(dev, FIFODataReg, test_value);
    fprintf(logFile, "  [+] Register operations test: %02X\n", test_value);
    fflush(logFile);
    uint8_t read_value = mfrc522_read_register(dev, FIFODataReg);
    fprintf(logFile, "  [+] Read value: %02X\n", read_value);
    fflush(logFile);
    if (read_value != test_value)
    {
        perror("    [-] Register operations failed!\n");
        return false;
    }

    // Write multiple data to FIFO
    uint8_t data[16] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                        0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10};
    fprintf(logFile, "    [+] Writing data to FIFO: \n");
    fflush(logFile);
    for (size_t i = 0; i < sizeof(data); i++)
    {
        fprintf(logFile, "0x%02X-", data[i]);
        fflush(logFile);
        mfrc522_write_register(dev, FIFODataReg, data[i]);
    }
    fprintf(logFile, "\n");
    fflush(logFile);
    // Read back the data
    uint8_t read_data[16] = {0};
    fprintf(logFile, "    [+] Reading back data from FIFO: \n");
    fflush(logFile);
    for (size_t i = 0; i < sizeof(read_data); i++)
    {
        read_data[i] = mfrc522_read_register(dev, FIFODataReg);
        fprintf(logFile, "0x%02X-", read_data[i]);
        fflush(logFile);
    }
    fprintf(logFile, "\n");
    fflush(logFile);
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

uint8_t mfrc522_get_version(Mfrc522 *dev) {
    return mfrc522_read_register(dev, VersionReg);
}

uint8_t mfrc522_wait_for_card(Mfrc522 *dev, Socket *sock, Uid *uid)
{
    // Wait for a card to be present
    while (1)
    {
        if (mfrc522_isNewCardPresent(dev))
        {
            fprintf(logFile, "[+] Card detected\n");
            fflush(logFile);
            if (PICC_ReadCardSerial(dev, uid))
            {
                // Convert UID to hex string Eg. "234A5B6C"
                char uid_hex[256] = {0};
                snprintf(uid_hex, sizeof(uid_hex), "%02X%02X%02X%02X", uid->uidByte[0], uid->uidByte[1], uid->uidByte[2], uid->uidByte[3]);

                // Send the UID to the master
                if (send(sock->socketFD, uid_hex, sizeof(uid_hex), strlen(uid_hex)) < 0)
                {
                    fprintf(logFile, "[-] Failed to send UID message to master\n");
                    fflush(logFile);
                    return ERROR_SOCKET_CONNECTION;
                }
                fprintf(logFile, "[+] Card UID: %s\n", uid_hex);
                fflush(logFile);
                return EXIT_SUCCESS;
            }
        }
        
    }
}
