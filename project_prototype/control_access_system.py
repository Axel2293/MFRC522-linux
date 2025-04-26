import socket
import argparse
import time
import os
import sys
import subprocess
import base64
import json
#import requests

# Define error codes
# Error code constants
ERROR_NO_ERROR = 0
ERROR_INVALID_COMMAND = 1
ERROR_ACCESS_DENIED = 2
ERROR_DEVICE_NOT_FOUND = 3
ERROR_TIMEOUT = 4
ERROR_UNKNOWN = 5
MFRC522_PROCESS_ERROR = 6
SOCKET_ERROR = 7

class ControlAccessSystem:
    def __init__(self, host='127.0.0.1', port=20077):
        """Initialize the Control Access System."""
        # Print project logo
        self.print_logo()
        print("Version 0.1\n")

        # Socket variables
        self.host = host
        self.port = port
        self.server_socket = None

        # Module process variables
        self.module = None
        self.module_pid = None
        self.module_log_fd = None
        self.module_log_path = "./mfrc522_module.log"
    
    def control_access(self):
        """Control access based on the command received."""
        # Start the MFRC522 process
        try:
            self.module_pid = self.start_mfrc522_process()
        except Exception as e:
            print(f"[-] Error starting MFRC522 process: {e}")   # Handle error
            exit(MFRC522_PROCESS_ERROR)


        # Connect to the mfrc522 process socket
        self.module_connection, self.module_add = self.server_socket.accept()
        print(f"[+] Connection from {self.module_add}")

        # Recieve the OK signal
        try:
            data = self.module_connection.recv(1024).decode("utf-8")
            if "OK" in data:
                print(f"[+] MFRC522 reader is active {data}")
            else:
                print(f"[-] Unexpected data received: {data}")
                exit(ERROR_UNKNOWN)
        except socket.error as e:
            print(f"[-] Socket error while recieving for OK: {e}")
            exit(SOCKET_ERROR)

        # Recieve the reader version
        try:
            version= self.module_connection.recv(1024).decode("utf-8")
            if version:
                print(f"[+] reader version: {version}")
        except socket.error as e:
            print(f"[-] Socket error while : {e}")
            exit(SOCKET_ERROR)
        # Main loop (Check for cards UID and face recognition)
        while True:
            uid_base64 = self.wait_till_card_uid()
            # TODO - Integrate logic to verify Face ID

        # Close the server
        self.stop_server()

        # Stop mfrc522_modue
        self.stop_mfrc522_process()
        
        

    def print_logo(self):
        """Print the project logo."""
        print(
            r"""
            .--------------------------------------------.
            | ______         ________         ______     |
            |/_____/\       /_______/\       /_____/\    |
            |\:::__\/       \::: _  \ \      \::::_\/_   |
            | \:\ \  __   ___\::(_)  \ \   ___\:\/___/\  |
            |  \:\ \/_/\ /__/\\:: __  \ \ /__/\\_::._\:\ |
            |   \:\_\ \ \\::\ \\:.\ \  \ \\::\ \ /____\:\|
            |    \_____\/ \:_\/ \__\/\__\/ \:_\/ \_____\/|
            '--------------------------------------------'
            """)
        print("Control Access System")

    def start_server(self):
        """Start the server to listen for incoming connections."""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # Create a TCP/IPv4 socket
            self.server_socket.bind((self.host, self.port)) # Bind to the host and port
            self.server_socket.listen(5)
        except socket.error as e:
            print(f"[-] Error starting server: {e}")
            exit(SOCKET_ERROR)
        print(f"[+] Server started on {self.host}:{self.port}")

    def stop_server(self):
        """Stop the server."""
        # Flush the log file
        if self.module_log_fd:
            self.module_log_fd.flush()
            self.module_log_fd.close()
            print("[*] Log file flushed and closed")
        
        if self.module_connection:
            self.module_connection.close()
            print("[*] Module connection closed")

        if self.server_socket:
            self.server_socket.close()
            print("[*] Server stopped")

        # Kill the MFRC522 process
        if hasattr(self, 'module_pid'):
            self.stop_mfrc522_process(self.module_pid)
            print("[*] MFRC522 process stopped")
        else:
            print("No MFRC522 process to stop")
        print("[*] See ya choom!\n")
        exit(0)

    # //////////////////////////// HANDLE MFRC522 PROCESS ///////////////////////
    def start_mfrc522_process(self):
        """Start the MFRC522 process."""
        
        self.module =  subprocess.Popen(
            ["./mfrc522_module", str(self.port)],
            stdout= subprocess.PIPE,
            stderr= subprocess.PIPE
        )
        print("[*] MFRC522 process started")
        time.sleep(1)

        # Get the PID of the MFRC522 process
        pid = subprocess.check_output(["pgrep", "-f", "mfrc522_module"]).strip()

        # Check if there are more than one process and kill them
        if len(pid.split()) > 1:
            print("[*] More than one MFRC522 process found, killing all...")
            for p in pid.split():
                os.kill(int(p), 9)
            exit(MFRC522_PROCESS_ERROR)
        print(f"[*] MFRC522 process PID: {pid.decode('utf-8')}")
        return pid.decode('utf-8')

    def stop_mfrc522_process(self, pid: str):
        """Stop the MFRC522 process."""
        print(f"[*] Stopping MFRC522 process with PID: {pid}")
        try:
            os.kill(int(pid), 9)  # Send SIGKILL signal
        except OSError as e:
            print(f"Error stopping MFRC522 process: {e}")
            return MFRC522_PROCESS_ERROR
        finally:
            print("[*] MFRC522 process stopped")

    # /////////////////////////// Card UID Utils ////////////////////////////
    def wait_till_card_uid(self) -> str:
        """Wait to recieve a UID through sockets and return it in base64"""
        data_serialized = self.module_connection.recv(1024).decode('utf-8')
        if data_serialized:
            """ The mfrc522 sends a JSON with the UID as a HEX String and the lenght"""
            # Parse response
            data_deserialized = json.loads(data_serialized)
            print(f"[*] Received data: {data_deserialized}")
            
            # Get uid in bytes
            uid_hex = data_deserialized.get("uid")
            uid_bytes = self.uid_hex_to_bytes(uid_hex)

            # Encode the UID to base64
            uid_base64 = self.uid_bytes_to_base64(uid_bytes)
            print(f"[*] UID in base64: {uid_base64}")
            return uid_base64
        return None

    def uid_hex_to_bytes(self, uid_hex: str) -> bytes:
        # Decode the UID from hex to bytes
        return bytes.fromhex(uid_hex)

    def uid_bytes_to_base64(self, uid_bytes: bytes) -> str:
        return base64.b64encode(uid_bytes).decode('utf-8')





def main():
    # Create an instance of the ControlAccessSystem
    control_access_system = ControlAccessSystem()

    # Start the server
    control_access_system.start_server()

    try:
        control_access_system.control_access()
    except KeyboardInterrupt:
        print("Keyboard interrupt received, stopping server...")
        control_access_system.stop_server()
    except Exception as e:
        print(f"An error occurred: {e}")
        control_access_system.stop_server()

if __name__ == "__main__":
    main()