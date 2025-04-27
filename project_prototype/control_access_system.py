import socket
import argparse
import time
import os
import sys
import subprocess
import base64
import json
import requests

class MFRC522Module:
    def __init__(self, port=20077, address='127.0.0.1'):
        """Initialize the MFRC522 module."""
        self.module = None
        self.module_pid = None
        self.module_log_fd = None
        self.socket = None
        self.socket_address = address
        self.socket_port = port
        self.module_bin_path = "./mfrc522_module"
        self.module_log_path = "./mfrc522_module.log"

        self.command_dict = {
            "GET_UID": 0,
            "GET_READER_VERSION": 1
        }

        self.status_dict = {
            "OK": 0,
            "ERROR": 1,
            "TIMEOUT": 2,
            "NO_CARD": 3,
        }

    def mfrc522_start_process(self):
        """Start the MFRC522 process."""
        # Start the MFRC522 process
        self.module = subprocess.Popen(
            ["./mfrc522_module", str(self.socket_port)]
            # stdout=subprocess.PIPE,
            # stderr=subprocess.PIPE,
        )
        print("[*] MFRC522 process started")
        time.sleep(1)

    def mfrc522_stop_process(self):
        """Stop the MFRC522 process."""
        if self.module:
            self.module.terminate()
            self.module.wait()
            print("[*] MFRC522 process stopped")
        else:
            print("No MFRC522 process to stop")
    
    def mfrc522_get_process_pid(self):
        """Get the PID of the MFRC522 process."""
        try:
            pid = subprocess.check_output(["pgrep", "-f", "mfrc522_module"]).strip()
            return pid.decode('utf-8')
        except subprocess.CalledProcessError as e:
            print(f"Error getting MFRC522 process PID: {e}")
            return None

    def mfrc522_create_socket(self):
        """Create a socket for the MFRC522 process."""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.bind((self.socket_address, self.socket_port))
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.listen(5)
            print(f"[*] Master Socket created and listening on {self.socket_address}:{self.socket_port}")
        except socket.error as e:
            print(f"[-] Error creating socket: {e}")
            return None
        return self.socket 

    def mfrc522_accept_connection(self):
        """Accept a connection from the MFRC522 process."""
        try:
            self.socket_connection, self.socket_address = self.socket.accept()

            # self.socket.settimeout(5)  # Set a timeout for the connection
            status = self.mfrc522_receive_data()
            if 'OK' in status:
                print(f"[+] MFRC522 reader is active {status}")
            print(f"[+] Connection from {self.socket_address}")
        except socket.error as e:
            print(f"[-] Error accepting connection: {e}")
            return None
        return self.socket
    
    def mfrc522_receive_data(self):
        """Receive data from the MFRC522 process."""
        try:
            data = self.socket_connection.recv(1024).decode("utf-8")
            if data:
                print(f"[+] Received data: {data}")
                return data
            else:
                return None
        except socket.error as e:
            print(f"[-] Socket error while receiving data: {e}")
            return None
    
    def mfrc522_send_data(self, data):
        """Send data to the MFRC522 process."""
        try:
            self.socket_connection.sendall(data.encode("utf-8"))
            print(f"[+] Sent data: {data}")
        except socket.error as e:
            print(f"[-] Socket error while sending data: {e}")
            return None
        return True

    def mfrc522_close_connection(self):
        """Close the socket connection."""
        if self.socket_connection:
            self.socket_connection.close()
            print("[*] Socket slave connection closed")
        else:
            print("No socket connection to close")

    def mfrc522_close_socket(self):
        """Close the socket connection."""
        if self.socket:
            self.socket.close()
            self.socket.close()
            print("[*] Socket connection closed")
        else:
            print("No socket connection to close")
    
    def mfrc522_send_command(self, command):
        """Send a command to the MFRC522 process."""
        try:
            self.socket_connection.sendall(command.encode("utf-8"))
            print(f"[+] Sent command: {command}")
            if "OK" in self.mfrc522_receive_data():
                print(f"[+] Command sent successfully: {command}")
            else:
                print(f"[-] Command failed: {command}")
                return None
            print(f"[+] Sent command: {command}")
        except socket.error as e:
            print(f"[-] Socket error while sending command: {e}")
            return None
        return True

    def mfrc522_get_reader_version(self):
        """Get the reader version."""
        print("[*] Getting reader version...")
        self.mfrc522_send_command(str(self.command_dict["GET_READER_VERSION"]))
        version = self.mfrc522_receive_data()
        if version:
            print(f"[+] Reader version: {version}")
            return version
        else:
            print("[-] Error getting reader version")
            return None

    def mfrc522_get_uid(self):
        """Get the UID of the card."""
        print("[*] Getting UID...")
        self.mfrc522_send_command(str(self.command_dict["GET_UID"]))
        uid = self.mfrc522_receive_data()
        if uid :
            return uid
        else:
            return None

    def mfrc522_close(self):
        """Close the MFRC522 module."""
        self.mfrc522_stop_process()
        self.mfrc522_close_connection()
        self.mfrc522_close_socket()
        print("[*] MFRC522 module closed")
        exit(0)

def main():
    try:
        mf_module = MFRC522Module()
        mf_module.mfrc522_create_socket()
        mf_module.mfrc522_start_process()
        time.sleep(1)
        pid = mf_module.mfrc522_get_process_pid()
        print(f"[+] MFRC522 process PID: {pid}")
        mf_module.mfrc522_accept_connection()
        print("-----------------------------------")
        mf_module.mfrc522_get_reader_version()
        while True:
            print("-----------------------------------")
            uid = mf_module.mfrc522_get_uid()
            if uid:
                print(f"[+] UID: {uid}")
            else:
                print("[-] Error getting UID")
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[*] Exiting...")
        mf_module.mfrc522_close()
        exit(0)
    except Exception as e:
        print(f"[-] Error: {e}")
        mf_module.mfrc522_close()
        exit(1)
    finally:
        mf_module.mfrc522_close()
        print("[*] MFRC522 module closed")
        exit(0)
    

if __name__ == "__main__":
    main()