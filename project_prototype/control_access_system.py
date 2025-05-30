import socket
import time
import os
import sys
import subprocess
import base64
import requests
import cv2
from dotenv import load_dotenv

class FaceRecognition:
    def capture_frame(path="./captured_frame.jpg"):
        """Capture a frame from the webcam"""
        cap = cv2.VideoCapture(0)
        ret, frame = cap.read()
        cap.release()
        if ret:
            # Save the frame to a file
            cv2.imwrite(path, frame)
            return path
        else:
            raise Exception("Failed to capture frame from cam.")

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
            "GET_READER_VERSION": 1,
            "TURN_ON_GREEN_LED": 2,
            "TURN_ON_RED_LED": 3,
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

    def mfrc522_green_led_on(self):
        """Turn on the green LED."""
        print("[*] Turning on green LED...")
        self.mfrc522_send_command(str(self.command_dict["TURN_ON_GREEN_LED"]))

    def mfrc522_red_led_on(self):
        """Turn on the red LED."""
        print("[*] Turning on red LED...")
        self.mfrc522_send_command(str(self.command_dict["TURN_ON_RED_LED"]))

    def mfrc522_close(self):
        """Close the MFRC522 module."""
        self.mfrc522_stop_process()
        self.mfrc522_close_connection()
        self.mfrc522_close_socket()
        print("[*] MFRC522 module closed")
        exit(0)

def print_logo():
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

def main():
    load_dotenv()

    try:
        print_logo()
        mf_module = MFRC522Module()
        mf_module.mfrc522_create_socket()
        mf_module.mfrc522_start_process()
        time.sleep(1)
        pid = mf_module.mfrc522_get_process_pid()
        print(f"[+] MFRC522 process PID: {pid}")
        mf_module.mfrc522_accept_connection()
        print("-----------------------------------")
        ##mf_module.mfrc522_get_reader_version()

        # Main loop to read UID and verify face ID
        while True:
            print("-----------------------------------")
            uid = mf_module.mfrc522_get_uid()
            uid = uid[0:8]      # Truncate UID to 8 characters
            #print(f"[+] Card UID: {uid}")
            # Convert UID to bytes
            uid_bytes = bytes.fromhex(str(uid))
            #print(f"[+] Card UID (bytes): {uid_bytes}")
            # Conver UID to base64
            uid_base64 = base64.b64encode(uid_bytes).decode('utf-8')
            print(f"[+] Card UID (base64): {uid_base64}")
            time.sleep(1)

            # Send UID to server with API key as a header
            server_url = os.getenv("API_URL")
            if not server_url:
                print("[-] API_URL not set in .env file")
                exit(1)

            print(f"[*] Sending UID to server: {server_url}")
            response = requests.post(
                server_url + "/api/card/validate",
                headers={"x-api-key": os.getenv("API_KEY")},
                json={"Card_ID": uid_base64}
            )

            if response.status_code != 200:
                print(f"[-] Error sending UID to server: {response.status_code}")
                print(f"[-] Response: {response.text}")
                # TODO: Send slave comand to turn the red LED on
                continue
            
            print(f"[+] Server response: {response.json()}")
            user_id = response.json().get("user")

            # Capture frame from camera
            frame_path = FaceRecognition.capture_frame()
            print(f"[+] Frame captured: {frame_path}")

            # Send frame to server for face recognition
            with open(frame_path, "rb") as image_file:
                file = {
                    "UserPhoto": (frame_path, image_file, "image/jpg")
                }
                response = requests.post(
                    server_url + "/api/face/validate",
                    headers={"x-api-key": os.getenv("API_KEY")},
                    files=file,
                    data={"User": user_id}
                )

                if response.status_code != 200:
                    print(f"[-] Error sending frame to server: {response.status_code}")
                    continue

                if not response.text:
                    print("[-] No response from server")
                    continue
                else:
                    print(f"[+] Server response: {response.json()}")

                if response.json().get("message") == "Access Granted":
                    # Send command to turn green led on
                    mf_module.mfrc522_green_led_on()
                    print("[+] Access granted!!")
                else:
                    # Send command to turn red led on
                    mf_module.mfrc522_red_led_on()
                    print("[-] Access denied!!")
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