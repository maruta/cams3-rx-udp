import socket
import struct
import numpy as np
import cv2
from dataclasses import dataclass
from collections import deque
import time
import select

@dataclass
class InputData:
    packet_id: int
    servo: list[int]

@dataclass
class OutputData:
    len: int
    width: int
    height: int
    format: int
    camera_timestamp_sec: int
    camera_timestamp_usec: int
    last_received_packet_timestamp_sec: int
    last_received_packet_timestamp_usec: int
    last_received_packet_id: int
    image: bytes

class CamS3TX:
    """
    CamS3TX class for managing communication with the camera equipped ESP32S3 device.
    This class handles connection, image retrieval, and servo control.
    """

    def __init__(self, recv_port=12346, send_port=12345, buffer_size=100):
        self.recv_port = recv_port
        self.send_port = send_port
        self.recv_sock = None
        self.send_sock = None
        self.camera_addr = None
        self.packet_id = 0
        self.buffer_size = buffer_size

        # Ring buffer for sent packet IDs and timestamps
        self.sent_packets = deque(maxlen=buffer_size)

        # Ring buffer for RTT calculations
        self.rtt_buffer = deque(maxlen=buffer_size)

        # Ring buffer for camera timestamp differences
        self.timestamp_diff_buffer = deque(maxlen=buffer_size)

        # Store the last received camera timestamp
        self.last_camera_timestamp = None

    def connect(self):
        """Establish connection with the device."""
        self.recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.recv_sock.bind(('', self.recv_port))
        self.recv_sock.settimeout(1.5)

        self.send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        print("Waiting for device connection...")
        while not self.camera_addr:
            try:
                _, address = self.recv_sock.recvfrom(65536)
                self.camera_addr = (address[0], self.send_port)
                self.send_sock.connect(self.camera_addr)
                print(f"Connected to device at {self.camera_addr}")
            except socket.timeout:
                print("Timeout, retrying...")

    def disconnect(self):
        """Safely disconnect from the device."""
        if self.recv_sock:
            self.recv_sock.close()
        if self.send_sock:
            self.send_sock.close()
        print("Disconnected from device")

    def get_image(self):
        """Retrieve and process the latest image from the device."""
        if not self.recv_sock:
            raise ConnectionError("Not connected to device. Call connect() first.")
        
        try:
            while True:
                data, _ = self.recv_sock.recvfrom(65536)
                decoded_data = self._decode_data(data)
                self._update_buffers(decoded_data)
                ready_to_read, _, _ = select.select([self.recv_sock], [], [], 0.001)
                # doscard data if there is more data to read
                if not(ready_to_read):
                    break
            return self._process_image(decoded_data.image)
        except socket.timeout:
            print("Timeout, no data received")
            
            return None

    def send_servo_command(self, servo_values):
        """Send servo command values to the device."""
        if not self.send_sock:
            raise ConnectionError("Not connected to device. Call connect() first.")
        
        self.packet_id += 1
        input_data = InputData(packet_id=self.packet_id, servo=servo_values)
        servo_data = struct.pack('IHH', input_data.packet_id, *input_data.servo)
        self.send_sock.send(servo_data)

        # Store the sent packet ID and timestamp
        self.sent_packets.append((self.packet_id, time.time()))

    def _decode_data(self, data):
        """Decode the received data into an OutputData object."""
        len_data = struct.unpack_from('I', data, 0)[0]
        return OutputData(
            len=len_data,
            width=struct.unpack_from('I', data, 4)[0],
            height=struct.unpack_from('I', data, 8)[0],
            format=struct.unpack_from('I', data, 12)[0],
            camera_timestamp_sec=struct.unpack_from('L', data, 16)[0],
            camera_timestamp_usec=struct.unpack_from('L', data, 24)[0],
            last_received_packet_timestamp_sec=struct.unpack_from('L', data, 32)[0],
            last_received_packet_timestamp_usec=struct.unpack_from('L', data, 40)[0],
            last_received_packet_id=struct.unpack_from('I', data, 48)[0],
            image=data[52:52+len_data]
        )

    def _process_image(self, buf):
        """Process the raw image data into a numpy array."""
        img_array = np.frombuffer(buf, dtype=np.uint8)
        img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        return img

    def _update_buffers(self, decoded_data):
        """Update ring buffers with new data."""
        # Calculate RTT
        for packet_id, send_time in reversed(self.sent_packets):
            if packet_id == decoded_data.last_received_packet_id:
                rtt = time.time() - send_time
                self.rtt_buffer.append(rtt)
                break

        # Calculate camera timestamp difference
        current_camera_timestamp = (decoded_data.camera_timestamp_sec +
                                    decoded_data.camera_timestamp_usec / 1e6)
        if self.last_camera_timestamp is not None:
            timestamp_diff = current_camera_timestamp - self.last_camera_timestamp
            self.timestamp_diff_buffer.append(timestamp_diff)
        self.last_camera_timestamp = current_camera_timestamp

    def get_rtt_stat(self):
        """Get the average RTT from the buffer."""
        return sum(self.rtt_buffer) / len(self.rtt_buffer) if self.rtt_buffer else None

    def get_frame_dt(self):
        """Get the average timestamp difference from the buffer."""
        return sum(self.timestamp_diff_buffer) / len(self.timestamp_diff_buffer) if self.timestamp_diff_buffer else None

    def __enter__(self):
        """Enter method for context manager."""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Exit method for context manager."""
        self.disconnect()