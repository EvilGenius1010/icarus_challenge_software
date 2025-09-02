"""

import socket
import time
import struct
import math
import os

class TelemetryBridge:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # cFS listens on UDP port 1234 by default
        self.addr = ("127.0.0.1", 1234)

        # For packet sequence count increment
        self.seq_count = 0

    def create_ccsds_packet(self, gyro_rates, altitude, timestamp):
        # CCSDS primary header fields
        version = 0          # 3 bits
        packet_type = 0      # telemetry = 0
        secondary_header_flag = 1  # 1 = header present
        apid = 0x0800        # example sensor data MID
        seq_flags = 3        # 3 = complete packet
        seq_count = self.seq_count & 0x3FFF  # 14 bits

        # Secondary header length + payload length = 20 bytes here (example)
        # Packet Length = (length of secondary header + payload - 1)
        # Let's say payload includes gyro (3 floats), altitude (1 float), timestamp (uint32)
        packet_data_length = (2 + 3*4 + 4 + 4) - 1  # 2 bytes secondary header assumed

        # Pack primary header (6 bytes)
        primary_header = struct.pack(
            ">H H H",
            (version << 13) | (packet_type << 12) | (secondary_header_flag << 11) | apid,
            (seq_flags << 14) | seq_count,
            packet_data_length
        )

        # Secondary header (example: timestamp uint32)
        secondary_header = struct.pack(">I", int(timestamp))

        # Payload: gyro_rates x,y,z (float32), altitude (float32)
        payload = struct.pack(">fff f", gyro_rates[0], gyro_rates[1], gyro_rates[2], altitude)

        self.seq_count += 1

        return primary_header + secondary_header + payload

    def run(self):
        initial_altitude = 280.0
        threshold = float(os.environ.get('ALTITUDE_THRESHOLD', 10.0))
        current_altitude = initial_altitude
        sim_start = time.time()

        print("Starting telemetry bridge with CCSDS packets")
        while True:
            elapsed = time.time() - sim_start
            if elapsed > 3600:  # 1 hour
                break

            # Simulate altitude decay and sensor data
            decay = 0.001 + 0.002*math.sin(elapsed*0.1)
            current_altitude -= decay

            gyro_rates = [
                0.01 * math.sin(elapsed * 0.1),
                0.005 * math.cos(elapsed * 0.1),
                0.002 * math.sin(elapsed * 0.05)
            ]

            # Create packet
            packet = self.create_ccsds_packet(gyro_rates, current_altitude, int(elapsed))
            self.sock.sendto(packet, self.addr)

            drop = initial_altitude - current_altitude
            status = "NORMAL"
            if drop > threshold:
                status = "ALTITUDE_FAIL"
            elif drop > threshold * 0.8:
                status = "WARNING"

            print(f"{int(elapsed)}s Alt: {current_altitude:.2f}km Drop: {drop:.2f}km Status: {status}")
            time.sleep(1)

if __name__ == "__main__":
    TelemetryBridge().run()
"""



import socket
import time
import struct
import math
import os

class TelemetryBridge:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # cFS listens on UDP port 1234 by default
        self.addr = ("127.0.0.1", 1234)

        # For packet sequence count increment
        self.seq_count = 0

    def create_ccsds_packet(self, gyro_rates, altitude, timestamp):
        # CCSDS primary header fields
        version = 0          # 3 bits
        packet_type = 0      # telemetry = 0
        secondary_header_flag = 1  # 1 = header present
        apid = 0x0800        # example sensor data MID
        seq_flags = 3        # 3 = complete packet
        seq_count = self.seq_count & 0x3FFF  # 14 bits

        # Secondary header length + payload length = 20 bytes here (example)
        # Packet Length = (length of secondary header + payload - 1)
        # Let's say payload includes gyro (3 floats), altitude (1 float), timestamp (uint32)
        packet_data_length = (2 + 12 + 4) - 1  # 2 bytes secondary header assumed

        # Pack primary header (6 bytes)
        primary_header = struct.pack(
            ">H H H",
            (version << 13) | (packet_type << 12) | (secondary_header_flag << 11) | apid,
            (seq_flags << 14) | seq_count,
            packet_data_length
        )

        # Secondary header (example: timestamp uint32)
        secondary_header = struct.pack(">I", int(timestamp))

        # Payload: gyro_rates x,y,z (float32), altitude (float32)
        payload = struct.pack(">fff f", gyro_rates[0], gyro_rates[1], gyro_rates[2], altitude)

        self.seq_count += 1

        return primary_header + secondary_header + payload

    def run(self):
        initial_altitude = 400.0
        threshold = float(os.environ.get('ALTITUDE_THRESHOLD', 10.0))
        current_altitude = initial_altitude
        sim_start = time.time()

        print("Starting telemetry bridge with CCSDS packets")
        while True:
            elapsed = time.time() - sim_start
            if elapsed > 3600:  # 1 hour
                break

            # Simulate altitude decay and sensor data
            decay = 0.001 + 0.002*math.sin(elapsed*0.1)
            current_altitude -= decay

            gyro_rates = [
                0.01 * math.sin(elapsed * 0.1),
                0.005 * math.cos(elapsed * 0.1),
                0.002 * math.sin(elapsed * 0.05)
            ]

            # Create packet
            packet = self.create_ccsds_packet(gyro_rates, current_altitude, int(elapsed))
            self.sock.sendto(packet, self.addr)

            drop = initial_altitude - current_altitude
            status = "NORMAL"
            if drop > threshold:
                status = "ALTITUDE_FAIL"
            elif drop > threshold * 0.8:
                status = "WARNING"

            print(f"{int(elapsed)}s Alt: {current_altitude:.2f}km Drop: {drop:.2f}km Status: {status}")
            time.sleep(1)

if __name__ == "__main__":
    TelemetryBridge().run()
