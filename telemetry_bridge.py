import time
import socket
import json

class TelemetryBridge:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addr = ("127.0.0.1", 1234)  # cFS UDP port

    def send(self, timestamp, rate, torque):
        packet = {
            "timestamp": timestamp,
            "gyro_rate": rate,
            "torque_cmd": torque
        }
        self.sock.sendto(json.dumps(packet).encode(), self.addr)

    def run(self):
        for i in range(100):
            timestamp = time.time()
            rate = 0.01 * i
            torque = -0.005 * i
            self.send(timestamp, rate, torque)
            time.sleep(0.1)

if __name__ == "__main__":
    TelemetryBridge().run()
