#!/usr/bin/env python3
"""
TurtleBot Pseudo-Hardware Emulator
===================================

Creates a virtual serial port (PTY) emulating the Arduino motor controller.
Speaks the exact TurtleBot hardware interface protocol:
  - Inbound:  VEL,<left_rad_s>,<right_rad_s>\n
  - Outbound: ENC,<left_ticks>,<right_ticks>\n at 50 Hz

Allows full ros2_control testing without physical hardware plugged in.
"""

import os
import pty
import time
import threading
import math
import argparse
import signal
import sys

class TurtlebotPseudoHardware:
    def __init__(self, port_path="/tmp/tty_turtlebot_fake", cpr=360, baud=115200):
        self.port_path = port_path
        self.cpr = cpr
        self.baud = baud
        
        self.master_fd, self.slave_fd = pty.openpty()
        self.slave_name = os.ttyname(self.slave_fd)
        
        # Link symlink
        if os.path.exists(self.port_path):
            try:
                os.remove(self.port_path)
            except OSError:
                pass
        os.symlink(self.slave_name, self.port_path)
        
        self.left_cmd = 0.0
        self.right_cmd = 0.0
        self.left_pos = 0.0
        self.right_pos = 0.0
        self.running = False
        self.last_cmd_time = time.time()
        
    def start(self):
        self.running = True
        self.tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.tx_thread.start()
        self.rx_thread.start()
        print(f"[PseudoHardware] Virtual serial bridge active:")
        print(f"  Slave PTY: {self.slave_name}")
        print(f"  Symlink:   {self.port_path}")
        print(f"  CPR:       {self.cpr}, Baud: {self.baud}")
        
    def _rx_loop(self):
        buf = ""
        while self.running:
            try:
                data = os.read(self.master_fd, 64).decode('utf-8', errors='ignore')
                if not data:
                    break
                buf += data
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip()
                    if line.startswith("VEL,"):
                        parts = line[4:].split(",")
                        if len(parts) >= 2:
                            try:
                                self.left_cmd = float(parts[0])
                                self.right_cmd = float(parts[1])
                                self.last_cmd_time = time.time()
                            except ValueError:
                                pass
                    elif line == "STOP":
                        self.left_cmd = 0.0
                        self.right_cmd = 0.0
                    elif line == "RST":
                        self.left_pos = 0.0
                        self.right_pos = 0.0
            except OSError:
                break
                
    def _tx_loop(self):
        dt = 0.02  # 50 Hz
        rad_to_counts = self.cpr / (2.0 * math.pi)
        last_time = time.time()
        
        while self.running:
            now = time.time()
            elapsed = now - last_time
            last_time = now
            
            # Watchdog: if no command received in 1.0s, zero velocity
            if now - self.last_cmd_time > 1.0:
                self.left_cmd = 0.0
                self.right_cmd = 0.0
                
            self.left_pos += self.left_cmd * elapsed
            self.right_pos += self.right_cmd * elapsed
            
            left_counts = int(round(self.left_pos * rad_to_counts))
            right_counts = int(round(self.right_pos * rad_to_counts))
            
            msg = f"ENC,{left_counts},{right_counts}\n"
            try:
                os.write(self.master_fd, msg.encode('utf-8'))
            except OSError:
                break
                
            time.sleep(dt)
            
    def stop(self):
        self.running = False
        try:
            os.close(self.master_fd)
        except OSError:
            pass
        try:
            os.close(self.slave_fd)
        except OSError:
            pass
        if os.path.islink(self.port_path):
            os.remove(self.port_path)
        print("[PseudoHardware] Bridge closed")

def main():
    parser = argparse.ArgumentParser(description="TurtleBot Pseudo-Hardware Serial Emulator")
    parser.add_argument("--port", default="/tmp/tty_turtlebot_fake", help="Symlink path for virtual serial port")
    parser.add_argument("--cpr", type=int, default=360, help="Encoder counts per revolution")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    args = parser.parse_args()
    
    bridge = TurtlebotPseudoHardware(port_path=args.port, cpr=args.cpr, baud=args.baud)
    bridge.start()
    
    def shutdown_handler(signum, frame):
        bridge.stop()
        sys.exit(0)
        
    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)
    
    print("[PseudoHardware] Press Ctrl+C to terminate.")
    while True:
        time.sleep(1)

if __name__ == "__main__":
    main()
