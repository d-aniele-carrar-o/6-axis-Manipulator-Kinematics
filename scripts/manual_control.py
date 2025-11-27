#!/usr/bin/env python3
"""
Manual Robot Control - Focused Joint Movement
"""

import serial
import serial.tools.list_ports
import time

class ManualRobotControl:
    def __init__(self, port=None, baud=115200):
        # Connect to robot
        if port is None:
            ports = list(serial.tools.list_ports.comports())
            if not ports:
                raise Exception("No serial ports found!")
            print("Available ports:")
            for i, p in enumerate(ports):
                print(f" {i}: {p.device}")
            
            if len(ports) == 1:
                port = ports[0].device
            else:
                choice = int(input(f"Select port (0-{len(ports)-1}): "))
                port = ports[choice].device
        
        print(f"Connecting to {port}...")
        self.ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2)
        print("Connected!")
        
        # Current position and last command tracking
        self.current_pos = [0, 0, 0, 0, 0]  # X, Y, Z, A, B
        self.last_command = None
        
    def wait_for_response(self):
        """Wait for OK from Arduino"""
        while True:
            if self.ser.in_waiting:
                line = self.ser.readline().decode().strip()
                if "OK" in line or "READY" in line:
                    break
            time.sleep(0.05)
    
    def send_gcode(self, cmd):
        """Send G-code command"""
        self.ser.write((cmd + "\n").encode())
        self.wait_for_response()
    
    def move_joint(self, joint, steps, speed=3000):
        """Move a specific joint by given steps"""
        self.current_pos[joint-1] += steps
        
        # Send ALL joint positions to prevent unwanted movements
        cmd = f"G1 X{self.current_pos[0]} Y{self.current_pos[1]} Z{self.current_pos[2]} A{self.current_pos[3]} B{self.current_pos[4]} F{speed}"
        self.send_gcode(cmd)
        
        # Store last command for repeat
        self.last_command = (joint, steps, speed)
        
        direction = "+" if steps > 0 else "-"
        print(f"J{joint} {direction}{abs(steps)} -> Total: {self.current_pos[joint-1]}")
    
    def repeat_last(self):
        """Repeat the last movement command"""
        if self.last_command:
            joint, steps, speed = self.last_command
            self.move_joint(joint, steps, speed)
        else:
            print("No previous command to repeat")
    
    def interactive_mode(self):
        """Main control loop"""
        print("\n=== Robot Joint Control ===")
        print("Format: J<1-5> <+/-> <steps> [speed]")
        print("Examples: J1 + 500, J3 - 2000 1500")
        print("Commands: h=home, s=status, r=repeat, q=quit")
        print()
        
        self.wait_for_response()  # Wait for Arduino READY
        
        while True:
            try:
                cmd = input("> ").strip()
                
                if cmd == 'q':
                    break
                elif cmd == 'h':
                    print("Homing...")
                    self.send_gcode("G28")
                    self.current_pos = [0, 0, 0, 0, 0]
                    self.last_command = None
                    print("Homed")
                elif cmd == 's':
                    joints = ['J1', 'J2', 'J3', 'J4', 'J5']
                    for i, pos in enumerate(self.current_pos):
                        print(f"{joints[i]}: {pos}")
                elif cmd == 'r':
                    self.repeat_last()
                elif cmd.startswith('J') or cmd.startswith('j'):
                    self.parse_move_command(cmd)
                else:
                    print("Format: J<1-5> <+/-> <steps> [speed]")
                    
            except KeyboardInterrupt:
                print("\nExiting...")
                break
            except Exception as e:
                print(f"Error: {e}")
    
    def parse_move_command(self, cmd):
        """Parse movement command like 'J1 + 500' or 'J3 - 2000 1500'"""
        parts = cmd.upper().split()
        
        if len(parts) < 3:
            print("Format: J<1-5> <+/-> <steps> [speed]")
            return
        
        # Parse joint
        joint_str = parts[0]
        if not joint_str.startswith('J') or len(joint_str) != 2:
            print("Invalid joint. Use J1-J5")
            return
        
        joint = int(joint_str[1])
        if joint < 1 or joint > 5:
            print("Invalid joint. Use J1-J5")
            return
        
        # Parse direction and steps
        direction = parts[1]
        steps_str = parts[2]
        
        if direction not in ['+', '-']:
            print("Direction must be + or -")
            return
        
        steps = int(steps_str)
        if direction == '-':
            steps = -steps
        
        # Parse optional speed
        speed = 3000  # default
        if len(parts) > 3:
            speed = int(parts[3])
        
        # Execute move
        self.move_joint(joint, steps, speed)

if __name__ == "__main__":
    try:
        robot = ManualRobotControl()
        robot.interactive_mode()
    except Exception as e:
        print(f"Failed to start: {e}")
