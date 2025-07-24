#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
DS Controller User Interface
Replaces original keyboard input with a more user-friendly interaction
"""

import rospy
import sys
import select
import termios
import tty
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Vector3

class DSUserInterface:
    def __init__(self):
        rospy.init_node('ds_user_interface')
        
        # ROS Publishers
        self.command_pub = rospy.Publisher('/unified_ds/user_command', String, queue_size=1)
        
        # ROS Subscribers - Monitor system status
        rospy.Subscriber('/unified_ds/control_phase', String, self.phase_callback)
        rospy.Subscriber('/unified_ds/energy_tank', Float64, self.energy_callback)
        rospy.Subscriber('/unified_ds/status', String, self.status_callback)
        
        # Status variables
        self.current_phase = "UNKNOWN"
        self.energy_level = 0.0
        self.is_running = False
        self.calibration_complete = False
        
        # Terminal settings
        self.old_settings = termios.tcgetattr(sys.stdin)
        
        print("=== DS Controller User Interface ===")
        print("DS System Advantages:")
        print("✓ Immediate force feedback control, no 10-second wait")
        print("✓ Automatic disturbance recovery")
        print("✓ Energy tank for system safety")
        print("✓ Parameterized configuration")
        print()
        self.print_help()
        
    def phase_callback(self, msg):
        """Control phase status callback"""
        self.current_phase = msg.data
        
    def energy_callback(self, msg):
        """Energy tank status callback"""
        self.energy_level = msg.data
        
    def status_callback(self, msg):
        """Controller status callback"""
        if msg.data == "CALIBRATION_COMPLETE":
            self.calibration_complete = True
            print("\nCalibration complete. Type 'start' and press Enter to begin motion.")
        
    def print_help(self):
        """Print help information"""
        print("\n=== DS Controller Command Help ===")
        print("Control Commands:")
        print("  start  - Begin motion")
        print("  stop   - Stop motion")
        print("  reset  - Reset force sensor bias")
        print("  info   - Display system status")
        print("  help   - Display this help")
        print("  exit   - Exit program")
        print()
        print("Real-time Status:")
        print(f"  Current Phase: {self.current_phase}")
        print(f"  Energy Level: {self.energy_level:.2f}")
        print()
        
        # Phase descriptions
        phase_descriptions = {
            "CALIBRATION": "Calibration phase - System initialization, zero velocity",
            "LINEAR_APPROACH": "Linear approach - Using LinearDS to converge to target",
            "PROBE_DESCENT": "Exploratory descent - Mixed DS (horizontal + vertical) to find surface",
            "CIRCULAR_MOTION": "Circular motion - Using CircularDS + force feedback control"
        }
        
        if self.current_phase in phase_descriptions:
            print(f"Phase Description: {phase_descriptions[self.current_phase]}")
        print()
        
    def send_command(self, command):
        """Send command to DS controller"""
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)
        
        command_names = {
            'start': 'Start Motion',
            'pause': 'Pause Motion', 
            'resume': 'Resume Motion',
            'stop': 'Stop Motion',
            'reset': 'Reset Force Sensor'
        }
        
        command_name = command_names.get(command, command)
        print(f"Command sent: {command_name}")
        
        if command == 'start':
            self.is_running = True
            print("DS System Advantage: Immediate start, no stabilization wait!")
        elif command in ['pause', 'stop']:
            self.is_running = False
            
    def print_status(self):
        """Print system status"""
        print("\n=== DS Controller Status ===")
        print(f"Control Phase: {self.current_phase}")
        print(f"Energy Level: {self.energy_level:.2f}")
        print(f"Running Status: {'Running' if self.is_running else 'Stopped'}")
        print()
        
        # Phase descriptions
        phase_descriptions = {
            "CALIBRATION": "Calibration phase - System initialization, zero velocity",
            "LINEAR_APPROACH": "Linear approach - Using LinearDS to converge to target",
            "PROBE_DESCENT": "Exploratory descent - Mixed DS (horizontal + vertical) to find surface",
            "CIRCULAR_MOTION": "Circular motion - Using CircularDS + force feedback control"
        }
        
        if self.current_phase in phase_descriptions:
            print(f"Phase Description: {phase_descriptions[self.current_phase]}")
        print()
        
    def run(self):
        """Main run loop - simplified for contact_control compatibility"""
        try:
            print("\nWaiting for force sensor calibration...")
            print("After calibration completes, type 'start' and press Enter to begin motion")
            print("Type 'help' for more commands")
            print("\nWaiting for command: ", end='', flush=True)
            
            while not rospy.is_shutdown():
                cmd = input()
                cmd = cmd.strip().lower()
                
                if cmd == 'start':
                    self.send_command('start')
                    print("Starting motion...\nWaiting for command: ", end='', flush=True)
                elif cmd == 'stop':
                    self.send_command('stop')
                    print("Stopping motion...\nWaiting for command: ", end='', flush=True)
                elif cmd == 'reset':
                    self.send_command('reset')
                    print("Resetting force sensor bias...\nWaiting for command: ", end='', flush=True)
                elif cmd == 'info':
                    self.print_status()
                    print("Waiting for command: ", end='', flush=True)
                elif cmd == 'help':
                    self.print_help()
                    print("Waiting for command: ", end='', flush=True)
                elif cmd == 'exit' or cmd == 'quit':
                    print("Exiting DS User Interface")
                    break
                else:
                    print(f"Unknown command: {cmd}")
                    self.print_help()
                    print("Waiting for command: ", end='', flush=True)
                    
        except KeyboardInterrupt:
            print("\nReceived Ctrl+C, exiting")
        except Exception as e:
            print(f"Program exception: {e}")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

def main():
    """Main function"""
    try:
        interface = DSUserInterface()
        interface.run()
    except rospy.ROSInterruptException:
        print("ROS node interrupted")
    except Exception as e:
        print(f"Startup failed: {e}")

if __name__ == '__main__':
    main() 