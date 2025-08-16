#!/usr/bin/env python3
import os
import sys
import time
import subprocess
from pynput import keyboard

class ROSKillSwitch:
    def __init__(self):
        self.listener = None
        print("ROS Kill Switch activated")
        print("Press SPACEBAR to kill all ROS processes")
        print("Press ESC to exit without killing processes")

    def kill_ros_processes(self):
        """Kill all ROS-related processes systematically"""
        print("\nInitiating ROS shutdown sequence...")

        # Step 1: Kill all ROS nodes
        try:
            subprocess.run(['rosnode', 'kill', '--all'], timeout=5)
            time.sleep(1)  # Give nodes time to shutdown
        except:
            pass  # Continue even if some nodes don't respond

        # Step 2: Kill core ROS processes
        core_processes = [
            'roscore', 'rosmaster', 'rosout',
            'gzserver', 'gzclient', 'rviz'
        ]
        for proc in core_processes:
            os.system(f'pkill -9 -f "{proc}"')

        # Step 3: Kill all remaining ROS nodes
        os.system('pkill -9 -f "ros/.*/node"')

        # Step 4: Kill Python processes that might be ROS-related
        os.system('pkill -9 -f "ros/.*/scripts"')

        print("All ROS processes terminated")
        print("Cleanup complete. Exiting...")
        sys.exit(0)

    def on_press(self, key):
        """Handle key press events"""
        try:
            if key == keyboard.Key.space:
                self.kill_ros_processes()
            elif key == keyboard.Key.esc:
                print("\nExit without killing processes")
                sys.exit(0)
        except AttributeError:
            pass  # Ignore special keys

    def start(self):
        """Start listening for keyboard input"""
        with keyboard.Listener(on_press=self.on_press) as listener:
            self.listener = listener
            listener.join()

if __name__ == '__main__':
    # Check if we're in a ROS environment
    if 'ROS_DISTRO' not in os.environ:
        print("ERROR: Not in a ROS environment!")
        print("Source your ROS setup.bash first")
        sys.exit(1)

    # Check for pynput installation
    try:
        from pynput import keyboard
    except ImportError:
        print("ERROR: pynput library not installed!")
        print("Install with: pip install pynput")
        sys.exit(1)

    kill_switch = ROSKillSwitch()
    kill_switch.start()
