#!/usr/bin/env python3

import subprocess
import time
import psutil

# Helper to check if a process with a keyword is running
def is_process_running(keyword):
    for proc in psutil.process_iter(['cmdline']):
        try:
            if keyword in ' '.join(proc.info['cmdline']):
                return True
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    return False

# Function to run a command in a new terminal tab
def run_command(command):
    subprocess.Popen(f"gnome-terminal --tab -- bash -c '{command}; exec bash'", shell=True)

def main():
    # Start roscore if not running
    if not is_process_running("roscore"):
        print("Starting roscore...")
        run_command("roscore")
        time.sleep(2)
    else:
        print("roscore is already running.")

    # Start roslaunch if not already launched
    if not is_process_running("phone_agent.launch"):
        print("Starting phone_agent.launch...")
        run_command("roslaunch robot_project_pkg phone_agent.launch")
        time.sleep(5)
    else:
        print("phone_agent.launch is already running.")

    # Start rostopic echo if not already running
    if not is_process_running("rostopic echo /phone_detections"):
        print("Starting rostopic echo...")
        run_command("rostopic echo /phone_detections")
    else:
        print("rostopic echo is already running.")

    # Start rqt_image_view if not already running
    if not is_process_running("rqt_image_view"):
        print("Starting rqt_image_view...")
        run_command("rqt_image_view")
    else:
        print("rqt_image_view is already running.")

if __name__ == "__main__":
    main()
