#!/usr/bin/env python3

import subprocess
import time
import psutil
import os
import signal

# Dictionary to store terminal PIDs for later termination
terminal_pids = {}

# Helper: Check if a process with a keyword is running
def is_process_running(keyword):
    for proc in psutil.process_iter(['pid', 'cmdline']):
        try:
            if keyword in ' '.join(proc.info['cmdline']):
                return True
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    return False

# Run a command in a new terminal and track PID
def run_command(name, command):
    proc = subprocess.Popen(
        ["gnome-terminal", "--", "bash", "-c", f"{command}; exec bash"],
        preexec_fn=os.setsid
    )
    terminal_pids[name] = proc.pid

# Stop a terminal by the name used
def stop_terminal(name):
    pid = terminal_pids.get(name)
    if pid:
        try:
            os.killpg(os.getpgid(pid), signal.SIGTERM)
            print(f"Closed terminal '{name}' (PID {pid})")
        except Exception as e:
            print(f"Failed to kill terminal '{name}': {e}")
    else:
        print(f"No terminal found for '{name}'")

# Kill any running rqt_image_view process
def force_kill_rqt_image_view():
    for proc in psutil.process_iter(['pid', 'cmdline']):
        try:
            if "rqt_image_view" in ' '.join(proc.info['cmdline']):
                print(f"Killing rqt_image_view (PID {proc.pid})")
                proc.terminate()
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue

# Kill the usb_cam node explicitly
def kill_usb_camera_node():
    try:
        subprocess.run(["rosnode", "kill", "/usb_cam"], check=True)
        print("usb_camera node stopped.")
    except subprocess.CalledProcessError:
        print("usb_camera node not found or already stopped.")

# Main logic
def main():
    # Start ROS core
    if not is_process_running("roscore"):
        print("Starting roscore...")
        run_command("roscore", "roscore")
        time.sleep(2)
    else:
        print("roscore is already running.")

    # Start phone agent
    if not is_process_running("phone_agent.launch"):
        print("Starting phone_agent.launch...")
        run_command("phone_agent", "roslaunch robot_project_pkg phone_agent.launch")
        time.sleep(5)
    else:
        print("phone_agent.launch is already running.")

    # Start rostopic echo
    if not is_process_running("/phone_detections"):
        print("Starting rostopic echo...")
        run_command("detection_echo", "rostopic echo /phone_detections")
    else:
        print("rostopic echo is already running.")

    # Start image viewer
    if not is_process_running("rqt_image_view"):
        print("Starting rqt_image_view...")
        run_command("rqt_image", "rqt_image_view")
    else:
        print("rqt_image_view is already running.")

    # Runtime duration
    runtime_seconds = 3
    print(f"\nRunning for {runtime_seconds} seconds before shutdown...\n")
    time.sleep(runtime_seconds)

    # Cleanup all processes
    print("\n🧹 Cleaning up...\n")
    stop_terminal("phone_agent")
    stop_terminal("detection_echo")
    stop_terminal("rqt_image")
    stop_terminal("roscore")
    force_kill_rqt_image_view()
    kill_usb_camera_node()

    print("All processes terminated.")

if __name__ == "__main__":
    main()
