#!/usr/bin/env python3
import signal
import sys
import subprocess
import os 
import psutil


def kill_process_by_name(process_name):
    for line in os.popen("ps ax | grep " + process_name + " | grep -v grep"):
        fields = line.split()
        pid = fields[0]
        os.kill(int(pid), signal.SIGKILL)

def signal_handler(sig, frame):
    
    kill_process_by_name('gzclient')
    print("Killed gzclient")
    
    kill_process_by_name('gzserver')
    print("Killed gzserver")
    
    kill_process_by_name('ros2')
    print("Killed ros2")
    
    subprocess.run(['ros2', 'daemon', 'stop'], check=False)

    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
print('Press Ctrl+C')
signal.pause()