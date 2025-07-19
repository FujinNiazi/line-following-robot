#!/usr/bin/env python3

import subprocess

# Define user options and corresponding launch files
launch_options = {
    "1": {
        "label": "🏁 Launch Spa track with black line",
        "launch_file": "line_following_spa.launch.py"
    },
    "2": {
        "label": "🏁 Launch monza track with blue line",
        "launch_file": "line_following_monza.launch.py"
    }
}

# Prompt user for input
print("Which track would you like to launch?\n")
for key, opt in launch_options.items():
    print(f"{key}: {opt['label']}")

choice = input("\nEnter 1 or 2: ").strip()

if choice not in launch_options:
    print("❌ Invalid choice. Exiting.")
    exit(1)

launch_file = launch_options[choice]["launch_file"]
print(f"\n🚀 Running: {launch_file}\n")

# Run ros2 launch
subprocess.run([
    "ros2", "launch", "line_follower", launch_file
])