from argparse import ArgumentParser
import subprocess

import colorama

G = colorama.Fore.GREEN
R = colorama.Fore.RED
Y = colorama.Fore.YELLOW
B = colorama.Fore.BLUE
RESET = colorama.Fore.RESET

def main():
    """
    Launch groups of ROS launch files
    """

    # launch ROS launch group
    parser = ArgumentParser(description="Launch ROS nodes")
    parser.add_argument("--sim", action="store_true", help="Launch the EUFS sim")

    args = parser.parse_args()

    if args.sim:
        command = ["ros2", "launch", "eufs_launcher", "eufs_launcher.launch.py"]

    else:
        print(
            R,
            "Please specify a launch group, use --help or -h for more info",
            RESET,
            flush=True,
        )
        return

    print(G, "Launching...", RESET, flush=True)
    process = subprocess.Popen(command, text=True)
    try:
        process.wait()
    except KeyboardInterrupt:
        try:
            process.terminate()
        except OSError:
            pass
        process.wait()
