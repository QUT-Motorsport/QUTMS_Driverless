from argparse import ArgumentParser
import subprocess

from common import Print


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
        Print.red("Please specify a launch group, use --help or -h for more info")
        return

    Print.green("Launching...")
    process = subprocess.Popen(command, text=True)
    try:
        process.wait()
    except KeyboardInterrupt:
        try:
            process.terminate()
        except OSError:
            pass
        process.wait()


if __name__ == "__main__":
    main()
