from argparse import ArgumentParser
import os
import subprocess

import colorama
import yaml

G = colorama.Fore.GREEN
R = colorama.Fore.RED
Y = colorama.Fore.YELLOW
B = colorama.Fore.BLUE
RESET = colorama.Fore.RESET

def main():
    """
    Record ros2 bag
    """

    parser = ArgumentParser(description="Record ROS2 bag")
    parser.add_argument(
        "--name",
        "-n",
        type=str,
        help="Name of the bag to record",
        action="store",
        dest="name",
    )
    parser.add_argument(
        "--dir",
        "-d",
        type=str,
        help="Directory to save bag to",
        action="store",
        dest="dir",
    )
    parser.add_argument(
        "--config",
        "-c",
        type=str,
        help="YAML config file to load topics from (WITHOUT .yaml)",
        action="store",
        dest="yaml",
    )
    args = parser.parse_args()

    if not args.yaml:
        print(
            R,
            "Please specify a YAML config file, use --help or -h for more info",
            RESET,
            flush=True,
        )
        return

    ws_path = os.path.expanduser(os.environ["QUTMS_WS"])

    if not args.dir:
        # check if bags dir exists
        if not os.path.isdir(os.path.join(ws_path, "bags")):
            os.mkdir(os.path.join(ws_path, "bags"))

        # Default to ws_path/bags
        args.dir = os.path.join(ws_path, "bags")

    print(
        G,
        "Recording bag...",
        RESET,
        flush=True,
    )

    config_path = os.path.join(ws_path, "QUTMS_Driverless", "tools", "topics_to_record", str(args.yaml)+".yaml")

    with open(config_path, "r") as f:
        topics = yaml.safe_load(f)
        print(B, "Recording topics:", topics[args.yaml])

    command = [
        "ros2",
        "bag",
        "record",
    ]
    if args.name:
        command.append("--output")
        command.append(args.name)

    for topic in topics:
        command.append(topic)

    if args.dir:
        process = subprocess.Popen(command, text=True, cwd=args.dir)
    else:
        process = subprocess.Popen(command, text=True)
    try:
        process.wait()
    except KeyboardInterrupt:
        try:
            process.terminate()
        except OSError:
            pass
        process.wait()
