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


def helper():
    """
    Print help message
    """

    print(
        G,
        "QUTMS CLI Tools",
        RESET,
        flush=True,
    )
    print(
        Y,
        "Usage: ws_<command> [<args>]\n\nCommands:\n\tbuild\t\tBuild selected packages\n\tlaunch\t\tLaunch groups of ROS launch files\n\tpull\t\tPull selected repos\n\trecord\t\tRecord ROS2 bag",
        flush=True,
    )


def build():
    """
    Colcon build selected packages
    """

    command_prefix = [
        "colcon",
        "build",
        "--symlink-install",
    ]

    # build packages listed as args
    parser = ArgumentParser(description="Building selected packages")
    parser.add_argument(
        "--select",
        "-s",
        type=str,
        nargs="*",
        help="A list of packages to build",
        default=[],
        action="store",
        dest="package",
    )
    # build up to packages listed as args
    parser.add_argument(
        "--up-to",
        "-u",
        type=str,
        nargs="*",
        help="A list of packages to build up to",
        default=[],
        action="store",
        dest="package",
    )
    parser.add_argument("--sim", action="store_true", help="Build the sim only")
    parser.add_argument(
        "--all",
        help="Build all packages not in colcon_ignore.yaml",
        action="store_true",
    )

    args = parser.parse_args()

    ws_path = os.path.expanduser(os.environ["QUTMS_WS"])
    colcon_ignore_path = os.path.join(
        ws_path,
        "QUTMS_Driverless",
        "tools",
        "colcon_ignore.yaml",
    )

    # Read yaml file with eufs repository paths
    with open(colcon_ignore_path, "r") as f:
        colcon_ignores: dict = yaml.safe_load(f.read())

    if args.select:
        command = command_prefix + ["--packages-select"] + args.select

    elif args.up_to:
        command = command_prefix + ["--packages-up-to"] + args.up_to

    elif args.sim:
        command = command_prefix + ["--packages-up-to", "eufs_launcher"]

    elif args.all:
        print(R, "Ignoring packages: ", colcon_ignores["colcon_ignore"], RESET, flush=True)
        command = command_prefix + ["--packages-ignore"] + colcon_ignores["colcon_ignore"]

    else:
        print(
            R,
            "Please specify a build group, use --help or -h for more info",
            RESET,
            flush=True,
        )
        return

    print(G, "Building packages...", RESET, flush=True)
    print(f"Command: {' '.join(command)}")
    process = subprocess.Popen(command, text=True, cwd=ws_path)
    try:
        process.wait()
    except KeyboardInterrupt:
        try:
            process.terminate()
        except OSError:
            pass
        process.wait()

def launch():
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


def pull():
    """
    Pull selected repos
    """

    parser = ArgumentParser(description="Repositories to pull")
    parser.add_argument(
        "--repos",
        "-r",
        type=str,
        nargs="*",
        help="A list of repos to pull",
        default=[],
        action="store",
        dest="alist",
    )
    parser.add_argument("--all", help="Pull all repos", action="store_true")
    args = parser.parse_args()

    ws_path = os.environ["QUTMS_WS"]

    if args.all:
        args.alist = ["QUTMS_Driverless", "eufs_sim", "eufs_rviz_plugins"]

    if not args.alist:
        print(
            R,
            "Please specify a repo, use --help or -h for more info",
            RESET,
            flush=True,
        )
        return

    print(
        G,
        "Pulling repos...",
        RESET,
        flush=True,
    )

    for repo in args.alist:
        print(repo)
        repo = os.path.join(ws_path, repo)
        command = ["git", "pull"]
        process = subprocess.Popen(command, text=True, cwd=repo)
        process.wait()
