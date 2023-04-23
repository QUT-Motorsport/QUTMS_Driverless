from argparse import ArgumentParser
from getpass import getpass
import os
import subprocess

import colorama
import yaml


def build_workspace():
    """
    Colcon build packages in workspace
    """
    print(
        colorama.Fore.GREEN,
        "Building workspace...",
        colorama.Fore.RESET,
        flush=True,
    )

    # can either build the sim or the whole workspace minus colcon_ignore packages
    parser = ArgumentParser(description="Building the workspace")
    parser.add_argument("--sim", action="store_true", help="Build the sim only")
    args = parser.parse_args()

    ws_path = os.environ["QUTMS_WS"]

    colcon_ignore_path = os.path.join(ws_path, "QUTMS_Driverless", "installation", "colcon_ignore.yaml")

    # Read yaml file with eufs repository paths
    with open(colcon_ignore_path, "r") as f:
        colcon_ignores: dict = yaml.safe_load(f.read())

    if args.sim:
        command = [
            "colcon",
            "build",
            "--symlink-install",
            "--packages-up-to",
            "eufs_launcher",
        ]
    else:
        print("Ignoring packages: ", colcon_ignores["colcon_ignore"])
        command = [
            "colcon",
            "build",
            "--symlink-install",
            "--packages-ignore",
        ] + colcon_ignores["colcon_ignore"]

    print(f"Command: {' '.join(command)}")
    process = subprocess.Popen(command, text=True, cwd=ws_path)
    process.wait()


def launch_sim():
    """
    Launch the EUFS simulator
    """
    print(
        colorama.Fore.GREEN,
        "Launching eufs_launcher...",
        colorama.Fore.RESET,
        flush=True,
    )

    command = ["ros2", "launch", "eufs_launcher", "eufs_launcher.launch.py"]
    process = subprocess.Popen(command, text=True)
    process.wait()


def pull_repos():
    """
    Pull selected repos
    """
    print(
        colorama.Fore.GREEN,
        "Pulling repos...",
        colorama.Fore.RESET,
        flush=True,
    )

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
    parser.add_argument("--all", "-a", help="Pull all repos", action="store_true")
    args = parser.parse_args()

    ws_path = os.environ["QUTMS_WS"]

    if args.all:
        args.alist = ["QUTMS_Driverless", "eufs_sim", "eufs_rviz_plugins"]

    for repo in args.alist:
        print(repo)
        repo = os.path.join(ws_path, repo)
        command = ["git", "pull"]
        process = subprocess.Popen(command, text=True, cwd=repo)
        process.wait()
