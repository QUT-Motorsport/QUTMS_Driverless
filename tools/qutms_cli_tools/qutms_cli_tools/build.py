from argparse import ArgumentParser
import os
import subprocess

from qutms_cli_tools.common import Print
import yaml


def main():
    """
    Colcon build selected packages
    """

    command_prefix = [
        "colcon",
        "build",
        "--cmake-args",
        "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON",
        "--symlink-install",
    ]

    # build packages listed as args
    parser = ArgumentParser(description="Building selected packages. Must select a build group.")
    parser.add_argument(
        "--select",
        "-s",
        type=str,
        nargs="*",
        help="A list of packages to build",
        default=[],
        action="store",
        dest="selected_package",
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
        dest="up_to_package",
    )
    parser.add_argument(
        "--sim",
        help="Build the sim only",
        action="store_true",
    )
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

    if args.selected_package:
        command = command_prefix + ["--packages-select"] + args.selected_package

    elif args.up_to_package:
        command = command_prefix + ["--packages-up-to"] + args.up_to_package

    elif args.sim:
        command = command_prefix + ["--packages-up-to", "eufs_launcher"]

    elif args.all:
        Print.blue("Ignoring packages from 'colcon_ignore.yaml':")
        command = command_prefix + ["--packages-ignore"] + colcon_ignores["colcon_ignore"]
    else:
        Print.red("Please specify a build group, use --help or -h for more info")
        return

    Print.green("Building packages...")
    Print.blue(f"Command: {' '.join(command)}")
    process = subprocess.Popen(command, text=True, cwd=ws_path)
    try:
        process.wait()
    except KeyboardInterrupt:
        try:
            process.terminate()
        except OSError:
            pass
        process.wait()
