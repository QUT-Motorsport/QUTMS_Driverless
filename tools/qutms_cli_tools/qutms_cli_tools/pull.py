from argparse import ArgumentParser
import os
import subprocess

from qutms_cli_tools.common import Print


def main():
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
        dest="repo",
    )
    parser.add_argument("--all", help="Pull all repos", action="store_true")
    args = parser.parse_args()

    ws_path = os.path.expanduser(os.environ["QUTMS_WS"])

    if args.all:
        args.repo = ["QUTMS_Driverless", "eufs_sim", "eufs_rviz_plugins"]

    if not args.repo:
        Print.red("Please specify a repo, use --help or -h for more info")
        return

    Print.green("Pulling repos...")
    for repo in args.repo:
        print(repo)
        repo = os.path.join(ws_path, repo)
        command = ["git", "pull"]
        process = subprocess.Popen(command, text=True, cwd=repo)
        process.wait()
