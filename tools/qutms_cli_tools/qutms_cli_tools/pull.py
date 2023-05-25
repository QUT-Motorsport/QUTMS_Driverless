from argparse import ArgumentParser
import os
import subprocess

import colorama

G = colorama.Fore.GREEN
R = colorama.Fore.RED
Y = colorama.Fore.YELLOW
B = colorama.Fore.BLUE
RESET = colorama.Fore.RESET

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

    for repo in args.repo:
        print(repo)
        repo = os.path.join(ws_path, repo)
        command = ["git", "pull"]
        process = subprocess.Popen(command, text=True, cwd=repo)
        process.wait()
