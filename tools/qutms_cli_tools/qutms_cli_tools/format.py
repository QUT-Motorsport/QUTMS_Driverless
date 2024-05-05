import os
import subprocess

from common import Print


def format_repo(repo_path_relative):
    """
    specify repo to format
    """


def main():
    """
    Format with pre-commit
    """

    ws_path = os.path.expanduser(os.environ["QUTMS_WS"])

    # TODO: make this an arg? with option for all
    repos = ["QUTMS_Driverless", "eufs_sim"]

    for repo in repos:
        repo_path_relative = os.path.join(ws_path, repo)
        Print.green(f"Formatting {repo_path_relative}...")

        command = ["pre-commit", "run", "--all-files"]
        process = subprocess.Popen(command, text=True, cwd=repo_path_relative)
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
