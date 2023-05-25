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
    Format with pre-commit
    """

    print(
        G,
        "Formatting...",
        RESET,
        flush=True,
    )

    ws_path = os.path.expanduser(os.environ["QUTMS_WS"])
    repo = os.path.join(ws_path, "QUTMS_Driverless")

    command = ["pre-commit", "run", "--all-files"]

    process = subprocess.Popen(command, text=True, cwd=repo)
    try:
        process.wait()
    except KeyboardInterrupt:
        try:
            process.terminate()
        except OSError:
            pass
        process.wait()
