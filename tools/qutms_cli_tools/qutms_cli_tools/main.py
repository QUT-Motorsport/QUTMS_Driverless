import colorama

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
