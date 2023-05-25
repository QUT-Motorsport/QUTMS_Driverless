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
    print(Y, "Usage: ws_<command> [<args>]")
    print(Y, "Workspace Commands:")
    print(Y, "\tws_pull\t\tPull selected repos")
    print(Y, "\tws_build\tBuild selected packages")
    print(Y, "\tws_launch\tLaunch groups of ROS launch files")
    print(Y, "\tws_format\tPre-commit format in repo")
    print(Y, "\tws_record\tRecord topics to ROS2 bag")
    print(Y, "Args:")
    print(Y, "\tCheck help for each command for specific args")
