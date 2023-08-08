from qutms_cli_tools.common import Print


def helper():
    """
    Print help message
    """

    Print.green("QUTMS CLI Tools")

    Print.yellow(
        """
Usage: ws_<command> [<args>]
Workspace Commands:
    ws_pull\t\tPull selected repos
    ws_build\tBuild selected packages
    ws_launch\tLaunch groups of ROS launch files
    ws_format\tPre-commit format in repo
    ws_record\tRecord topics to ROS2 bag
Args:
    Check help for each command for specific args
        """
    )
