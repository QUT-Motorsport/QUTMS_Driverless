import colorama

G = colorama.Fore.GREEN
R = colorama.Fore.RED
Y = colorama.Fore.YELLOW
B = colorama.Fore.BLUE
RESET = colorama.Fore.RESET


def print_with_col(col, msg):
    print(
        col,
        msg,
        RESET,
        flush=True,
    )


class Print:
    def green(msg):
        print_with_col(G, msg)

    def red(msg):
        print_with_col(R, msg)

    def yellow(msg):
        print_with_col(Y, msg)

    def blue(msg):
        print_with_col(B, msg)
