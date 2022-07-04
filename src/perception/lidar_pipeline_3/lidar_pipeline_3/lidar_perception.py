import sys

from .config import Config

def main(args=sys.argv[1:]):
    config = Config()
    config.update(args)
    
    if not config.print_logs:
        print("WARNING: --print_logs flag not specified")
        print("Running lidar_perception without printing to terminal ...")

if __name__ == '__main__':
    main(sys.argv[1:])
