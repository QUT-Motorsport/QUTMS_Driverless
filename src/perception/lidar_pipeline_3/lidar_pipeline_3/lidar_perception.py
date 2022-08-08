import sys
import logging

from .config import Config

def main(args=sys.argv[1:]):
    config: Config = Config()
    config.update(args)

    # Setup logging
    logger = logging.getLogger(__name__)
    logging.basicConfig(filename=f'{config.runtime_dir}/output.log',
                        filemode='w',
                        format='%(asctime)s | %(levelname)s: %(message)s',
                        datefmt='%I:%M:%S %p',
                        # encoding='utf-8',
                        level=config.numeric_loglevel)
    
    if not config.print_logs:
        print("WARNING: --print_logs flag not specified")
        print("Running lidar_perception without printing to terminal ...")
    else:
        stdout_handler = logging.StreamHandler(sys.stdout)
        logger.addHandler(stdout_handler)
    
    logger.info(f'lidar_pipeline_3 initialised at {config.datetimestamp}')
    logger.info(f'args = {args}')

if __name__ == '__main__':
    main(sys.argv[1:])
