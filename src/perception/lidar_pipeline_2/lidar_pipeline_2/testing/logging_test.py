import logging
import pathlib
import getopt
import sys

def main(args=None):
    path = pathlib.Path(__file__).parent.resolve()

    opts, arg = getopt.getopt(sys.argv[1:], str(), ['log='])
    
    loglevel = 'INFO'
    
    print(opts)
    for opt, arg in opts:
        if opt == '--log':
            loglevel = arg

    numeric_level = getattr(logging, loglevel.upper(), None)
    if not isinstance(numeric_level, int):
        raise ValueError('Invalid log level: %s' % loglevel)
    print(numeric_level)
    logging.basicConfig(filename=f'{path}/example.log', filemode='w', encoding='utf-8', level=logging.DEBUG)
    logging.debug('This message should go to the log file')
    logging.info('So should this')
    logging.warning('And this, too')
    logging.error('And non-ASCII stuff, too, like Øresund and Malmö')

if __name__ == '__main__':
    main()