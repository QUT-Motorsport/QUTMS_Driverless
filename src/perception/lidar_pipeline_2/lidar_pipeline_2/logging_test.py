import logging
import pathlib
path = pathlib.Path(__file__).parent.resolve()

logging.basicConfig(filename=f'{path}/example.log', filemode='w', encoding='utf-8', level=logging.DEBUG)
logging.debug('This message should go to the log file')
logging.info('So should this')
logging.warning('And this, too')
logging.error('And non-ASCII stuff, too, like Øresund and Malmö')