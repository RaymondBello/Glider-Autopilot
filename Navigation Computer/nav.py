import os
import logging
import logging.config

# Set up environment variables
os.environ["NAV_REPO_PATH"] = os.path.dirname(os.path.realpath(__file__))
NAV_REPO_PATH = os.environ["NAV_REPO_PATH"]

# Setup nav logger  
logging.config.fileConfig(f'{NAV_REPO_PATH}/config/logging.conf')
nav = logging.getLogger('NavigationSystem')


def main():
    nav.debug(f'debug tyryemessage')
    nav.info('info message')
    nav.warning('warn message')
    nav.error('error message')
    nav.critical('critical message')
    

if __name__ == '__main__':
    main()