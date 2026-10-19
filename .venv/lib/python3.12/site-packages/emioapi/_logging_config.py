import logging
# based on https://docs.python.org/3/howto/logging-cookbook.html

logger = logging.getLogger('emioapi_logger')
logger.setLevel(logging.INFO)  # Log everything (DEBUG level or higher)

# Prevent multiple handlers if the logger is configured multiple times
# if not logger.handlers:
# Create a console handler to log to the console
ch = logging.StreamHandler()
ch.setLevel(logging.INFO)

# Define log format
FORMAT = "[%(levelname)s]\t[%(filename)s:%(lineno)s - %(funcName)s() ] %(message)s"
formatter = logging.Formatter(FORMAT)
ch.setFormatter(formatter)

logger.addHandler(ch)