class Logger:
    """Simple logger class"""
    def error(self, msg):
        print('ERROR: ', msg)
    def debug(self, msg):
        print('DEBUG: ', msg)
    def info(self, msg):
        print('INFO: ', msg)
    def warn(self, msg):
        print('WARN: ', msg)
