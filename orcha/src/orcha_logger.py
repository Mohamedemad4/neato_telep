import logging
FORMAT = "[%(asctime)s:%(levelname)s -- %(filename)s:%(lineno)s - %(funcName)s() ] %(message)s"
logging.basicConfig(filename='orcha.log',level=logging.DEBUG,format=FORMAT)
log=logging.getLogger('root')