from __future__ import absolute_import
import logging
import rospy

from rosgraph.roslogging import RosStreamHandler
from rospy.impl.rosout import RosOutHandler


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class PyLogger(object):
    '''
    Wraps a default python logger and prefix all the functions with log
    for use in the ROS eco system
    '''
    def __init__(self, logger):
        self.logger = logger

    def loginfo(self, msg, *args, **kwargs):
        self.logger.info(str(msg), *args, **kwargs)

    def logwarn(self, msg, *args, **kwargs):
        self.logger.warn(str(msg), *args, **kwargs)

    def logdebug(self, msg, *args, **kwargs):
        self.logger.debug(str(msg), *args, **kwargs)

    def logerr(self, msg, *args, **kwargs):
        self.logger.err(str(msg), *args, **kwargs)



class ColorLogger(object):
    '''
    Forward log messages wrapped with a color
    '''
    def __init__(self, logger, color=bcolors.OKBLUE):
        self.logger = logger

        if color is True:
            self.color = bcolors.OKBLUE
        else:
            self.color = color

    def loginfo(self, msg, *args, **kwargs):
        self.logger.loginfo(self.color + str(msg) + bcolors.ENDC, *args, **kwargs)

    def logwarn(self, msg, *args, **kwargs):
        self.logger.loginfo(self.color + str(msg) + bcolors.ENDC, *args, **kwargs)

    def logdebug(self, msg, *args, **kwargs):
        self.logger.loginfo(self.color + str(msg) + bcolors.ENDC, *args, **kwargs)

    def logerr(self, msg, *args, **kwargs):
        self.logger.loginfo(self.color + str(msg) + bcolors.ENDC, *args, **kwargs)



def getLogger(namespace=None, level=logging.INFO, color=True):
    '''
    Forward log messages wrapped with a color, optionally in a namespace

    @namespace: The name is potentially a period-separated hierarchical value,
                like foo.bar.baz (though it could also be just plain foo, for example)
    @return: instance of logging.Logger
    '''

    if not namespace:
        if color:
            return ColorLogger(rospy, color)
        else:
            return rospy

    logger = logging.getLogger(namespace)
    logger.setLevel(level)

    # ..for console output
    logger.addHandler(RosStreamHandler())
    # ..for network output (/rosout)
    logger.addHandler(RosOutHandler())

    if color:
        return ColorLogger(PyLogger(logger), color)
    else:
        return PyLogger(logger)
