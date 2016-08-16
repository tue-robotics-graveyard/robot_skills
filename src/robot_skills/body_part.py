class BodyPart(object):
    """ Base class for robot bodyparts.
    """
    def __init__(self, robot_name, tf_listener):
        """ Constructor

        Args:
            robot_name: string with robot name
            tf_listener: tf listener
        """
        self.robot_name = robot_name
        self.tf_listener = tf_listener
