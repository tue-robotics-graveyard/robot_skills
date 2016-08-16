# ROS
import rospy
import actionlib

PENDING_CONNECTIONS = {}


def create_simple_action_client(name, action_type):
    """ Creates a simple actionlib client and waits for the action server

    Args:
        name: string with the name of the action in the correct namespace
        action_type: action type of this action

    Returns: the action client
    """
    ac = actionlib.SimpleActionClient(name, action_type)
    _add_connection(name, ac)
    return ac


def create_service_client(name, srv_type):
    """ Creates a service client and waits for the server

    Args:
        name: string with the name of the service in the correct namespace
        srv_type: service type

    Returns: the service client

    """
    srv = rospy.ServiceProxy(name, srv_type)
    _add_connection(name, srv)
    return srv


def _add_connection(name, connection):
    """ Adds a connection to the (global) dict with connections that is used when initializing the robot object.

    Args:
        name: name of the connection
        connection: connection to add

    Returns:

    """
    global PENDING_CONNECTIONS
    PENDING_CONNECTIONS[name] = connection


def wait_for_connections(timeout):
    """ Waits for the connections until they are connected

    Args:
        timeout: timeout in seconds

    Returns: bool indicating whether all connections are connected

    """
    global PENDING_CONNECTIONS
    start = rospy.Time.now()
    t = rospy.Duration(timeout)
    r = rospy.Rate(20)

    # Loop until the timeout
    while (rospy.Time.now() - start) < t:

        # If everything is connected: return True
        if len(PENDING_CONNECTIONS) == 0:
            return True

        # Check all connections
        for k, v in PENDING_CONNECTIONS.iteritems():
            # print "Checking {}".format(k)
            connected = False

            # Check actionlib connection
            if isinstance(v, actionlib.SimpleActionClient):
                connected = v.wait_for_server(rospy.Duration(0.01))
            elif isinstance(v, rospy.ServiceProxy):
                # Check service connection
                # Need to use try-except in case of service since this throws an exception if not connected.
                try:
                    v.wait_for_service(timeout=0.01)
                    connected = True
                except rospy.ROSException:
                    connected = False
            else:
                rospy.logerr("Don't know what to do with a {}".format(type(v)))

            # If connected, remove from the list
            if connected:
                rospy.loginfo("Connected to {}".format(k))
                PENDING_CONNECTIONS = {name: connection for name, connection in PENDING_CONNECTIONS.iteritems() if name != k}

        r.sleep()

    for k, v in PENDING_CONNECTIONS.iteritems():
        rospy.logerr("{} not connected timely".format(k))
    return False
