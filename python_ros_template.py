#!/usr/bin/env python
"""

"""

# # Standart libraries:
import rospy

# # Third party libraries:

# # Standart messages and services:
from std_msgs.msg import (Bool)
from std_srvs.srv import (SetBool)

# # Third party messages and services:
from random_package.msg import (
    MessageType1,
    MessageType2,
)
from random_package.srv import (
    ServiceType1,
    ServiceType2,
)


class ClassName:
    """
    
    """

    def __init__(
        self,
        node_name,
    ):
        """
        
        """

        # # Private constants:
        self.__PRIVATE_CONSTANT = 1

        # # Public constants:
        self.NODE_NAME = node_name

        # # Private variables:
        self.__private_variable = 1

        # # Public variables:
        self.public_variable = 1

        # # Initialization and dependency status topics:
        self.__is_initialized = False
        self.__dependency_initialized = False

        self.__node_is_initialized = rospy.Publisher(
            f'{self.NODE_NAME}/is_initialized',
            Bool,
            queue_size=1,
        )

        # NOTE: Specify dependency initial False initial status.
        self.__dependency_status = {
            # 'dependency_node_name': False,
        }

        # NOTE: Specify dependency is_initialized topic (or any other topic,
        # which will be available when the dependency node is running properly).
        self.__dependency_status_topics = {
            # 'dependency_node_name':
            #     rospy.Subscriber(
            #         f'/dependency_node_name/is_initialized',
            #         Bool,
            #         self.__dependency_name_callback,
            #     ),
        }

        # # Service provider:
        rospy.Service(
            f'{self.NODE_NAME}/service_name1',
            SetBool,
            self.__service_name1_handler,
        )

        # # Service subscriber:
        self.__service = rospy.ServiceProxy(
            '/service_name2',
            ServiceType2,
        )

        # # Topic publisher:
        self.__publisher = rospy.Publisher(
            f'{self.NODE_NAME}/topic_name1',
            Bool,
            queue_size=1,
        )

        # # Topic subscriber:
        rospy.Subscriber(
            '/topic_name2',
            MessageType2,
            self.__topic_name2_callback,
        )

        # # Timers:
        rospy.Timer(rospy.Duration(1.0 / 100), self.__some_function_timer)

    # # Dependency status callbacks:
    # NOTE: each dependency topic should have a callback function, which will
    # set __dependency_status variable.
    def __dependency_name_callback(self, message):
        """Monitors <node_name> is_initialized topic.
        
        """

        # self.__dependency_status['dependency_node_name'] = message.data

    # # Service handlers:
    def __service_name1_handler(self, request):
        """

        """

        response = True

        return response

    # # Topic callbacks:
    def __topic_name2_callback(self, message):
        """

        """

    # Timer callbacks:
    def __seme_function_timer(self, event):
        """Calls <some_function> on each timer callback with 100 Hz frequency.
        
        """

        self.__some_function()

    # # Private methods:
    def __check_initialization(self):
        """Monitors required criteria and sets is_initialized variable.

        Monitors nodes' dependency status by checking if dependency's
        is_initialized topic has at most one publisher (this ensures that
        dependency node is alive and does not have any duplicates) and that it
        publishes True. If dependency's status was True, but get_num_connections
        is not equal to 1, this means that the connection is lost and emergency
        actions should be performed.

        Once all dependencies are initialized and additional criteria met, the
        nodes' is_initialized status changes to True. This status can change to
        False any time to False if some criteria are no longer met.
        
        """

        self.__dependency_initialized = True

        for key in self.__dependency_status:
            if self.__dependency_status_topics[key].get_num_connections() != 1:
                if self.__dependency_status[key]:
                    rospy.logerr(
                        (f'{self.NODE_NAME}: '
                         f'lost connection to {key}!')
                    )

                    # # Emergency actions on lost connection:
                    # NOTE (optionally): Add code, which needs to be executed if
                    # connection to any of dependencies was lost.

                self.__dependency_status[key] = False

            if not self.__dependency_status[key]:
                self.__dependency_initialized = False

        if not self.__dependency_initialized:
            waiting_for = ''
            for key in self.__dependency_status:
                if not self.__dependency_status[key]:
                    waiting_for += f'\n- waiting for {key} node...'

            rospy.logwarn_throttle(
                15,
                (
                    f'{self.NODE_NAME}:'
                    f'{waiting_for}'
                    # f'\nMake sure those dependencies are running properly!'
                ),
            )

        # NOTE (optionally): Add more initialization criterea if needed.
        if (self.__dependency_initialized):
            if not self.__is_initialized:
                rospy.loginfo(f'\033[92m{self.NODE_NAME}: ready.\033[0m',)

                self.__is_initialized = True

        else:
            if self.__is_initialized:
                # NOTE (optionally): Add code, which needs to be executed if the
                # nodes's status changes from True to False.

                pass

            self.__is_initialized = False

        self.__node_is_initialized.publish(self.__is_initialized)

    # # Public methods:
    def main_loop(self):
        """
        
        """

        self.__check_initialization()

        if not self.__is_initialized:
            return

        # NOTE: Add code (function calls), which has to be executed once the
        # node was successfully initialized.

    def node_shutdown(self):
        """
        
        """

        rospy.loginfo_once(f'{self.NODE_NAME}: node is shutting down...',)

        # NOTE: Add code, which needs to be executed on nodes' shutdown here.
        # Publishing to topics is not guaranteed, use service calls or
        # set parameters instead.

        rospy.loginfo_once(f'{self.NODE_NAME}: node has shut down.',)


def main():
    """
    
    """

    # # Default node initialization.
    # This name is replaced when a launch file is used.
    rospy.init_node(
        'node_name',
        log_level=rospy.INFO,  # rospy.DEBUG to view debug messages.
    )

    rospy.loginfo('\n\n\n\n\n')  # Add whitespaces to separate logs.

    # # ROS launch file parameters:
    node_name = rospy.get_name()

    node_frequency = rospy.get_param(
        param_name=f'{rospy.get_name()}/node_frequency',
        default=1000,
    )

    parameter1 = rospy.get_param(
        param_name=f'{node_name}/parameter1_name',
        default='some_value',
    )

    class_instance = ClassName(
        node_name=node_name,
        argument1=parameter1,
    )

    rospy.on_shutdown(class_instance.node_shutdown)
    node_rate = rospy.Rate(node_frequency)

    while not rospy.is_shutdown():
        class_instance.main_loop()
        node_rate.sleep()


if __name__ == '__main__':
    main()
