import rclpy
from rclpy.node import Node

from enum import Enum, auto

# Messages
from robotnik_safety_msgs.msg import SafetyModeStatus
from robotnik_io_msgs.msg import InputsOutputs

from threading import Lock

RECEIVED_IO_TIMEOUT = 0.5  # Seconds

class State(Enum):
    INIT = auto()
    STANDBY = auto()
    READY = auto()
    EMERGENCY = auto()
    FAILURE = auto()
    SHUTDOWN = auto()

class RobotnikFlexisoft(Node):
    def __init__(self):
        super().__init__('robotnik_flexisoft')

        period = 0.1  # Seconds
        self.__timer = self.create_timer(period, self._control_loop)

        self._current_state = State.INIT
        self._state_callbacks = {
            State.INIT:       self.init_state,
            State.STANDBY:    self.standby_state,
            State.READY:      self.ready_state,
            State.EMERGENCY:  self.emergency_state,
            State.FAILURE:    self.failure_state,
            State.SHUTDOWN:   self.shutdown_state,
        }

        self.ros_setup()

    def _control_loop(self):
        # Call the current state's method
        self._state_callbacks[self._current_state]()

    def transition_to_state(self, new_state: State):
        # Skip if already in the desired state
        if self._current_state == new_state:
            return

        # Validate the state transition
        if new_state in self._state_callbacks:
            self._current_state = new_state
            self.get_logger().info(f'Switch to state: {new_state.name}')
        else:
            self.get_logger().error(f'Invalid state transition attempted: {new_state.name}')

    # State methods
    def init_state(self):
        self.transition_to_state(State.STANDBY)

    def standby_state(self):
        self.transition_to_state(State.READY)

    def ready_state(self):
        now = self.get_clock().now()
        with self._io_mtx:
            # Emergency if no data received
            if self._io_last_update is None or (now - self._io_last_update).nanoseconds / 1e9 > RECEIVED_IO_TIMEOUT:
                self.get_logger().warn(f'No IO data received in the last {RECEIVED_IO_TIMEOUT} seconds')
                self.transition_to_state(State.EMERGENCY)
                return

            def get_io_value(name: str):
                for io in self._io_data.digital_inputs:
                    if io.name == name:
                        return io.value

            emergency_stop = get_io_value('emergency_stop')
            print('Emergency stop:', emergency_stop)

    def emergency_state(self):
        self.get_logger().info('In emergency state')

    def failure_state(self):
        self.get_logger().info('In failure state')

    def shutdown_state(self):
        self.get_logger().info('In shutdown state')

    # General methods
    def ros_setup(self):
        # Publishers
        self._status_publisher = self.create_publisher(
            SafetyModeStatus,
            '~/status',
            10,
        )

        # Subscribers
        self._io_mtx = Lock()
        self._io_data: InputsOutputs = InputsOutputs()
        self._io_last_update = None
        def update_io_data(msg: InputsOutputs):
            self._io_last_update = self.get_clock().now()
            with self._io_mtx:
                self._io_data = msg
        self._io_subscriber = self.create_subscription(
            InputsOutputs,
            '/robot/modbus_io/io',  # '~/io',
            update_io_data,
            10,
        )

    def ros_publish(self):
        self.get_logger().info('Publishing data to ROS topics')

def main(args=None):
    rclpy.init(args=args)

    node = RobotnikFlexisoft()

    rclpy.spin(node)

    # Destroy and clean up the node
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
