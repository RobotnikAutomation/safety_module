import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from enum import Enum, auto

from robotnik_common_msgs.srv import SetString
from robotnik_safety_msgs.msg import SafetyModeStatus
from robotnik_io_msgs.msg import InputsOutputs, DigitalIO
from robotnik_io_msgs.srv import SetDigitalOutputArray

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

        self._laser_modes = {
            "standard": {
                "input": {
                    "laser_mode_standard": True,
                },
                "output": {
                    "laser_mode_standard_legacy_1": False,
                    "laser_mode_standard_legacy_2": False,
                },
            },
            "charging_station": {
                "input": {
                    "laser_mode_charging_station": True,
                },
                "output": {
                    "laser_mode_standard_legacy_1": True,
                    "laser_mode_standard_legacy_2": True,
                },
            },
        }

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

            def get_current_laser_mode() -> str:
                for mode, config in self._laser_modes.items():
                    matched = True
                    for input_name, expected_value in config['input'].items():
                        if get_io_value(input_name) != expected_value:
                            matched = False
                            break
                    if matched:
                        return mode
                return 'invalid'

            # Emergency and safety stop logic
            emergency_stop = not get_io_value('emergency_stop')
            safety_stop = not get_io_value('safety_stop')
            # self.get_logger().info(f'Emergency stop value: {emergency_stop}')
            # self.get_logger().info(f'Safety stop value: {safety_stop}')

            # Working mode key
            selector_mode_auto = get_io_value('selector_mode_auto')
            selector_mode_manual = get_io_value('selector_mode_manual')
            selector_mode_maintenance = get_io_value('selector_mode_maintenance')
            laser_mute = get_io_value('laser_mute')

            # Laser parser
            standby = get_io_value('standby')
            edm_ok = get_io_value('edm_ok')
            laser_ok = get_io_value('laser_ok')

            # Power
            wheels_power_enabled = get_io_value('wheels_power_enabled')

            # Check laser mode
            def set_laser_mode(mode: str):
                if mode not in self._laser_modes:
                    self.get_logger().error(f'Invalid laser mode: {mode}')
                    return

                output_array = SetDigitalOutputArray.Request()
                output_array.output = []
                for output_name, output_value in self._laser_modes[mode]['output'].items():
                    output = DigitalIO()
                    output.name = output_name
                    output.value = output_value
                    output_array.output.append(output)

                if self._set_digital_output_client is None or not self._set_digital_output_client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().error('Set digital output service not available')
                    return

                future = self._set_digital_output_client.call_async(output_array)
                rclpy.spin_until_future_complete(self, future)
                response: SetDigitalOutputArray.Response = future.result()
                if response is None:
                    self.get_logger().error('Failed to call set digital output service')
                    return
                if response.response.success:
                    self.get_logger().info(f'Successfully set laser mode outputs for mode: {mode}')
                else:
                    self.get_logger().error(f'Failed to set laser mode outputs: {response.response.message}')

            current_laser_mode = get_current_laser_mode()
            if current_laser_mode != self._desired_laser_mode:
                self.get_logger().info(f'Detected desired laser mode: {self._desired_laser_mode} different from current mode: {current_laser_mode}. Setting outputs...')
                set_laser_mode(self._desired_laser_mode)


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

        # Service client
        self._set_digital_output_callback_group = MutuallyExclusiveCallbackGroup()
        self._set_digital_output_client = self.create_client(
            SetDigitalOutputArray,
            '/robot/modbus_io/set_digital_output_array', # '~/set_digital_output_array',
            callback_group=self._set_digital_output_callback_group,
        )

        # Service server
        self._desired_laser_mode = 'standard'
        def set_laser_mode_callback(request: SetString.Request, response: SetString.Response) -> SetString.Response:
            if request.data in self._laser_modes:
                self.get_logger().info(f'Setting laser mode target to: {request.data}')
                self._desired_laser_mode = request.data
                response.response.success = True
                response.response.message = f'Succeeded desired laser mode to: {request.data}'
            else:
                response.response.success = False
                response.response.message = f'Requested laser mode "{request.data}" is not valid. Valid modes are: {list(self._laser_modes.keys())}'
                self.get_logger().error(response.response.message)
            return response
        self._set_laser_mode_service = self.create_service(
            SetString,
            '~/set_laser_mode',
            set_laser_mode_callback,
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
