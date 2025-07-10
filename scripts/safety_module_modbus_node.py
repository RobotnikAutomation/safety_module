#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException

from enum import Enum, auto

from std_msgs.msg import Bool
from robotnik_common_msgs.srv import SetString
from robotnik_safety_msgs.msg import SafetyModeStatus, LaserStatus
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

class ModbusIOData():
    def __init__(self):
        self.data = InputsOutputs()
        self.timestamp: Time | None = None

    def update(self, timestamp: Time, msg: InputsOutputs):
        self.timestamp = timestamp
        self.data = msg

    def get_io_value(self, name: str) -> bool | None:
        for io in self.data.digital_inputs:
            io: DigitalIO
            if io.name == name:
                return io.value
        print(f'IO name "{name}" not found in the data', flush=True)
        return None

class ModbusSubscriber():
    def __init__(self, node: Node):
        self._node = node
        self._io_mtx = Lock()
        self._io_data: ModbusIOData = ModbusIOData()

        self._io_subscriber = self._node.create_subscription(
            InputsOutputs,
            '~/io',
            self.__update_io_data,
            10,
        )

    def __update_io_data(self, msg: InputsOutputs):
        now = self._node.get_clock().now()
        with self._io_mtx:
            self._io_data.update(now, msg)

    def is_timeout(self) -> bool:
        with self._io_mtx:
            if self._io_data.timestamp is None:
                return True
            now = self._node.get_clock().now()
            return (now - self._io_data.timestamp).nanoseconds / 1e9 > RECEIVED_IO_TIMEOUT

    def get_io_data(self) -> ModbusIOData:
        with self._io_mtx:
            return self._io_data


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

        self._default_laser_mode = "standard"  # empty to not set any mode at startup

        self._laser_modes = {
            "standard": {
                "input": {
                    "laser_mode_standard": True,
                    "laser_mode_charging_station": False,
                    "laser_mode_standby": False,
                    "laser_mode_charging": False,
                    "laser_mode_workstation": False,
                    "laser_mode_aux1": False,
                    "laser_mode_aux2": False,
                    "laser_mode_aux3": False,
                },
                "output": {
                    "set_laser_mode_standard": True,
                    "set_laser_mode_docking": False,
                    "set_laser_mode_standby": False,
                    "set_laser_mode_unknown1": False,
                    "set_laser_mode_manipulation": False,
                    "set_laser_mode_unknown2": False,
                    "set_laser_mode_unknown3": False,
                    "set_laser_mode_unknown4": False,
                },
            },
            "charging_station": {
                "input": {
                    "laser_mode_standard": False,
                    "laser_mode_charging_station": True,
                    "laser_mode_standby": False,
                    "laser_mode_charging": False,
                    "laser_mode_workstation": False,
                    "laser_mode_aux1": False,
                    "laser_mode_aux2": False,
                    "laser_mode_aux3": False,
                },
                "output": {
                    "set_laser_mode_standard": False,
                    "set_laser_mode_docking": True,
                    "set_laser_mode_standby": False,
                    "set_laser_mode_unknown1": False,
                    "set_laser_mode_manipulation": False,
                    "set_laser_mode_unknown2": False,
                    "set_laser_mode_unknown3": False,
                    "set_laser_mode_unknown4": False,
                },
            },
            "manipulation": {
                "input": {
                    "laser_mode_standard": False,
                    "laser_mode_charging_station": False,
                    "laser_mode_standby": False,
                    "laser_mode_charging": False,
                    "laser_mode_workstation": True,
                    "laser_mode_aux1": False,
                    "laser_mode_aux2": False,
                    "laser_mode_aux3": False,
                },
                "output": {
                    "set_laser_mode_standard": False,
                    "set_laser_mode_docking": False,
                    "set_laser_mode_standby": False,
                    "set_laser_mode_unknown1": False,
                    "set_laser_mode_manipulation": True,
                    "set_laser_mode_unknown2": False,
                    "set_laser_mode_unknown3": False,
                    "set_laser_mode_unknown4": False,
                },
            },
        }

        self._laser_attr = {
            "front_laser": {
                "detecting_obstacles": "front_laser_detecting_obstacles",
                "contamination": "front_laser_contamination_led",
                "free_warning": "front_laser_free_warning",
            },
            "rear_laser": {
                "detecting_obstacles": "rear_laser_detecting_obstacles",
                "contamination": "rear_laser_contamination_led",
                "free_warning": "rear_laser_free_warning",
            },
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

    def _set_default_laser_mode(self) -> bool:
        if not self._default_laser_mode:
            return True  # No default mode set, nothing to do

        if self._default_laser_mode in self._laser_modes:
            self.get_logger().info(f'Setting default laser mode: {self._default_laser_mode}')
            request = SetString.Request()
            request.data = self._default_laser_mode
            response = self._set_laser_mode(request, SetString.Response())
            if not response.response.success:
                self.get_logger().error(f'Failed to set default laser mode: {response.response.message}')
            return response.response.success

        return True  # Silently ignore if the default mode is not defined in the laser modes

    # State methods
    def init_state(self):
        # Set initial laser mode if defined
        if not self._set_default_laser_mode():
            self.transition_to_state(State.EMERGENCY)
        else:
            self.transition_to_state(State.READY)

    def standby_state(self):
        self.get_logger().info('In standby state')

    def ready_state(self):
        # Check if we have io updated
        if self._modbus_subscriber.is_timeout():
            self.get_logger().warn(f'No IO data received in the last {RECEIVED_IO_TIMEOUT} seconds')
            self.transition_to_state(State.EMERGENCY)
            return

        # Get the last IO data
        last_io_data = self._modbus_subscriber.get_io_data()

        def get_current_selector_mode() -> str:
            if last_io_data.get_io_value('selector_mode_auto'):
                return 'auto'
            elif last_io_data.get_io_value('selector_mode_manual'): # laser_mute'):
                return 'manual'
            elif last_io_data.get_io_value('selector_mode_maintenance'):
                return 'maintenance'
            else:
                return 'invalid'

        def get_current_laser_mode() -> str:
            for mode, config in self._laser_modes.items():
                matched = True
                for input_name, expected_value in config['input'].items():
                    if last_io_data.get_io_value(input_name) != expected_value:
                        matched = False
                        break
                if matched:
                    return mode
            return 'invalid'

        # Emergency and safety stop logic
        emergency_stop = not last_io_data.get_io_value('emergency_stop')
        safety_stop = not last_io_data.get_io_value('safety_stop')

        # Working mode key
        selector_mode = get_current_selector_mode()
        laser_mute = last_io_data.get_io_value('laser_mute')  # laser_mute: selector_mode_manual

        # Check laser mode
        current_laser_mode = get_current_laser_mode()

        # Publish status
        emergency_stop_msg = Bool()
        emergency_stop_msg.data = emergency_stop
        self._emergency_stop_publisher.publish(emergency_stop_msg)

        # Publish safety mode status
        status_msg = SafetyModeStatus()
        status_msg.operation_mode = selector_mode
        status_msg.safety_mode = SafetyModeStatus.SAFETYMODE_LASER_MUTE if laser_mute else SafetyModeStatus.SAFETYMODE_SAFE
        status_msg.emergency_stop = emergency_stop
        status_msg.safety_stop = safety_stop
        status_msg.laser_mode = current_laser_mode

        for laser, laser_attr in self._laser_attr.items():
            status = LaserStatus()
            status.name = laser
            status.detecting_obstacles = False if last_io_data.get_io_value(laser_attr['detecting_obstacles']) else True
            status.contaminated = True if last_io_data.get_io_value(laser_attr['contamination']) else False
            status.free_warning = True if last_io_data.get_io_value(laser_attr['free_warning']) else False
            status_msg.laser_status.append(status)

        self._status_publisher.publish(status_msg)


    def emergency_state(self):
        self.get_logger().warning('In emergency state', throttle_duration_sec=5.0)
        if not self._modbus_subscriber.is_timeout():
            self.get_logger().info('Emergency state: IO data received, moving to READY state')
            if self._set_default_laser_mode():
                self.transition_to_state(State.READY)

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
        self._emergency_stop_publisher = self.create_publisher(
            Bool,
            '~/emergency_stop',
            10,
        )

        # Subscribers
        self._modbus_subscriber = ModbusSubscriber(self)

        # Service client
        self._set_digital_output_callback_group = MutuallyExclusiveCallbackGroup()
        self._set_digital_output_client = self.create_client(
            SetDigitalOutputArray,
            '~/set_digital_output_array', # '~/set_digital_output_array',
            callback_group=self._set_digital_output_callback_group,
        )

        # Service server
        self._set_laser_mode_service = self.create_service(
            SetString,
            '~/set_laser_mode',
            self._set_laser_mode,
        )

    def _set_laser_mode(self, request: SetString.Request, response: SetString.Response) -> SetString.Response:
        if request.data not in self._laser_modes:
            response.response.success = False
            response.response.message = f'Requested laser mode "{request.data}" is not valid. Valid modes are: {list(self._laser_modes.keys())}'
            self.get_logger().error(response.response.message)
            return response

        # Set outputs based on the requested laser mode
        requested_mode = request.data
        output_array = SetDigitalOutputArray.Request()
        output_array.output = []
        for output_name, output_value in self._laser_modes[requested_mode]['output'].items():
            output = DigitalIO()
            output.name = output_name
            output.value = output_value
            output_array.output.append(output)

        if self._set_digital_output_client is None or not self._set_digital_output_client.wait_for_service(timeout_sec=1.0):
            response.response.success = False
            response.response.message = f'Service set_digital is not available.'
            self.get_logger().error(response.response.message)
            return response

        # Call the service to set the digital outputs
        future = self._set_digital_output_client.call_async(output_array)
        rclpy.spin_until_future_complete(self, future)
        serv_resp: SetDigitalOutputArray.Response = future.result()
        if serv_resp is None:
            response.response.success = False
            response.response.message = 'Failed to call set digital output service'
            response.response.message += f' Requested mode: {requested_mode}'
            self.get_logger().error(response.response.message)
            return response

        # TODO: Check if actual outputs match the requested mode

        if not serv_resp.response.success:
            response.response.success = False
            response.response.message = f'Failed to set laser mode outputs: {serv_resp.response.message}.'
            response.response.message += f' Requested mode: {requested_mode}'
            self.get_logger().error(response.response.message)
        else:
            response.response.success = True
            response.response.message = f'Successfully set laser mode outputs for: {requested_mode}'
            self.get_logger().info(response.response.message)
        return response

def main(args=None):
    rclpy.init(args=args)

    node = RobotnikFlexisoft()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info('Shutting down due to external request or keyboard interrupt.')
    except Exception as e:
        node.get_logger().error(f'An error occurred: {e}')

if __name__ == '__main__':
    main()
