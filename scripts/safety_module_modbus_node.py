#!/usr/bin/env python3

import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException

from enum import Enum, auto

from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from robotnik_common_msgs.srv import SetString
from robotnik_safety_msgs.msg import SafetyModeStatus, LaserStatus
from robotnik_io_msgs.msg import InputsOutputs, DigitalIO
from robotnik_io_msgs.srv import SetDigitalOutputArray

from threading import Lock


RECEIVED_IO_TIMEOUT = 0.5  # Seconds
RECEIVED_SPEED_TIMEOUT = 0.5  # Seconds

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
            'io',
            self.__update_io_data,
            qos_profile=qos_profile_sensor_data
        )

    def __update_io_data(self, msg: InputsOutputs):
        now = self._node.get_clock().now()
        self._node.get_logger().info(f'Received IO data with {len(msg.digital_inputs)} inputs and {len(msg.digital_outputs)} outputs')
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

class SpeedSubscriber():
    def __init__(self, node: Node):
        self._node = node
        self._speed_mtx = Lock()
        self._timestamp: Time | None = None
        self._current_speed = 0.0  # m/s

        self._speed_subscriber = self._node.create_subscription(
            Odometry,
            'odom',
            self.__update_speed,
            qos_profile=qos_profile_sensor_data
        )

    def __update_speed(self, msg: Odometry):
        self._timestamp = self._node.get_clock().now()
        with self._speed_mtx:
            x = msg.twist.twist.linear.x
            y = msg.twist.twist.linear.y
            self._current_speed = (x * x + y * y) ** 0.5  # m/s

    def is_timeout(self) -> bool:
        with self._speed_mtx:
            return self.__timeout_check()

    def __timeout_check(self) -> bool:
        """Not thread-safe check for timeout."""
        if self._timestamp is None:
            return True
        now = self._node.get_clock().now()
        return (now - self._timestamp).nanoseconds / 1e9 > RECEIVED_SPEED_TIMEOUT

    def get_speed(self) -> float:
        """Get the current speed in m/s. 0.0 if timeout."""
        with self._speed_mtx:
            if self.__timeout_check():
                return 0.0
            return self._current_speed


class RobotnikFlexisoft(Node):
    def __init__(self):
        super().__init__('robotnik_flexisoft')

        self._current_state = State.INIT
        self._state_callbacks = {
            State.INIT:       self.init_state,
            State.STANDBY:    self.standby_state,
            State.READY:      self.ready_state,
            State.EMERGENCY:  self.emergency_state,
            State.FAILURE:    self.failure_state,
            State.SHUTDOWN:   self.shutdown_state,
        }

        config_path = ""
        self.declare_parameter('config_path', config_path)
        config_path = self.get_parameter('config_path').get_parameter_value().string_value
        self.get_logger().info(f'Loading configuration from: {config_path}')

        # Load the configuration file yaml
        try:
            with open(config_path, 'r') as file:
                config = yaml.safe_load(file)
                if not config:
                    raise ValueError("Configuration file is empty or invalid.")
                self.get_logger().info(f'Configuration loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load configuration file: {e}')
            raise

        # Read label
        self._global = {
            'emergency_stop': 'emergency_stop',
            'safety_stop': 'safety_stop',
            'selector_mode_auto': 'selector_mode_auto',
            'selector_mode_manual': 'selector_mode_manual',
            'selector_mode_maintenance': 'selector_mode_maintenance',
            'laser_mute': 'laser_mute',
        }
        for key, value in config.get('global', {}).items():
            self._global[key] = value

        # Initialize laser modes and attributes
        self._default_laser_mode = config.get('laser', {}).get('default_mode', None)
        self._laser_modes = {}
        for mode_name, mode_config in config.get('laser', {}).get('modes', {}).items():
            if mode_name not in self._laser_modes:
                self._laser_modes[mode_name] = {
                    'input': {},
                    'output': {},
                }
            for input_name, input_value in mode_config.get('input', {}).items():
                self._laser_modes[mode_name]['input'][input_name] = input_value
            for output_name, output_value in mode_config.get('output', {}).items():
                self._laser_modes[mode_name]['output'][output_name] = output_value

        self._laser_attr = {}
        for laser_name, laser_config in config.get('laser', {}).get('attributes', {}).items():
            if laser_name not in self._laser_attr:
                self._laser_attr[laser_name] = {}
            for attr_name, attr_value in laser_config.items():
                self._laser_attr[laser_name][attr_name] = attr_value

        period = 0.1  # Seconds
        self.__timer = self.create_timer(period, self._control_loop)

        # Watchdog configuration
        self.__signals = {}
        watchdog_config = config.get('watchdog', {})
        if watchdog_config.get('enabled', False):
            self.__signals[watchdog_config.get('signal_a', 'watchdog_signal_a')] = False
            self.__signals[watchdog_config.get('signal_b', 'watchdog_signal_b')] = True
            period_ms = watchdog_config.get('period_ms', 1400)
            self.get_logger().info(f'Watchdog enabled with period: {period_ms} ms')
            semi_period_s = period_ms / 2000.0  # Convert ms to seconds (half period)
            self.__watchdog_timer = self.create_timer(semi_period_s, self.__watchdog_loop)
        else:
            self.get_logger().info('Watchdog is disabled')
            self.__watchdog_timer = None

        # Speed configuration
        speed_config = config.get('speed', {})
        if speed_config.get('enabled', False):
            self.get_logger().info(f'Speed control enabled with period: {speed_config.get("period_ms", 100)} ms')
            self.__speed_signal_prefix = speed_config.get('prefix', 'speed_bit_')
            period = speed_config.get('period_ms', 100) / 1000.0
            self.__speed_timer = self.create_timer(period, self.__speed_loop)

        self.ros_setup()

    def _control_loop(self):
        # Call the current state's method
        self._state_callbacks[self._current_state]()

    def __watchdog_loop(self):
        # Check if watchdog is enabled
        if self._current_state != State.READY:
            return

        # Switch the watchdog signals
        for signal_name in self.__signals:
            self.__signals[signal_name] = not self.__signals[signal_name]

        # Write the watchdog signals
        try:
            args = []
            for signal_name, signal_value in self.__signals.items():
                args.append(signal_name)
                args.append(signal_value)
            success, message = self._write_digital_output(*args)
            if not success:
                self.get_logger().error(f'watchdog: Failed to set watchdog signals: {message}', throttle_duration_sec=5.0)

        except Exception as e:
            self.get_logger().error(f'watchdog: Failed to set watchdog signals: {str(e)}', throttle_duration_sec=5.0)

    def __speed_loop(self):
        # Check if watchdog is enabled
        if self._current_state != State.READY:
            return

        try:
            # Write bits in msb order
            current_speed_int_cm = int(self._speed_subscriber.get_speed() * 100.0)  # Convert m/s to cm/s

            if current_speed_int_cm < 0 or current_speed_int_cm > 4095:
                self.get_logger().error(f'speed: Invalid speed value: {current_speed_int_cm} cm/s', throttle_duration_sec=5.0)
                return

            args = []
            for i in range(0, 12):
                args.append(f'{self.__speed_signal_prefix}{i}')
                args.append(True if (current_speed_int_cm >> i) & 0x01 else False)

            success, message = self._write_digital_output(*args)
            if not success:
                self.get_logger().error(f'speed: Failed to set speed bits: {message}', throttle_duration_sec=5.0)

        except Exception as e:
            self.get_logger().error(f'speed: Failed to set speed bits: {str(e)}', throttle_duration_sec=5.0)

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
            # Don't log the response message, as it is already logged in the service callback
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

        def get_current_mode() -> tuple[str, str]:
            if last_io_data.get_io_value(self._global['selector_mode_auto']) is True:
                return SafetyModeStatus.OPERATIONMODE_AUTO, SafetyModeStatus.SAFETYMODE_SAFE

            elif last_io_data.get_io_value(self._global['selector_mode_manual']) is True:
                if last_io_data.get_io_value(self._global['laser_mute']) is True:
                    return SafetyModeStatus.OPERATIONMODE_MANUAL, SafetyModeStatus.SAFETYMODE_LASER_MUTE
                else:
                    return SafetyModeStatus.OPERATIONMODE_MANUAL, SafetyModeStatus.SAFETYMODE_SAFE

            elif last_io_data.get_io_value(self._global['selector_mode_maintenance']) is True:
                return SafetyModeStatus.OPERATIONMODE_MAINTENANCE, SafetyModeStatus.SAFETYMODE_LASER_MUTE

            else:
                return 'invalid', 'invalid'

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
        emergency_stop = not last_io_data.get_io_value(self._global['emergency_stop'])
        safety_stop = not last_io_data.get_io_value(self._global['safety_stop'])

        # Check laser mode
        current_laser_mode = get_current_laser_mode()

        # Publish status
        emergency_stop_msg = Bool()
        emergency_stop_msg.data = emergency_stop
        self._emergency_stop_publisher.publish(emergency_stop_msg)

        # Publish safety mode status
        status_msg = SafetyModeStatus()
        status_msg.operation_mode, status_msg.safety_mode = get_current_mode()
        status_msg.current_speed = self._speed_subscriber.get_speed()
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
        if not self._modbus_subscriber.is_timeout():
            self.get_logger().info('Setting default laser mode before transitioning to READY state')
            if self._set_default_laser_mode():
                self.transition_to_state(State.READY)
            else:
                self.get_logger().error('Failed to set default laser mode, staying in EMERGENCY state')
        else:
            self.get_logger().error(f'No IO data received in the last {RECEIVED_IO_TIMEOUT} seconds, staying in EMERGENCY state', throttle_duration_sec=5.0)


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
        self._speed_subscriber = SpeedSubscriber(self)

        # Service client
        self._set_digital_output_callback_group = MutuallyExclusiveCallbackGroup()
        self._set_digital_output_client = self.create_client(
            SetDigitalOutputArray,
            'set_digital_output_array',
            callback_group=self._set_digital_output_callback_group,
        )

        # Service server
        self._set_laser_mode_service = self.create_service(
            SetString,
            '~/set_laser_mode',
            self._set_laser_mode,
        )

    def _await_future(self, future, timeout_sec: float) -> bool:
        """
        Wait for a future to complete while letting the executor process callbacks.
        Returns True if done before timeout.
        """
        deadline_ns = self.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
        while not future.done() and self.get_clock().now().nanoseconds < deadline_ns:
            # Allow IO and timers to update
            rclpy.spin_once(self, timeout_sec=0.05)
        return future.done()

    def _write_digital_output(self, *args, **kwargs) -> tuple[bool, str]:
        """
        It can be used to set specific digital outputs if needed.
        _write_digital_output("output_name", output_value, "output_name2", output_value2, ..., timeout_sec=1.0)
        Non-blocking wait loop replaces rclpy.spin_until_future_complete to avoid starving Modbus IO updates.
        """
        timeout_sec = kwargs.pop('timeout_sec', 1.0)

        output_array = SetDigitalOutputArray.Request()
        output_array.output = []
        for i in range(0, len(args), 2):
            if i + 1 >= len(args):
                return False, 'Invalid number of arguments, must be pairs of (output_name, output_value)'
            output_name = args[i]
            output_value = args[i + 1]
            output = DigitalIO()
            output.name = output_name
            output.value = output_value
            output_array.output.append(output)

        if self._set_digital_output_client is None:
            return False, 'Service set_digital_output_array is not initialized.'

        # Wait for the service to be available without blocking other callbacks
        if not self._set_digital_output_client.wait_for_service(timeout_sec=timeout_sec):
            return False, f'Service \'{self._set_digital_output_client.service_name}\' is not available.'

        # Call the service asynchronously and wait with spin_once loop
        future = self._set_digital_output_client.call_async(output_array)
        if not self._await_future(future, timeout_sec=timeout_sec):
            return False, 'Timed out waiting for set_digital_output_array response'
        serv_resp: SetDigitalOutputArray.Response = future.result()
        if serv_resp is None:
            return False, 'Failed to call set digital output service, no response received'
        return serv_resp.response.success, serv_resp.response.message

    def _laser_inputs_match(self, mode: str) -> bool:
        """Confirm Flexisoft inputs reflect the requested laser mode."""
        io_data = self._modbus_subscriber.get_io_data()
        for input_name, expected_value in self._laser_modes[mode].get('input', {}).items():
            val = io_data.get_io_value(input_name)
            if val is None or val != expected_value:
                return False
        return True

    def _set_laser_mode(self, request: SetString.Request, response: SetString.Response) -> SetString.Response:
        if request.data not in self._laser_modes:
            response.response.success = False
            response.response.message = f'Requested laser mode "{request.data}" is not valid. Valid modes are: {list(self._laser_modes.keys())}'
            self.get_logger().error(response.response.message)
            return response

        try:
            # Retry up to 10 times within 5 seconds total
            tries = 0
            deadline_ns = self.get_clock().now().nanoseconds + int(5.0 * 1e9)
            last_err = ""

            seep_rate = self.create_rate(10)
            self.get_logger().info(f'Setting laser mode: {request.data}')
            while tries < 10 and self.get_clock().now().nanoseconds < deadline_ns:
                tries += 1

                # Write requested outputs
                arguments = []
                for output_name, output_value in self._laser_modes[request.data]['output'].items():
                    arguments.append(output_name)
                    arguments.append(output_value)

                success, message = self._write_digital_output(*arguments, timeout_sec=0.8)
                if not success:
                    last_err = message
                    self.get_logger().error(f'Attempt {tries}: set outputs failed: {message}')
                    rclpy.spin_once(self, timeout_sec=0.1)
                    continue

                # Allow IO to update then verify inputs reflect the mode
                self.get_logger().info(f'Attempt {tries}: set outputs succeeded, verifying inputs...')
                now = self.get_clock().now().nanoseconds
                while self.get_clock().now().nanoseconds - now < int(0.5 * 1e9):
                    rclpy.spin_once(self, timeout_sec=0.05)

                if not self._modbus_subscriber.is_timeout() and self._laser_inputs_match(request.data):
                    self.get_logger().info(f'Successfully set laser mode outputs for: {request.data} (tries={tries})')
                    response.response.success = True
                    response.response.message = f'Successfully set laser mode outputs for: {request.data}'
                    return response

                self.get_logger().warn(f'Attempt {tries}: IO not yet matching mode "{request.data}"')

            # If here, confirmation failed
            msg = f'Laser mode "{request.data}" not confirmed on inputs after {tries} tries or 5s'
            if last_err:
                msg += f' (last error: {last_err})'
            self.get_logger().error(msg)
            response.response.success = False
            response.response.message = msg
            return response

        except Exception as e:
            response.response.success = False
            response.response.message = f'setting laser mode failed: {str(e)}'
            self.get_logger().error(response.response.message)
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
