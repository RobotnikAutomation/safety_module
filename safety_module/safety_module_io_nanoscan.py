from rclpy.node import Node

from robotnik_msgs.msg import InputsOutputs, State, SafetyModuleStatus, LaserStatus
from robotnik_msgs.srv import SetLaserMode, SetDigitalOutput
from std_srvs.srv import SetBool
from std_msgs.msg import Bool
from sick_safetyscanners2_interfaces.msg import OutputPaths
from nav_msgs.msg import Odometry
import math

# Global variables
RECEIVED_IO_TIMEOUT = 5.0
RECEIVED_ODOM_TIMEOUT = 5.0
RECEIVED_OUTPUT_PATHS_TIMEOUT = 5.0


class SafetyModuleIO(Node):
    """
    A class used to manage the safety of the robot via I/O
    """

    def __init__(self):
        super().__init__("safety_module_io_nanoscan")
        self.current_speed = 0.0

        # Setup
        self.setup()

        # Read parameters
        self.ros_read_params()

        # Setup ROS
        self.ros_setup()

    def ros_read_params(self):
        """
        Reads parameters from the ROS server
        """
        # Declare parameters
        self.declare_parameter("inputs.emergency_stop", 1)
        self.declare_parameter("inputs.safety_stop", 2)
        self.declare_parameter("outputs.laser_modes", [1, 2, 3, 4])
        self.declare_parameter("monitoring_cases", [''])
        self.declare_parameter("active_case_in_emergency", "")

        # Read parameters
        self.emergency_stop_input = self.get_parameter("inputs.emergency_stop").value
        self.safety_stop_input = self.get_parameter("inputs.safety_stop").value
        self.laser_modes_output_ids = self.get_parameter("outputs.laser_modes").value
        self.monitoring_cases_list = self.get_parameter("monitoring_cases").value
        self.monitoring_cases = {}
        self.laser_modes_available_ = []
        if self.monitoring_cases_list == ['']:
            self.get_logger().error("No monitoring cases defined")
            exit(-1)

        for case in self.monitoring_cases_list:
            self.declare_parameter(case + ".safety_mode", "")
            self.declare_parameter(case + ".operation_mode", "")
            self.declare_parameter(case + ".laser_mode", "")
            self.declare_parameter(case + ".speed_range", [0.0, 0.0])
            self.declare_parameter(case + ".case", "undefined")
            self.declare_parameter(case + ".laser_mode_outputs", [False, False, False, False])

            self.monitoring_cases[case] = {
                "safety_mode": self.get_parameter(case + ".safety_mode").value,
                "operation_mode": self.get_parameter(case + ".operation_mode").value,
                "laser_mode": self.get_parameter(case + ".laser_mode").value,
                "speed_range": self.get_parameter(case + ".speed_range").value,
                "case": self.get_parameter(case + ".case").value,
                "laser_mode_outputs": self.get_parameter(case + ".laser_mode_outputs").value
            }

            laser_mode = self.monitoring_cases[case]["laser_mode"]
            if laser_mode != "undefined" and laser_mode not in self.laser_modes_available_:
                self.laser_modes_available_.append(laser_mode)

        self.active_case_in_emergency = self.get_parameter("active_case_in_emergency").value

        if self.active_case_in_emergency not in self.monitoring_cases:
            self.get_logger().error(
                "The active_case_in_emergency (%s) is not defined in the monitoring_cases",
                self.active_case_in_emergency,
            )
            exit(-1)

    def ros_setup(self):
        """
        Setups ROS components
        """

        # Publishers
        self.__state_pub = self.create_publisher(State, "~/state", 10)
        self.__emergency_stop_pub = self.create_publisher(Bool, "~/emergency_stop", 1)
        self.__safety_stop_pub = self.create_publisher(Bool, "~/safety_stop", 1)
        self.__safety_module_state_pub = self.create_publisher(
            SafetyModuleStatus, "~/status", 1
        )

        # Subscribers
        self.__io_sub = self.create_subscription(InputsOutputs, "~/io", self.io_cb, 1)
        self.__lidar_output_paths_sub = self.create_subscription(
            OutputPaths, "~/lidar_output_paths", self.lidar_output_paths_cb, 1
        )
        self.__odometry_sub = self.create_subscription(
            Odometry, "~/odom", self.odometry_cb, 1
        )

        # ROS service clients
        self.__set_digital_output_client = self.create_client(
            SetDigitalOutput, "~/set_digital_output"
        )

        # ROS service servers
        self.__set_laser_mode_server = self.create_service(
            SetLaserMode, "~/set_laser_mode", self.set_laser_mode_cb
        )
        self.__set_standby_server = self.create_service(
            SetBool, "~/set_to_standby", self.set_standby_cb
        )

        # Timers
        self.__ros_health_timer = self.create_timer(1.0, self.check_topics_health)
        self.__loop_timer = self.create_timer(0.1, self.loop)

    def setup(self):
        """
        Initialize
        """
        self.sm_status_msg = SafetyModuleStatus()
        self.laser_mode = "unknown"
        self.inputs_outputs_msg = InputsOutputs()
        self.lidar_output_paths_msg = OutputPaths()
        self.emergency_stop_msg = Bool()
        self.safety_stop_msg = Bool()
        self.safety_mode = "unknown"
        self.operation_mode = "unknown"
        self.emergency_stop = False
        self.safety_stop = False
        self.safety_overrided = False

        self.set_desired_standby_mode = None
        self.set_desired_laser_mode = None

        self.now = self.get_clock().now()
        self.io_last_stamp = None
        self.lidar_output_paths_last_stamp = None
        self.odom_last_stamp = None

    def check_topics_health(self):
        """
        Return true if the health of the received topics is OK
        """
        ret = True

        is_timeout = lambda last_stamp, timeout: (
            self.now - last_stamp
        ).to_msg().sec + (self.now - last_stamp).to_msg().nanosec / 1e9 >= timeout if last_stamp else True
        
        if is_timeout(self.io_last_stamp, RECEIVED_IO_TIMEOUT):
            self.get_logger().error("No data received from io topic", throttle_duration_sec=5)
            ret = False

        if is_timeout(self.lidar_output_paths_last_stamp, RECEIVED_OUTPUT_PATHS_TIMEOUT):
            self.get_logger().error("No data received from lidar_output_paths topic", throttle_duration_sec=5)
            ret = False

        if is_timeout(self.odom_last_stamp, RECEIVED_ODOM_TIMEOUT):
            self.get_logger().error("No data received from odom topic", throttle_duration_sec=5)
            ret = False
        return ret

    def loop(self):
        """
        Main loop
        """
        self.now = self.get_clock().now()

        if self.check_topics_health() == True:
            self.safety_mode = self.get_safety_mode()
            self.operation_mode = self.get_operation_mode()

            self.emergency_stop = self.inputs_outputs_msg.digital_inputs[
                self.emergency_stop_input - 1
            ]
            self.safety_stop = self.inputs_outputs_msg.digital_inputs[
                self.safety_stop_input - 1
            ]

            self.emergency_stop_msg.data = self.emergency_stop
            self.safety_stop_msg.data = self.safety_stop


            self.laser_status = []
            laser_msg = LaserStatus()
            laser_msg.name = "front"
            laser_msg.detecting_obstacles = self.is_obstacle_detected()
            laser_msg.contaminated = False
            laser_msg.free_warning = not self.is_warning_detected()
            self.laser_status.append(laser_msg)

            self.updateLaserMode()
            self.updateSafetyModuleStatus()

            # Performs the control of cases based on the current speed
            self.velocity_safety_case_control()

        else:
            self.velocity_safety_case_control(in_emergency=True)

        self.ros_publish()

    def ros_publish(self):
        """
        Publish topics at standard frequency
        """
        # Publish emergency stop
        self.__emergency_stop_pub.publish(self.emergency_stop_msg)
        # Publish safety stop
        self.__safety_stop_pub.publish(self.safety_stop_msg)
        # Publish the safety module status
        self.__safety_module_state_pub.publish(self.sm_status_msg)

    def io_cb(self, msg):
        """
        I/O Callback
        """
        self.inputs_outputs_msg = msg
        self.io_last_stamp = self.get_clock().now()

    def lidar_output_paths_cb(self, msg: OutputPaths):
        """
        Lidar Output Paths callback
        """
        if self.lidar_output_paths_msg.active_monitoring_case > len(
            self.monitoring_cases
        ):
            self.get_logger().error(
                "The active case is greater than current cases configuration!"
            )
            return

        self.lidar_output_paths_last_stamp = self.get_clock().now()
        self.lidar_output_paths_msg = msg

    def set_laser_mode_cb(
        self, req: SetLaserMode.Request, response: SetLaserMode.Response
    ):
        """
        ROS service server to change the laser mode
        """
        response.ret = False

        if not self.checkValidLaserMode(req.mode):
            self.get_logger().error("Invalid mode: %s", req.mode)
            return response

        self.set_desired_laser_mode = req.mode
        self.get_logger().info("Setting laser mode: %s", req.mode)
        response.ret = True
        return response

    def set_standby_cb(self, req: SetBool.Request, response: SetBool.Response):
        """
        ROS service server to enable/disable safety override
        """

        self.set_desired_standby_mode = req.data

        if self.safety_mode != "overridable":
            response.success = False
            response.message = (
                "Cannot activate/deactivate safety override because current mode is not manual %d"
                % req.data
            )
        else:
            response.success = True
            response.message = (
                "received petition to set the standby mode to %d" % req.data
            )

        return response

    def checkValidLaserMode(self, mode):
        """
        Checks if a laser mode is valid
        """
        return mode in self.laser_modes_available_

    def switchLaserMode(self, new_mode) -> bool:
        """
        Change the laser mode by sending the new mode through a service
        """
        codes = self.getLaserModeCode(new_mode)

        if codes == None:
            self.get_logger().error("Invalid mode: %s", new_mode)
            return False

        if len(codes) != len(self.laser_modes_output_ids):
            self.get_logger().error(
                "outputs_laser_modes and the output of each mode should be the same length"
            )
            return False

        res = True
        for i in range(0, len(self.laser_modes_output_ids)):
            res = res and self.setDigitalOutput(
                self.laser_modes_output_ids[i], codes[i]
            )

        if res == True:
            self.get_logger().info("Switching to mode: %s", new_mode)
            # self.laser_mode = new_mode
        else:
            self.get_logger().error("Error while switching to mode: %s", new_mode)

        return res

    def getLaserModeCode(self, mode):
        """
        Returns the register value for the mode
        """
        if self.checkValidLaserMode(mode) != True:
            return None
        return self.laser_modes_available_[mode]["output"]

    def setDigitalOutput(self, output: int, value: bool) -> bool:
        """
        Function that writes the info received into the write_digital_output service
        """
        try:
            request = SetDigitalOutput.Request()
            request.output = output
            request.value = value
            response: SetDigitalOutput.Response = self.__set_digital_output_client(
                request
            )
            return response.ret

        except Exception as e:
            self.get_logger().error("service call failed: %s" % str(e))
            return False

    def updateLaserMode(self):
        """
        Updates current laser mode based on the current inputs received
        """
        self.laser_mode = self.get_laser_mode()
        return

    def updateSafetyModuleStatus(self):
        # Robot mode
        self.sm_status_msg.safety_mode = self.safety_mode
        self.sm_status_msg.operation_mode = self.operation_mode

        # lasers
        self.sm_status_msg.lasers_mode.name = self.laser_mode
        self.sm_status_msg.lasers_status = self.laser_status

        # emergency stop
        self.sm_status_msg.emergency_stop = self.emergency_stop

        # lasers on standby
        self.sm_status_msg.lasers_on_standby = False

        self.sm_status_msg.safety_stop = self.safety_stop

        # speed (m/s)
        self.sm_status_msg.current_speed = self.current_speed

    def is_obstacle_detected(self):
        for i in range(0, len(self.lidar_output_paths_msg.status)):
            if self.lidar_output_paths_msg.is_valid[i] == True:
                if self.lidar_output_paths_msg.is_safe[i] == True:
                    if self.lidar_output_paths_msg.status[i] == False:
                        return True
        return False

    def is_warning_detected(self):
        for i in range(0, len(self.lidar_output_paths_msg.status)):
            if self.lidar_output_paths_msg.is_valid[i] == True:
                if self.lidar_output_paths_msg.is_safe[i] == False:
                    if self.lidar_output_paths_msg.status[i] == False:
                        return True
        return False

    def get_safety_mode(self):
        return self.monitoring_cases[
            str(self.lidar_output_paths_msg.active_monitoring_case)
        ]["safety_mode"]

    def get_operation_mode(self):
        return self.monitoring_cases[
            str(self.lidar_output_paths_msg.active_monitoring_case)
        ]["operation_mode"]

    def get_safety_case(self):
        return self.monitoring_cases[
            str(self.lidar_output_paths_msg.active_monitoring_case)
        ]["case"]

    def get_laser_mode(self):
        return self.monitoring_cases[
            str(self.lidar_output_paths_msg.active_monitoring_case)
        ]["laser_mode"]

    def get_safety_speed_range(self):
        return self.monitoring_cases[
            str(self.lidar_output_paths_msg.active_monitoring_case)
        ]["speed_range"]

    def odometry_cb(self, msg: Odometry):
        """
        Callback for odometry
        """
        self.odom_last_stamp = self.get_clock().now()
        x = msg.twist.twist.linear.x
        y = msg.twist.twist.linear.y
        self.current_speed = math.sqrt(x * x + y * y)

    def velocity_safety_case_control(self, in_emergency=False):
        laser_mode = self.laser_mode
        current_case = str(self.lidar_output_paths_msg.active_monitoring_case)
        desired_case = ""
        current_operation_mode = self.get_operation_mode()

        if (
            self.set_desired_laser_mode != None
            and self.set_desired_laser_mode != self.laser_mode
        ):
            laser_mode = self.set_desired_laser_mode

        elif self.set_desired_laser_mode == self.laser_mode:
            self.set_desired_laser_mode = None  # avoid continuous control

        if (
            self.monitoring_cases[current_case]["safety_mode"]
            != SafetyModuleStatus.SAFE
        ):
            self.get_logger().info(
                "no control since it's in %s"
                % (self.monitoring_cases[current_case]["safety_mode"]),
                throttle_duration_sec=30,
            )
            return

        if in_emergency == True:
            desired_case = self.active_case_in_emergency
        else:
            # look for the desired case
            for case in self.monitoring_cases:
                if (
                    self.monitoring_cases[case]["operation_mode"]
                    == current_operation_mode
                ):
                    if self.monitoring_cases[case]["laser_mode"] == laser_mode:
                        if (
                            self.current_speed
                            >= self.monitoring_cases[case]["speed_range"][0]
                            and self.current_speed
                            <= self.monitoring_cases[case]["speed_range"][1]
                        ):
                            desired_case = case
                            break

        if desired_case == current_case:  # Nothing to do
            return

        if desired_case == "":  # It should not happen
            self.get_logger().error(
                "no desired case found for the current speed %f" % (self.current_speed)
            )
            return

        # Set the Digital outputs to change the mode
        len_of_outputs = len(self.laser_modes_output_ids)
        len_of_case_outputs = len(
            self.monitoring_cases[desired_case]["laser_mode_outputs"]
        )
        if len_of_outputs != len_of_case_outputs:
            self.get_logger().error(
                "the lenght of the laser_mode_outputs (%d) for the desired case (%s) is different than the default one (%d) "
                % (len_of_case_outputs, desired_case, len_of_outputs),
                throttle_duration_sec=5,
            )
            return

        for i in range(0, len_of_outputs):
            if (
                self.setDigitalOutput(
                    self.laser_modes_output_ids[i],
                    self.monitoring_cases[desired_case]["laser_mode_outputs"][i],
                )
                == False
            ):
                self.get_logger().error(
                    "%s::velocity_safety_case_control:error setting digital output %d to %s"
                    % (
                        self._node_name,
                        self.laser_modes_output_ids[i],
                        self.monitoring_cases[desired_case]["laser_mode_outputs"][i],
                    ),
                    throttle_duration_sec=5,
                )
                return
