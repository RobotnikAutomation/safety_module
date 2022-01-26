#!/usr/bin/env python

from rcomponent.rcomponent import *

from robotnik_msgs.msg import inputs_outputs, LaserStatus, SafetyModuleStatus, State, LaserMode, BoolArray, named_inputs_outputs, named_input_output
from robotnik_msgs.srv import SetLaserMode, SetLaserModeResponse
from robotnik_msgs.srv import SetNamedDigitalOutput
from robotnik_msgs.srv import set_digital_output, set_digital_outputRequest, set_digital_outputResponse
from std_srvs.srv import SetBool, SetBoolResponse
from std_msgs.msg import Bool, Int32
from sick_safetyscanners.msg import OutputPathsMsg
from nav_msgs.msg import Odometry
import math


RECEIVED_IO_TIMEOUT = 5.0
RECEIVED_ODOM_TIMEOUT = 5.0
RECEIVED_OUTPUT_PATHS_TIMEOUT = 5.0


class SafetyModuleIO(RComponent):
    '''
        A class used to manage the safety of the robot via I/O
    '''

    def __init__(self):

        RComponent.__init__(self)
        self.current_speed = 0.0

    def ros_read_params(self):
        '''
            Gets params from the server
        '''
        RComponent.ros_read_params(self)

        self.io_subscriber_name = rospy.get_param('~io_subscriber_name')
        self.lidar_output_paths_subscriber_name = rospy.get_param('~lidar_output_paths_subscriber_name')

        self.set_digital_outputs_service_name = rospy.get_param(
            '~set_digital_outputs_service_name')

        #self.laser_modes_available_ = rospy.get_param('~laser_modes')
        self.emergency_stop_input = rospy.get_param('~inputs/emergency_stop')
        self.safety_stop_input = rospy.get_param('~inputs/safety_stop')
        self.laser_modes_output_ids = rospy.get_param('~outputs/laser_modes')
        self.monitoring_cases = rospy.get_param('~monitoring_cases')
        self.laser_modes_available_ = []
        for case in self.monitoring_cases:
            laser_mode = self.monitoring_cases[case]['laser_mode']
            if laser_mode != 'undefined' and laser_mode not in self.laser_modes_available_:
                self.laser_modes_available_.append(laser_mode)


        '''if len(self.laser_modes_available_) == 0:
            rospy.logwarn('%s::init: no laser modes defined!' %
                          (self._node_name))
        else:
            for mode in self.laser_modes_available_:
                if not self.laser_modes_available_[mode].has_key('output'):
                    rospy.logerr('%s::init  : laser_modes format has to have the key output. Invalid format: %s',
                                 self._node_name, str(self.laser_modes_available_))
                    exit(-1)
        '''


    def ros_setup(self):
        '''
            Creates and inits ROS components
        '''
        RComponent.ros_setup(self)

        self.state_pub = rospy.Publisher(
            '~state', State, queue_size=10)

        self.emergency_stop_pub = rospy.Publisher(
            '~emergency_stop', Bool, queue_size=1)

        self.safety_stop_pub = rospy.Publisher(
            '~safety_stop', Bool, queue_size=1)

        self.safety_module_state_pub = rospy.Publisher(
            '~status', SafetyModuleStatus, queue_size=1)

        # Subscriber to I/O
        self.io_sub = rospy.Subscriber(
            self.io_subscriber_name, inputs_outputs, self.io_cb,  queue_size=1)

        self.lidar_output_paths_sub = rospy.Subscriber(
            self.lidar_output_paths_subscriber_name, OutputPathsMsg, self.lidar_output_paths_cb,  queue_size=1)

        self.odometry_sub = rospy.Subscriber("robotnik_base_control/odom", Odometry, self.odometry_cb, queue_size=1)

        # ROS service servers
        self.set_laser_mode_server = rospy.Service(
            '~set_laser_mode',  SetLaserMode, self.setLaserModeCb)

        self.set_standby_server = rospy.Service(
            '~set_to_standby', SetBool, self.setStandByCb)


        # ROS service client
        self.write_digital_output_client = rospy.ServiceProxy(
            self.set_digital_outputs_service_name, set_digital_output)

    def setup(self):
        '''
            Initialize
        '''
        self.sm_status_msg = SafetyModuleStatus()
        self.laser_mode = "unknown"
        self.inputs_outputs_msg = inputs_outputs()
        self.lidar_output_paths_msg = OutputPathsMsg()
        self.emergency_stop_msg = Bool()
        self.safety_stop_msg = Bool()
        self.safety_mode = "unknown"
        self.operation_mode = "unknown"
        self.emergency_stop = False
        self.safety_stop = False
        self.safety_overrided = False

        self.set_desired_standby_mode = None
        self.set_desired_laser_mode = None

        self.now = rospy.Time.now()
        self.io_last_stamp = rospy.Time(0)
        self.lidar_output_paths_last_stamp = rospy.Time(0)
        self.odom_last_stamp = rospy.Time(0)

        RComponent.setup(self)

    def check_topics_health(self):
        '''
            Return true if the health of the received topics is OK
        '''
        ret = True
        if (self.now - self.io_last_stamp).to_sec() >= RECEIVED_IO_TIMEOUT:
            rospy.logerr_throttle(2, '%s::check_topics_health: no data received from io topic' % (self._node_name))
            ret = False
        if (self.now - self.lidar_output_paths_last_stamp).to_sec() >= RECEIVED_OUTPUT_PATHS_TIMEOUT:
            rospy.logerr_throttle(2, '%s::check_topics_health: no data received from nanoscan topic' % (self._node_name))
            ret = False
        if (self.now - self.odom_last_stamp).to_sec() >= RECEIVED_ODOM_TIMEOUT:
            rospy.logerr_throttle(2, '%s::check_topics_health: no data received from odom topic' % (self._node_name))
            ret = False

        return ret

    def standby_state(self):

        self.now = rospy.Time.now()

        if self.check_topics_health() == True:
            rospy.loginfo(
                '%s::emergency_state: Receiving all the topics correctly!' % self._node_name)
            self.switch_to_state(State.READY_STATE)


    def ready_state(self):

        self.now = rospy.Time.now()

        if self.check_topics_health() == True:

            self.safety_mode = self.get_safety_mode()
            self.operation_mode = self.get_operation_mode()

            self.emergency_stop = self.inputs_outputs_msg.digital_inputs[self.emergency_stop_input - 1]
            self.safety_stop = self.inputs_outputs_msg.digital_inputs[self.safety_stop_input - 1]

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
            rospy.logwarn(
                '%s::readyState: Not receiving all the topics correctly...' % self._node_name)
            self.switch_to_state(State.EMERGENCY_STATE)

    def emergency_state(self):
        self.now = rospy.Time.now()

        if self.check_topics_health() == True:
            rospy.loginfo(
                '%s::emergency_state: Receiving all the topics correctly!' % self._node_name)
            self.switch_to_state(State.READY_STATE)


    def ros_publish(self):
        '''
            Publish topics at standard frequency
        '''
        # Publish emergency stop
        self.emergency_stop_pub.publish(self.emergency_stop_msg)
        # Publish safety stop
        self.safety_stop_pub.publish(self.safety_stop_msg)
        # Publish the safety module status
        self.safety_module_state_pub.publish(self.sm_status_msg)

    def io_cb(self, msg):
        '''
            I/O Callback
        '''
        self.inputs_outputs_msg = msg
        self.io_last_stamp = rospy.Time.now()

    def lidar_output_paths_cb(self, msg):
        '''
            Lidar Output Paths callback
        '''
        if self.lidar_output_paths_msg.active_monitoring_case > len(self.monitoring_cases):
            rospy.logerr_throttle(5, '%s::lidar_output_paths_cb: the active case is greater than current cases configuration!'%(self._node_name, self.lidar_output_paths_msg.active_monitoring_case))
            return
        self.lidar_output_paths_msg = msg
        self.lidar_output_paths_last_stamp = rospy.Time.now()

    def setLaserModeCb(self, req):
        '''
           ROS service server to change the laser mode
        '''
        response = SetLaserModeResponse()

        if self._state != State.READY_STATE:
            rospy.logerr(
                '%s::setLaserModeCb: component not in READY STATE' % self._node_name)
            response.ret = False
        else:
            if not self.checkValidLaserMode(req.mode):
                rospy.logerr('%s::setLaserModeCb: invalid mode: %s',
                             self._node_name, req.mode)
                response.ret = False
            else:
                self.set_desired_laser_mode = req.mode
                rospy.loginfo('%s::setLaserModeCb: setting laser mode: %s', self._node_name, req.mode)
                response.ret = True

        return response

    def setStandByCb(self, req):
        '''
            ROS service server to enable/disable safety override
        '''
        response = SetBoolResponse()

        self.set_desired_standby_mode = req.data

        if self.safety_mode != 'overridable':
            response.success = False
            response.message = 'Cannot activate/deactivate safety override because current mode is not manual %d' % req.data
        else:
            response.success = True
            response.message = 'received petition to set the standby mode to %d' % req.data

        return response

    def checkValidLaserMode(self, mode):
        '''
            Checks if a laser mode is valid
        '''
        return (mode in self.laser_modes_available_)

    def switchLaserMode(self, new_mode):
        '''
            Change the laser mode by sending the new mode through a service
        '''
        codes = self.getLaserModeCode(new_mode)

        if codes == None:
            rospy.logerr('%s::switchLaserMode: invalid new mode: %s',
                         self._node_name, new_mode)
            return False

        if len(codes) != len(self.laser_modes_output_ids):
            rospy.logerr('%s::switchLaserMode: outputs_laser_modes and the output of each mode should be the same length')
            return False

        res = True
        for i in range(0, len(self.laser_modes_output_ids)):
            res = res and self.setDigitalOutput(self.laser_modes_output_ids[i], codes[i])

        if res == True:
            rospy.loginfo_throttle(
                2, '%s::switchLaserMode: switching to mode: %s ' % (self._node_name, new_mode))
            # self.laser_mode = new_mode
        else:
            rospy.logerr('%s::switchLaserMode: error while switching to mode: %s ' % (
                self._node_name, new_mode))

        return res

    def getLaserModeCode(self, mode):
        '''
            Returns the register value for the mode
        '''
        if self.checkValidLaserMode(mode) != True:
            return None

        # Change this
        return self.laser_modes_available_[mode]['output']

    def setDigitalOutput(self, output, value):
        '''
           Function that writes the info received into the write_digital_output service
        '''
        try:
            request = set_digital_outputRequest()
            request.output = output
            request.value = value
            response = self.write_digital_output_client(request)
            return response.ret
        except rospy.ServiceException, e:
            rospy.logerr(
                '%s::setDigitalOutput: service call failed: %s', self._node_name, str(e))
            return False
        except Exception, e:
            rospy.logerr(
                '%s::setDigitalOutput: service call failed: %s', self._node_name, str(e))
            return False

    def updateLaserMode(self):
        '''
            Updates current laser mode based on the current inputs received
        '''
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

        return

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
        return self.monitoring_cases[str(self.lidar_output_paths_msg.active_monitoring_case)]['safety_mode']

    def get_operation_mode(self):
        return self.monitoring_cases[str(self.lidar_output_paths_msg.active_monitoring_case)]['operation_mode']

    def get_safety_case(self):
        return self.monitoring_cases[str(self.lidar_output_paths_msg.active_monitoring_case)]['case']

    def get_laser_mode(self):
        return self.monitoring_cases[str(self.lidar_output_paths_msg.active_monitoring_case)]['laser_mode']

    def get_safety_speed_range(self):
        return self.monitoring_cases[str(self.lidar_output_paths_msg.active_monitoring_case)]['speed_range']

    def odometry_cb(self, msg):
        '''
            Callback for odometry
            @param msg: received message
            @type msg: navigation_msgs/Odometry
        '''

        self.odom_last_stamp = rospy.Time.now()
        x = msg.twist.twist.linear.x
        y = msg.twist.twist.linear.y
        self.current_speed = math.sqrt(x*x+y*y)
        #rospy.logwarn_throttle(2, '%s::odometry_cb: speed = %.3lf cm/s'%(self._node_name, self.current_speed))


    def velocity_safety_case_control(self):

        #self.current_speed
        laser_mode = self.laser_mode
        current_case = str(self.lidar_output_paths_msg.active_monitoring_case)
        desired_case = ''
        current_operation_mode = self.get_operation_mode()

        if self.set_desired_laser_mode != None and self.set_desired_laser_mode != self.laser_mode:
            laser_mode = self.set_desired_laser_mode

        elif self.set_desired_laser_mode == self.laser_mode:
            self.set_desired_laser_mode = None # avoid continuous control

        if self.monitoring_cases[current_case]['safety_mode'] != SafetyModuleStatus.SAFE:
            rospy.loginfo_throttle(30, "%s::velocity_safety_case_control: no control since it's in %s"%(self._node_name, self.monitoring_cases[current_case]['safety_mode']))
            return

        # look for the desired case
        for case in self.monitoring_cases:
            if self.monitoring_cases[case]['operation_mode'] == current_operation_mode:
                if self.monitoring_cases[case]['laser_mode'] == laser_mode:
                    if self.current_speed >= self.monitoring_cases[case]['speed_range'][0] and self.current_speed <= self.monitoring_cases[case]['speed_range'][1]:
                        desired_case = case
                        break
        rospy.loginfo_throttle(2,"%s::velocity_safety_case_control: case = %s, desired case %s, laser_mode = %s"%(self._node_name, current_case, desired_case, laser_mode))
        if desired_case == current_case: # Nothing to do
            return
        if desired_case == '': # It should not happen
            rospy.logerr_throttle(5, '%s::velocity_safety_case_control: it could get a desired case from monitoring_cases'%(self._node_name))
            return

        rospy.loginfo("%s::velocity_safety_case_control: setting desired case %s"%(self._node_name, desired_case))
        # Set the Digital outputs to change the mode
        len_of_outputs = len(self.laser_modes_output_ids)
        len_of_case_outputs = len(self.monitoring_cases[desired_case]['laser_mode_outputs'])
        if len_of_outputs != len_of_case_outputs:
            rospy.logerr_throttle(5, '%s::velocity_safety_case_control: the len of the laser_mode_outputs (%d) for the desired case (%s) is different than the default one (%d) '%(self._node_name, len_of_case_outputs, desired_case, len_of_outputs))
            return
        for i in range(0, len_of_outputs):
            if self.setDigitalOutput(self.laser_modes_output_ids[i], self.monitoring_cases[desired_case]['laser_mode_outputs'][i]) == False:
                rospy.logerr_throttle(5, '%s::velocity_safety_case_control:error setting digital output %d to %s'%(self._node_name, self.laser_modes_output_ids[i], self.laser_modes_output_ids[i], self.monitoring_cases[desired_case]['laser_mode_outputs'][i]))
                return

        return
