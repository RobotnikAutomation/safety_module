#!/usr/bin/env python

from rcomponent.rcomponent import *

from robotnik_msgs.msg import inputs_outputs, LaserStatus, SafetyModuleStatus, State
from robotnik_msgs.srv import SetLaserMode, SetLaserModeResponse
from robotnik_msgs.srv import set_digital_output, set_digital_outputRequest, set_digital_outputResponse

RECEIVED_IO_TIMEOUT = 10.0


class SafetyModuleIO(RComponent):
    '''
        A class used to manage the safety of the robot via I/O
    '''

    def __init__(self):

        RComponent.__init__(self)

    def ros_read_params(self):
        '''
            Gets params from the server
        '''
        RComponent.ros_read_params(self)

        self.io_subscriber_name = rospy.get_param('~io_subscriber_name')
        self.set_digital_outputs_service_name = rospy.get_param(
            '~set_digital_outputs_service_name')
        self.lasers = rospy.get_param('~inputs/lasers')
        self.laser_modes_available_ = rospy.get_param('~laser_modes')

        if len(self.laser_modes_available_) == 0:
            rospy.logwarn('%s::init: no laser modes defined!' %
                          (self._node_name))
        else:
            for mode in self.laser_modes_available_:
                if not self.laser_modes_available_[mode].has_key('output'):
                    rospy.logerr('%s::init  : laser_modes format has to have the key output. Invalid format: %s',
                                 self._node_name, str(self.laser_modes_available_))
                    exit(-1)

                # Transform output numbers to binary
                binary_out = list('{0:08b}'.format(
                    self.laser_modes_available_[mode]['output']))
                self.laser_modes_available_[
                    mode]['output'] = binary_out[2:][::-1]

    def ros_setup(self):
        '''
            Creates and inits ROS components
        '''
        RComponent.ros_setup(self)

        self.safety_module_state_pub = rospy.Publisher(
            '~status', SafetyModuleStatus, queue_size=1)

        # Subscriber to I/O
        self.io_sub = rospy.Subscriber(
            self.io_subscriber_name, inputs_outputs, self.io_cb,  queue_size=1)

        # ROS service server
        self.set_laser_mode_server = rospy.Service(
            '~set_laser_mode',  SetLaserMode, self.setLaserModeCb)

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

        self.now = rospy.get_time()
        self.io_last_stamp = 0

        RComponent.setup(self)

    def ready_state(self):

        self.now = rospy.get_time()

        if (self.now - self.io_last_stamp) < RECEIVED_IO_TIMEOUT:

            self.laser_status = []
            for laser in self.lasers:
                laser_msg = LaserStatus()
                laser_msg.name = laser
                laser_msg.detecting_obstacles = not self.inputs_outputs_msg.digital_inputs[0]
                laser_msg.contaminated = False
                laser_msg.free_warning = self.inputs_outputs_msg.digital_inputs[1]
                self.laser_status.append(laser_msg)

            self.updateSafetyModuleStatus()

        else:
            rospy.logwarn(
                '%s::readyState: Not receiving i/o messages' % self._node_name)
            # TODO: change to emergency

    def ros_publish(self):
        '''
            Publish topics at standard frequency
        '''
        self.safety_module_state_pub.publish(self.sm_status_msg)

    def io_cb(self, msg):
        '''
            I/O Callback
        '''
        self.inputs_outputs_msg = msg
        self.io_last_stamp = rospy.get_time()

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
                res = self.switchLaserMode(req.mode)
                if res is True:
                    rospy.loginfo(
                        '%s::setLaserModeCb: setting laser mode: %s', self._node_name, req.mode)
                    response.ret = True
                else:
                    rospy.logerr('%s::setLaserModeCb: unable to set mode: %s',
                                 self._node_name, req.mode)
                    response.ret = False

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

        # TODO: instead of writing every code, write only the different ones
        for i, code in enumerate(codes, 1):
            res = self.setDigitalOutput(i, bool(int(code)))

        if res == True:
            rospy.loginfo_throttle(
                2, '%s::switchLaserMode: switching to mode: %s ' % (self._node_name, new_mode))
            self.laser_mode = new_mode
        else:
            rospy.logerr('%s::switchLaserMode: could not switch to mode: %s ' % (
                self._node_name, new_mode))

        return res

    def getLaserModeCode(self, mode):
        '''
            Returns the register value for the mode
        '''
        if self.checkValidLaserMode(mode) != True:
            return None

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

    def updateSafetyModuleStatus(self):
        # Robot mode
        self.sm_status_msg.safety_mode = SafetyModuleStatus.SAFE

        # is charging
        self.sm_status_msg.charging = False

        # lasers
        self.sm_status_msg.lasers_mode.name = self.laser_mode
        self.sm_status_msg.lasers_status = self.laser_status

        # emergency stop
        self.sm_status_msg.emergency_stop = False

        # safety overrided
        self.sm_status_msg.safety_overrided = False

        # lasers on standby
        self.sm_status_msg.lasers_on_standby = False

        # Check the safety override status
        self.sm_status_msg.safety_stop = False

        return
