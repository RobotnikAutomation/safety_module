# Copyright 2024 Robotnik Automation S.L.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# @maintanier Rafael Martin  <rmartin@robotnik.es> Robotnik Automation S.L.

from .safety_module import SafetyModuleFactory

import rospy

import robotnik_msgs.msg as robotnik_msgs
import robotnik_msgs.srv as robotnik_srvs
import std_srvs.srv as std_srvs

from typing import List, Union

# Global variables
IO_TIMEOUT = 1.0 # seconds

class SafetyModule:
    def __init__(self):
        rospy.loginfo("safety module started")
        self.__safety_factory: SafetyModuleFactory = SafetyModuleFactory()
        
        self.__trigger = False

        self.publishers: dict[str, rospy.Publisher] = {}
        publishers_data = [
            ("status_publisher", "~status", robotnik_msgs.SafetyModuleStatus),
        ]
        
        self.__subscribers: dict[str, rospy.Subscriber] = {}
        subscribers_data = [
            ('/robot/robotnik_modbus_io/input_output', robotnik_msgs.inputs_outputs, self.__io_callback),
            # ('~io', inputs_outputs, self.__io_callback),
        ]
        
        self.__services: dict[str, rospy.Service] = {}
        services_data = [
            ('~trigger', std_srvs.Trigger, self.__trigger_callback),
        ]
        
        self.__io_data: robotnik_msgs.inputs_outputs = None
        self.__io_data_time: rospy.Time = None
        
        for publisher, topic, msg_type in publishers_data:
            self.publishers[publisher] = rospy.Publisher(topic, msg_type, queue_size=10)
            
        for subscriber, msg_type, callback in subscribers_data:
            self.__subscribers[subscriber] = rospy.Subscriber(subscriber, msg_type, callback)
            
        for service, srv_type, callback in services_data:
            self.__services[service] = rospy.Service(service, srv_type, callback)
            
        self.loop_timer = rospy.Timer(rospy.Duration(0.1), self.__loop)
    
    def __del__(self):
        rospy.loginfo("safety module stopped")
        
    def _set_module_from_bitset(self, bitset: List[int]) -> None:
        """
        Set the module from the bitset.

        :param bitset: The bitset

        """
        # first 8 bits are for interface
        interface_bits = bitset[:8]
        interface_id = 0
        for i, bit in enumerate(interface_bits):
            interface_id += bit << i
        interface = SafetyModuleFactory.get_interface(interface_id)

        # next 8 bits are for version
        version_bits = bitset[8:16]
        version = 0
        for i, bit in enumerate(version_bits):
            version += bit << i
            
        interface = SafetyModuleFactory.get_interface(0x50)
        version = 1

        # Ignore if already set
        if self.__safety_factory.already_set(interface, version):
            return

        rospy.loginfo(f"setting safety module \"{interface}:v{version}\"")
        self.__safety_factory.set_module(
            interface, version, write_callback=self.__write_callback
        )

    def __fill_laser_status_msg(self, name: str) -> robotnik_msgs.LaserStatus:
        """
        Fill the laser status message.

        :param name: The name of the laser
        :return: The laser status message

        """
        current_module = self.__safety_factory.get_module()
        if current_module is None:
            return None
        upper_name = name.upper()

        # States
        laser_zone_free = current_module.get_register_context(
            f"{upper_name}_LASER_SAFE_ZONE_FREE"
        )["value"]
        laser_contamination = current_module.get_register_context(
            f"{upper_name}_LASER_CONTAMINATION"
        )["value"]
        laser_warning_zone_free = current_module.get_register_context(
            f"{upper_name}_LASER_WARNING_ZONE_FREE"
        )["value"]

        laser_status = robotnik_msgs.LaserStatus()
        laser_status.name = name
        laser_status.detecting_obstacles = not laser_zone_free
        laser_status.contaminated = laser_contamination
        laser_status.free_warning = not laser_warning_zone_free

        return laser_status

    def __fill_laser_mode(self) -> str:
        """
        Get the current laser mode from registers.

        :return: The current laser mode

        """
        current_module = self.__safety_factory.get_module()
        if current_module is None:
            return None
        safety_mode = current_module.get_register_context("SAFETY_MODE")[
            "value"
        ]
        if safety_mode == "standard":
            return current_module.get_register_context("SPECIAL_SAFETY_MODE")[
                "value"
            ]

    def __fill_status_msg(self, status_msg: robotnik_msgs.SafetyModuleStatus) -> None:
        """
        Fill the status message.

        :param status_msg: The status message

        """
        current_module = self.__safety_factory.get_module()
        if current_module is None:
            rospy.logwarn("no safety module loaded")
            return

        status_msg.operation_mode = current_module.get_register_context(
            "KEY_MODES"
        )["value"]
        status_msg.safety_mode = current_module.get_register_context(
            "LASER_MODE"
        )["value"]
        status_msg.emergency_stop = not bool(
            current_module.get_register_context("PWR_DRIVERS")["value"]
        )
        status_msg.safety_stop = not bool(
            current_module.get_register_context("MOTION_ENABLED")["value"]
        )
        status_msg.lasers_on_standby = False  # deprecated
        status_msg.current_speed = 0.0
        status_msg.lasers_mode.name = self.__fill_laser_mode()
        status_msg.lasers_status.append(self.__fill_laser_status_msg("front"))


    def __loop(self, event: rospy.timer.TimerEvent):
        if self.__io_data is None or self.__io_data_time is None:
            return

        if (rospy.Time.now() - self.__io_data_time).to_sec() > IO_TIMEOUT:
            rospy.logwarn_throttle(1.0, "inputs_outputs message not received in %f seconds", IO_TIMEOUT)
            return
        
        # Publish the status
        status_msg = robotnik_msgs.SafetyModuleStatus()
        self.__fill_status_msg(status_msg)

        self.publishers["status_publisher"].publish(status_msg)
        rospy.loginfo(status_msg)


    def __io_callback(self, msg: robotnik_msgs.inputs_outputs):
        """
        Receive the inputs_outputs message and process it.
        """
        self.__io_data = msg
        self.__io_data_time = rospy.Time.now()

        self._set_module_from_bitset(self.__io_data.digital_inputs[:16])

        # Process the digital inputs
        current_module = self.__safety_factory.get_module()
        if current_module is None:
            rospy.logwarn_throttle(1.0, "no safety module loaded, could not process data")
            return

        current_module.process(self.__io_data.digital_inputs[16:])

    def __write_callback(
        self, output: Union[List[int], int], value: Union[List[int], int]
    ) -> None:
        rospy.loginfo(f"write callback: {output} -> {value}")

        req = robotnik_srvs.set_digital_output_listRequest()
        if isinstance(output, int):
            req.output = [output]
            req.value = [value]
            
        elif isinstance(output, list):
            for i in range(len(output)):
                req.output.append(output[i])
                req.value.append(value[i])

        else:
            rospy.logerr("invalid output type")
            return
            
        try:
            service_name = '/robot/robotnik_modbus_io/write_digital_output_list'
            rospy.wait_for_service(service_name, timeout=1.0)
            write_service = rospy.ServiceProxy(service_name, robotnik_srvs.set_digital_output_list)
            resp = write_service(req)
            rospy.loginfo(f"service response: {resp}")
            
        except rospy.ServiceException as e:
            rospy.logerr(f"service call failed: {e}")
            return
        
        except rospy.ROSException as e:
            rospy.logerr(f"service not available: {e}")
            return
        
        except rospy.ROSInterruptException as e:
            rospy.logerr(f"service interrupted: {e}")
            return
        
        rospy.loginfo(f"service call succeeded: {req}")
        
    def __trigger_callback(self, req):
        rospy.loginfo("trigger callback")
        self.__trigger = not self.__trigger
        
        current_module = self.__safety_factory.get_module()
        if current_module is None:
            rospy.logwarn("no safety module loaded")
            return
        
        current_module.write("CHARGE_LATCHING", self.__trigger)
        
        res = std_srvs.TriggerResponse()
        res.success = True
        return res
        
        
        
        


def run():
    rospy.init_node('safety_module')
    safety_module = SafetyModule()
    rospy.spin()
