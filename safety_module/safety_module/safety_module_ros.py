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

from robotnik_msgs.msg import SafetyModuleStatus, inputs_outputs
from robotnik_msgs.srv import set_digital_output_list, set_digital_output_listRequest, set_digital_output_listResponse
from std_srvs.srv import Trigger, TriggerResponse

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
            ("status_publisher", "~status", SafetyModuleStatus),
        ]
        
        self.__subscribers: dict[str, rospy.Subscriber] = {}
        subscribers_data = [
            ('/robot/robotnik_modbus_io/input_output', inputs_outputs, self.__io_callback),
            # ('~io', inputs_outputs, self.__io_callback),
        ]
        
        self.__services: dict[str, rospy.Service] = {}
        services_data = [
            ('~trigger', Trigger, self.__trigger_callback),
        ]
        
        self.__io_data: inputs_outputs = None
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
        version = 2

        # Ignore if already set
        if self.__safety_factory.already_set(interface, version):
            return

        rospy.loginfo(f"Setting safety module \"{interface}:v{version}\"")
        self.__safety_factory.set_module(
            interface, version, write_callback=self.__write_callback
        )
        
    def __loop(self, event: rospy.timer.TimerEvent):
        """
        """
        if self.__io_data is None or self.__io_data_time is None:
            return
        
        if (rospy.Time.now() - self.__io_data_time).to_sec() > IO_TIMEOUT:
            rospy.logwarn_throttle(1.0, "inputs_outputs message not received in %f seconds", IO_TIMEOUT)
            return
        
        self._set_module_from_bitset(self.__io_data.digital_inputs[:16])

        # Process the digital inputs
        current_module = self.__safety_factory.get_module()
        if current_module is None:
            rospy.logwarn_throttle(1.0, "No safety module loaded")
            return
        
        current_module.process(self.__io_data.digital_inputs[16:])
        
        # processed_data = current_module.get_registers()
        # for register in processed_data:
        #     print(register)


    def __io_callback(self, msg: inputs_outputs):
        self.__io_data = msg
        self.__io_data_time = rospy.Time.now()

    def __write_callback(
        self, output: Union[List[int], int], value: Union[List[int], int]
    ) -> None:
        rospy.loginfo(f"write callback: {output} -> {value}")

        req = set_digital_output_listRequest()
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
            write_service = rospy.ServiceProxy(service_name, set_digital_output_list)
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
        
        res = TriggerResponse()
        res.success = True
        return res
        
        
        
        


def run():
    rospy.init_node('safety_module')
    safety_module = SafetyModule()
    rospy.spin()
