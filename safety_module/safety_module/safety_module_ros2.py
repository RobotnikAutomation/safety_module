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

"""
Safety module wrapper for ROS2
"""


from rclpy.node import Node
from rclpy.publisher import Publisher

import std_msgs.msg as std_msgs
import std_srvs.srv as std_srv
import robotnik_msgs.msg as robotnik_msg
import robotnik_msgs.srv as robotnik_srv
import robotnik_safety_msgs.msg as robotnik_safety_msg

from .safety_module import SafetyModuleFactory


class SafetyModuleNode(Node):
    """
    Safety module wrapper for ROS2
    """

    def __init__(self):
        """
        Constructor
        """
        super().__init__("safety_module_node")
        self.__safety_factory: SafetyModuleFactory = SafetyModuleFactory()

        self.__publishers: dict[str, Publisher] = {}
        self.__publishers_data = [
            ("status_publisher", "~/status", robotnik_msg.SafetyModuleStatus),
            ("emergency_stop", "~/emergency_stop", std_msgs.Bool),
            ("safety_stop", "~/safety_stop", std_msgs.Bool),
            (
                "raw_registers",
                "~/raw_registers",
                robotnik_safety_msg.RegisterArray,
            ),
        ]

        self.__subscriptions = []
        self.__subscriptions_data = [
            (
                "/robot/modbus_io/inputs_outputs",
                robotnik_msg.InputsOutputs,
                self._inputs_outputs_callback,
            ),
        ]

        self.__services = []
        self.__services_data = [
            (
                "~/set_laser_fields",
                robotnik_srv.SetLaserMode,
                self._set_laser_mode_callback,
            ),
            (
                "~/enable_charge_mode",
                std_srv.SetBool,
                self._enable_charge_mode_callback,
            ),
            (
                "~/set_brakes",
                std_srv.SetBool,
                self._set_brake_callback,
            ),
            (
                "~/one_short_beep",
                std_srv.Trigger,
                self._enable_short_beep_callback,
            ),
            (
                "~/enable_long_beep",
                std_srv.SetBool,
                self._enable_long_beep_callback,
            ),
        ]

        for publisher, topic, msg_type in self.__publishers_data:
            self.__publishers[publisher] = self.create_publisher(
                msg_type, topic, 1
            )

        for topic, msg_type, callback in self.__subscriptions_data:
            self.__subscriptions.append(
                self.create_subscription(msg_type, topic, callback, 1)
            )

        for topic, srv_type, callback in self.__services_data:
            self.__services.append(
                self.create_service(srv_type, topic, callback)
            )

        self.get_logger().info("Safety module node started")

    def _fill_laser_status(self, name: str) -> robotnik_msg.LaserStatus:
        """
        Fill the laser status message

        :param name: The name of the laser

        :return: The laser status message

        """
        current_module = self.__safety_factory.get_module()
        if current_module is None:
            return None
        upper_name = name.upper()

        laser_status = robotnik_msg.LaserStatus()
        laser_status.name = name
        laser_status.detecting_obstacles = not bool(
            current_module.get_register_context(
                f"{upper_name}_LASER_SAFE_ZONE_FREE"
            )
        )
        laser_status.contaminated = not bool(
            current_module.get_register_context(
                f"{upper_name}_LASER_CONTAMINATION"
            )
        )
        laser_status.free_warning = not bool(
            current_module.get_register_context(
                f"{upper_name}_LASER_WARNING_ZONE_FREE"
            )
        )
        return laser_status

    def _get_laser_mode(self) -> str:
        """
        Get the current laser mode from registers

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

        return safety_mode

    def _fill_status_msg(self) -> robotnik_msg.SafetyModuleStatus:
        """
        Fill the safety module status message

        :return: The safety module status message

        """
        current_module = self.__safety_factory.get_module()
        if current_module is None:
            return None

        status_msg = robotnik_msg.SafetyModuleStatus()
        status_msg.operation_mode = current_module.get_register_context(
            "KEY_MODES"
        )["value"]
        status_msg.safety_mode = current_module.get_register_context(
            "LASER_MODE"
        )["value"]
        status_msg.emergency_stop = not bool(
            current_module.get_register_context("PWR_DRIVES")
        )
        status_msg.safety_stop = not bool(
            current_module.get_register_context("MOTION_ENABLED")
        )
        status_msg.lasers_on_standby = False  # deprecated
        status_msg.current_speed = 0.0

        status_msg.lasers_mode.name = self._get_laser_mode()
        status_msg.lasers_status.append(self._fill_laser_status("front"))
        status_msg.lasers_status.append(self._fill_laser_status("rear"))
        return status_msg

    def _write_callback(
        self, output: list[int] | int, value: list[int] | int
    ) -> None:
        """
        Write callback for the safety module

        :param output: The output to write
        :param value: The value to write

        """
        MsgType = robotnik_srv.SetDigitalOutputWithMask
        msg = MsgType.Request()
        set_modbus_srv = self.create_client(
            MsgType, "/robot/modbus_io/set_digital_output_with_mask"
        )

        def get_size(list_: list[int] | int) -> int:
            """
            Get the size of the list

            :param list_: The list to get the size of

            :return: The size of the list
            """
            if isinstance(list_, int):
                return list_
            return max(list_)

        output_size = get_size(output) + 1
        msg.mask = [MsgType.Request.DISCARD] * output_size
        msg.value = [MsgType.Request.LOW] * output_size
        msg.logic = [MsgType.Request.POSITIVE] * output_size

        if isinstance(output, int):
            msg.mask[output] = MsgType.Request.SET_VALUE
            msg.value[output] = (
                MsgType.Request.HIGH if value else MsgType.Request.LOW
            )

        else:
            for i, out in enumerate(output):
                msg.mask[out] = MsgType.Request.SET_VALUE
                msg.value[out] = (
                    MsgType.Request.HIGH if value[i] else MsgType.Request.LOW
                )

        set_modbus_srv.call_async(msg)

    def _inputs_outputs_callback(self, msg: robotnik_msg.InputsOutputs) -> None:
        """
        Callback for the inputs outputs topic

        :param msg: The inputs outputs message

        """
        # first 8 bits are for interface
        interface_bits = msg.digital_inputs[:8]
        interface_id = 0
        for i, bit in enumerate(interface_bits):
            interface_id += bit << i
        interface = SafetyModuleFactory.get_interface(interface_id)

        # next 8 bits are for version
        version_bits = msg.digital_inputs[8:16]
        version = 0
        for i, bit in enumerate(version_bits):
            version += bit << i

        self.__safety_factory.set_module(
            interface, version, write_callback=self._write_callback
        )
        current_module = self.__safety_factory.get_module()
        if current_module is None:
            return

        current_module.process(msg.digital_inputs[16:])

        status_msg = self._fill_status_msg()
        self.__publishers["status_publisher"].publish(status_msg)
        self.__publishers["emergency_stop"].publish(
            std_msgs.Bool(data=status_msg.emergency_stop)
        )
        self.__publishers["safety_stop"].publish(
            std_msgs.Bool(data=status_msg.safety_stop)
        )

        raw_registers = robotnik_safety_msg.RegisterArray()
        for register in current_module.get_registers():
            if register.kind() == "output":
                continue

            key_value = robotnik_safety_msg.Register()
            key_value.key = register.get_name()
            key_value.value = str(register.get_context()["value"])
            key_value.type = register.get_type()
            key_value.description = register.get_context()["description"]
            raw_registers.registers.append(key_value)
        self.__publishers["raw_registers"].publish(raw_registers)

    def _set_laser_mode_callback(
        self,
        request: robotnik_srv.SetLaserMode.Request,
        response: robotnik_srv.SetLaserMode.Response,
    ) -> robotnik_srv.SetLaserMode.Response:
        """
        Callback for the set laser mode service

        :param request: The request
        :param response: The response

        :return: The response

        """
        current_module = self.__safety_factory.get_module()
        if current_module is None:
            self.get_logger().error("Safety module not set")
            return response

        if request.mode == "standard":
            current_module.write("SAFETY_MODE_SET", "standard")
            current_module.write("SPECIAL_SAFETY_MODE_CORRIDOR", False)
            current_module.write("SPECIAL_SAFETY_MODE_REDUCED_SPEED", False)

        elif request.mode == "charge":
            current_module.write("SAFETY_MODE_SET", "charge")
            current_module.write("SPECIAL_SAFETY_MODE_CORRIDOR", False)
            current_module.write("SPECIAL_SAFETY_MODE_REDUCED_SPEED", False)

        elif request.mode == "corridor":
            current_module.write("SAFETY_MODE_SET", "standard")
            current_module.write("SPECIAL_SAFETY_MODE_CORRIDOR", True)
            current_module.write("SPECIAL_SAFETY_MODE_REDUCED_SPEED", False)

        elif request.mode == "reduced_speed":
            current_module.write("SAFETY_MODE_SET", "standard")
            current_module.write("SPECIAL_SAFETY_MODE_CORRIDOR", False)
            current_module.write("SPECIAL_SAFETY_MODE_REDUCED_SPEED", True)

        else:
            self.get_logger().error(f"Invalid laser mode: {request.mode}")
            return response

        return response

    def _enable_charge_mode_callback(
        self,
        request: std_srv.SetBool.Request,
        response: std_srv.SetBool.Response,
    ) -> std_srv.SetBool.Response:
        """
        Callback for the enable charge mode service

        :param request: The request
        :param response: The response

        :return: The response

        """
        current_module = self.__safety_factory.get_module()
        if current_module is None:
            self.get_logger().error("Safety module not set")
            return response

        current_module.write("CHARGE_LATCHING", request.data)
        return response

    def _set_brake_callback(
        self,
        request: std_srv.SetBool.Request,
        response: std_srv.SetBool.Response,
    ) -> std_srv.SetBool.Response:
        """
        Callback for the set brake service

        :param request: The request
        :param response: The response

        :return: The response
        """
        current_module = self.__safety_factory.get_module()
        if current_module is None:
            self.get_logger().error("Safety module not set")
            return response

        current_module.write("BRAKE_LATCHING", request.data)
        return response

    def _enable_short_beep_callback(
        self,
        _: std_srv.Trigger.Request,
        response: std_srv.Trigger.Response,
    ) -> std_srv.Trigger.Response:
        """
        Callback for the enable short beep service

        :param _: The request (unused)
        :param response: The response

        :return: The response

        """
        current_module = self.__safety_factory.get_module()
        if current_module is None:
            self.get_logger().error("Safety module not set")
            return response

        current_module.write("BUZZER_BEEP", True)
        return response

    def _enable_long_beep_callback(
        self,
        request: std_srv.SetBool.Request,
        response: std_srv.SetBool.Response,
    ) -> std_srv.SetBool.Response:
        """
        Callback for the enable long beep service

        :param request: The request
        :param response: The response

        :return: The response

        """
        current_module = self.__safety_factory.get_module()
        if current_module is None:
            self.get_logger().error("Safety module not set")
            return response

        current_module.write("BUZZER_QUICK_BEEP", request.data)
        return response
