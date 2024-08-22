from .safety_module import SafetyModuleFactory

from rclpy.node import Node
from rclpy.publisher import Publisher
import std_srvs.srv as std_srv
import robotnik_msgs.msg as robotnik_msg
import robotnik_msgs.srv as robotnik_srv


class SafetyModuleNode(Node):
    def __init__(self):
        super().__init__('safety_module_node')
        self.__safety_factory: SafetyModuleFactory = SafetyModuleFactory()

        self.__publishers: dict[str, Publisher] = {}
        self.__publishers_data = [
            ('status_publisher', '~/status', robotnik_msg.SafetyModuleStatus, 1),
        ]

        self.__subscriptions = []
        self.__subscriptions_data = [
            ('/robot/modbus_io/inputs_outputs', robotnik_msg.InputsOutputs, self._inputs_outputs_callback),
        ]

        self.__services = []
        self.__services_data = [
            ('~/set_laser_mode', robotnik_srv.SetLaserMode, self._set_laser_mode_callback),
            ('~/enable_charge_mode', std_srv.SetBool, self._enable_charge_mode_callback),
        ]

        for publisher, topic, msg_type, qos in self.__publishers_data:
            self.__publishers[publisher] = self.create_publisher(msg_type, topic, qos)

        for topic, msg_type, callback in self.__subscriptions_data:
            self.__subscriptions.append(self.create_subscription(msg_type, topic, callback, 1))

        for topic, srv_type, callback in self.__services_data:
            self.__services.append(self.create_service(srv_type, topic, callback))

        self.get_logger().info('Safety module node started')


    def _fill_laser_status(self, name: str):
        current_module = self.__safety_factory.get_module()
        if current_module is None:
            return None
        upper_name = name.upper()
        
        laser_status = robotnik_msg.LaserStatus()
        laser_status.name = name
        laser_status.detecting_obstacles = not bool(current_module.get_register_context(f'{upper_name}_LASER_SAFE_ZONE_FREE'))
        laser_status.contaminated = not bool(current_module.get_register_context(f'{upper_name}_LASER_CONTAMINATION'))
        laser_status.free_warning = False
        return laser_status


    def _fill_status_msg(self) -> robotnik_msg.SafetyModuleStatus:
        current_module = self.__safety_factory.get_module()
        if current_module is None:
            return None
        
        status_msg = robotnik_msg.SafetyModuleStatus()
        status_msg.operation_mode = 'unknown'
        status_msg.safety_mode = 'unknown'
        status_msg.emergency_stop = bool(current_module.get_register_context('ESTOP_OK')["value"]) # TODO: check with the team
        status_msg.safety_stop = bool(current_module.get_register_context('ESTOP_OK')["value"])
        status_msg.lasers_on_standby =  False # TODO: check with the team
        status_msg.current_speed = 0.0 # TODO: check with the team

        status_msg.lasers_mode.name = self._get_laser_mode()
        status_msg.lasers_status.append(self._fill_laser_status('front'))
        status_msg.lasers_status.append(self._fill_laser_status('rear'))
        return status_msg


    def _write_callback(self, output: list[int] | int, value: list[int] | int):
        MsgType = robotnik_srv.SetDigitalOutputWithMask
        msg = MsgType.Request()
        set_modbus_srv = self.create_client(MsgType, '/robot/modbus_io/set_digital_output_with_mask')

        get_size = lambda x: x if isinstance(x, int) else max(x)
        output_size = get_size(output) + 1
        msg.mask = [MsgType.Request.DISCARD] * output_size
        msg.value = [MsgType.Request.LOW] * output_size
        msg.logic = [MsgType.Request.POSITIVE] * output_size

        if isinstance(output, int):
            msg.mask[output] = MsgType.Request.SET_VALUE
            msg.value[output] = MsgType.Request.HIGH if value else MsgType.Request.LOW
    
        else:
            for i, out in enumerate(output):
                msg.mask[out] = MsgType.Request.SET_VALUE
                msg.value[out] = MsgType.Request.HIGH if value[i] else MsgType.Request.LOW
    
        set_modbus_srv.call_async(msg)


    def _inputs_outputs_callback(self, msg: robotnik_msg.InputsOutputs):
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

        self.__safety_factory.set_module(interface, version, write_callback=self._write_callback)
        current_module = self.__safety_factory.get_module()
        if current_module is None:
            return

        current_module.process(msg.digital_inputs[16:])
        status_msg = self._fill_status_msg()
        self.__publishers['status_publisher'].publish(status_msg)
        
        # print(status_msg)
        # current_module.show()


    def _get_laser_mode(self):
        current_module = self.__safety_factory.get_module()
        if current_module is None:
            return None
        safety_mode = current_module.get_register_context('SAFETY_MODE')['name']
        if safety_mode == 'driven_by_special':
            return current_module.get_register_context('SPECIAL_SAFETY_MODE')['name']

        return safety_mode
        

    def _set_laser_mode_callback(self, request: robotnik_srv.SetLaserMode.Request, response: robotnik_srv.SetLaserMode.Response):
        current_module = self.__safety_factory.get_module()
        if current_module is None:
            self.get_logger().error('Safety module not set')
            return response

        if request.mode == 'standard':
            current_module.write('SAFETY_MODE_SET', "standard")
            current_module.write('SPECIAL_SAFETY_MODE_CORRIDOR', False)
            current_module.write('SPECIAL_SAFETY_MODE_REDUCED_SPEED', False)

        elif request.mode == 'charge':
            current_module.write('SAFETY_MODE_SET', "charge")
            current_module.write('SPECIAL_SAFETY_MODE_CORRIDOR', False)
            current_module.write('SPECIAL_SAFETY_MODE_REDUCED_SPEED', False)

        elif request.mode == 'corridor':
            current_module.write('SAFETY_MODE_SET', "standard")
            current_module.write('SPECIAL_SAFETY_MODE_CORRIDOR', True)
            current_module.write('SPECIAL_SAFETY_MODE_REDUCED_SPEED', False)

        elif request.mode == 'reduced_speed':
            current_module.write('SAFETY_MODE_SET', "standard")
            current_module.write('SPECIAL_SAFETY_MODE_CORRIDOR', False)
            current_module.write('SPECIAL_SAFETY_MODE_REDUCED_SPEED', True)

        return response


    def _enable_charge_mode_callback(self, request: std_srv.SetBool.Request, response: std_srv.SetBool.Response):
        current_module = self.__safety_factory.get_module()
        if current_module is None:
            self.get_logger().error('Safety module not set')
            return response

        current_module.write('CHARGE_LATCHING', request.data)
        return response
