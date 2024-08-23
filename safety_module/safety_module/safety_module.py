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

from safety_module.auxiliar import load_config
from safety_module.loader import load_registers
from safety_module.registers.v1 import RegisterBase
from ament_index_python import get_package_share_directory


class SafetyModule:
    def __init__(self, interface, version):
        self.config = load_config(
            path=f'{get_package_share_directory("safety_module")}/tables/{interface}/v{version}/base.yaml'
        )
        self.plugins: list[RegisterBase] = load_registers(self.config["registers"])

    def process(self, bits):
        for plugin in self.plugins:
            plugin.process(data=bits)

    def get_register_context(self, register_name) -> dict | None:
        register = self.get_register(register_name)
        if register is None:
            return None
        return register.get_context()

    def get_register(self, register_name) -> RegisterBase | None:
        for plugin in self.plugins:
            if plugin.get_name() == register_name:
                return plugin
        return None

    def get_registers(self) -> list[RegisterBase]:
        return self.plugins

    def show(self):
        print("-" * 50)
        print("Registers:")
        for plugin in self.plugins:
            print(plugin)
            # print(f'{plugin.get_name()}: {plugin.get_context()}')

    def set_write_callback(self, callback):
        self.__write_callback = callback

    def write(self, register_name, value, write_callback=None):
        if write_callback is None:
            write_callback = self.__write_callback

        for plugin in self.plugins:
            if plugin.get_name() == register_name:
                plugin.write(value=value, set_value_callback=write_callback)


class SafetyModuleFactory:
    def __init__(self):
        self.__current_module = None
        self.__interface = None
        self.__version = None

    def get_module(self) -> SafetyModule:
        return self.__current_module

    def set_module(self, interface: str, version: str, write_callback=None):
        if self.already_set(interface, version):
            return

        del self.__current_module
        try:
            self.__current_module = SafetyModule(interface, version)
        except FileNotFoundError:
            self.__current_module = None
            return
        self.__current_module.set_write_callback(write_callback)
        self.__interface = interface
        self.__version = version

    def already_set(self, interface, version):
        if self.__current_module is None:
            return False
        return self.__interface == interface and self.__version == version

    __interfaces = {
        0x50: "modbus",
    }

    @staticmethod
    def get_interface(interface_id: int):
        return SafetyModuleFactory.__interfaces.get(interface_id, "unknown")
