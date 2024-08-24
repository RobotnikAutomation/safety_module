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
Safety module
"""
from typing import Any, Callable

from ament_index_python import get_package_share_directory

from safety_module.auxiliar import load_config
from safety_module.loader import load_registers
from safety_module.registers.v1.register_base import RegisterBase


class SafetyModule:
    """
    Safety module
    """

    def __init__(self, interface, version):
        """
        Constructor

        :param interface: The interface
        :param version: The version

        """
        path = (
            f'{get_package_share_directory("safety_module")}/'
            f"tables/{interface}/v{version}/base.yaml"
        )
        self.config = load_config(path)
        self.plugins: list[RegisterBase] = (
            load_registers(self.config["registers"])
        )
        self.__write_callback: Callable = None

    def process(self, bits: list[int]) -> None:
        """
        Process the bits

        :param bits: The bits

        """
        for plugin in self.plugins:
            plugin.process(data=bits)

    def get_register_context(self, register_name: str) -> dict | None:
        """
        Get the context of a register

        :param register_name: The register name
        :return: The context of the register

        """
        register = self.get_register(register_name)
        if register is None:
            return None
        return register.get_context()

    def get_register(self, register_name: str) -> RegisterBase | None:
        """
        Get a register by name

        :param register_name: The register name
        :return: The register

        """
        for plugin in self.plugins:
            if plugin.get_name() == register_name:
                return plugin
        return None

    def get_registers(self) -> list[RegisterBase]:
        """
        Get the registers

        :return: The registers
        """
        return self.plugins

    def show(self) -> None:
        """
        Show the registers
        """
        print("-" * 50)
        print("Registers:")
        for plugin in self.plugins:
            print(plugin)
            # print(f'{plugin.get_name()}: {plugin.get_context()}')

    def set_write_callback(self, callback: Callable) -> None:
        """
        Set the write callback

        :param callback: The callback

        """
        self.__write_callback = callback

    def write(
        self, register_name: str, value: Any, write_callback: Callable = None
    ) -> None:
        """
        Write to a register

        :param register_name: The register name
        :param value: The value
        :param write_callback: The write callback

        """
        if write_callback is None:
            write_callback = self.__write_callback

        for plugin in self.plugins:
            if plugin.get_name() == register_name:
                plugin.write(value=value, set_value_callback=write_callback)


class SafetyModuleFactory:
    """
    Safety module factory
    """

    def __init__(self):
        """
        Constructor
        """
        self.__current_module: SafetyModule = None
        self.__interface: str = None
        self.__version: int = None

    def get_module(self) -> SafetyModule:
        """
        Get the current module

        :return: The current module

        """
        return self.__current_module

    def set_module(
        self, interface: str, version: int, write_callback: Callable
    ) -> SafetyModule | None:
        """
        Set the module

        :param interface: The interface, modbus, etc.
        :param version: The version
        :param write_callback: The write callback
        :return: The actual loaded module

        """
        if self.already_set(interface, version):
            return self.__current_module

        del self.__current_module
        try:
            self.__current_module = SafetyModule(interface, version)
        except FileNotFoundError:
            self.__current_module = None
            return None
        self.__current_module.set_write_callback(write_callback)
        self.__interface = interface
        self.__version = version
        return self.__current_module

    def already_set(self, interface: str, version: int) -> bool:
        """
        Check if the module is already set

        :param interface: The interface
        :param version: The version
        :return: True if the module is already set

        """
        if self.__current_module is None:
            return False
        return self.__interface == interface and self.__version == version

    __interfaces = {
        0x50: "modbus",
    }

    @staticmethod
    def get_interface(interface_id: int) -> str:
        """
        All interfaces have an unique id, this method returns the interface name

        :param interface_id: The interface id
        :return: The interface

        """
        return SafetyModuleFactory.__interfaces.get(interface_id, "unknown")
