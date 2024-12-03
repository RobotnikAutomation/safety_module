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

"""LaserMode register."""

from typing import Any, List
from ..register_base import RegisterBase, RegisterWriteCallback


class LaserMode(RegisterBase):
    """LaserMode register."""

    def __init__(self, name, description, config):
        """
        Initialize the register.

        :param name: Name of the register
        :param description: Description of the register
        :param config: Configuration of the register

        """
        super().__init__(name, description, config["kind"])
        self._config = config
        self.__pins_list = config["pins_list"]
        self.__pins_config = config["config"]
        self.set_context(
            {
                "value": None,
                "description": "Unititialized",
            }
        )

    def get_type(self) -> str:
        """
        Get the type of the register.

        :return: Type of the register

        """
        return "LaserMode.v1"

    def process(self, data: List[int]) -> None:
        """
        Process the data.

        :param data: Data to process

        """
        if self.kind() == "output":
            return

        for config in self.__pins_config:
            for i, pin in enumerate(self.__pins_list):
                if data[pin] != config["value"][i]:
                    break
            else:
                value_str = config["name"]
                self.set_context(
                    {
                        "value": config["name"],
                        "description": self.get_value_description(value_str),
                    }
                )
                return

        # If no config matches, set the value to None
        self.set_context(
            {
                "value": "unknown",
                "description": "Unititialized",
            }
        )

    def write(
        self, value: bool, set_value_callback: RegisterWriteCallback
    ) -> None:
        """
        Write bool value to the register.

        :param value: Value to write
        :param set_value_callback: Callback to set the value

        """
        if self.kind() == "input":
            print(f"Cannot write to input register {self.get_name()}")
            return

        for config in self.__pins_config:
            if value == config["name"]:
                set_value_callback(self.__pins_list, config["value"])
                self.set_context(
                    {
                        "value": config["name"],
                        "description": self.get_value_description(config["name"]),
                    }
                )
                return

        # If no config matches, set the value to None
        self.set_context(
            {
                "value": "unknown",
                "description": "Unititialized",
            }
        )
