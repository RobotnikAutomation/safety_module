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

"""States register."""

from .register_base import RegisterBase, RegisterWriteCallback


class States(RegisterBase):
    """States register."""

    def __init__(self, name, description, config):
        """
        Initialize the register.

        :param name: Name of the register
        :param description: Description of the register
        :param config: Configuration of the register

        """
        super().__init__(name, description, config["kind"])
        self.__base_address = config["base_address"]
        self.__num_bits = config["num_bits"]
        if "bit_order" not in config:
            self.__bit_order = "msbf"
        else:
            self.__bit_order = config["bit_order"]
        self.__states = config["states"]
        self.set_context(
            {
                "value": None,
                "description": "Unititialized",
                "raw_value": None,
            }
        )

    def get_type(self) -> str:
        """
        Get the type of the register.

        :return: Type of the register

        """
        return "States.v1"

    def process(self, data: list[int]) -> None:
        """
        Process the data.

        :param data: Data to process

        """
        if self.kind() == "output":
            return

        value = 0
        for i in range(self.__num_bits):
            if self.__bit_order == "lsbf":
                value += data[self.__base_address + i] * (2**i)
            else:
                value += data[self.__base_address + i] * (
                    2 ** (self.__num_bits - 1 - i)
                )

        raw_value = value
        curr_state = {
            "name": 'unknown',
            "value": 'unknown',
        }
        for state_definition in self.__states:
            if state_definition["value"] == value:
                curr_state = state_definition
                break

        if curr_state:
            self.set_context(
                {
                    "value": curr_state["name"],
                    "description": (
                        self.get_value_description(curr_state["name"])
                    ),
                    "raw_value": raw_value,
                }
            )
        else:
            self.set_context(
                {
                    "value": f"unknown_{value}",
                    "description": "Unknown state",
                    "raw_value": raw_value,
                }
            )

    def write(
        self, value: str, set_value_callback: RegisterWriteCallback
    ) -> None:
        """
        Write the state value to the register.

        :param value: Value to write
        :param set_value_callback: Callback to set the value

        """
        if self.kind() == "input":
            print(f"Cannot write to input register {self.get_name()}")
            return

        # Get value from state name
        state = None
        if isinstance(value, str):
            for state_definition in self.__states:
                if state_definition["name"] == value:
                    state = state_definition
                    break
            else:
                print(f"Unknown state {value}")
                return

        addressess = []
        values = []
        for i in range(self.__num_bits):
            addressess.append(self.__base_address + i)
            if state["value"] & (1 << i):
                values.append(True)
            else:
                values.append(False)
        set_value_callback(addressess, values)

        self.set_context(state)
