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

"""Registers first version."""

from typing import Any, Callable, List, Union

RegisterWriteCallback = (
    Callable[[Union[List[int], int], Union[List[int], int]], None]
)


class RegisterBase:
    """Base class for registers first version."""

    def __init__(self, name, description, kind):
        """
        Initialize the register.

        :param name: Name of the register
        :param description: Description of the register
        :param kind: Kind of the register

        """
        self.__name = name
        self.__description = description
        self.__context = None
        self.__kind = kind

    def process(self, data: dict[int]) -> None:
        """
        Process the data.

        :param data: Data to process
        """

    def write(
        self, value: Any, set_value_callback: RegisterWriteCallback
    ) -> None:
        """
        Write the value.

        :param value: Value to write
        :param set_value_callback: Callback to set the value
        """

    def get_name(self) -> str:
        """
        Get the name of the register.

        :return: Name of the register

        """
        return self.__name

    def get_description(self) -> dict:
        """
        Get the description of the register.

        :return: Description of the register

        """
        return self.__description

    def get_context(self) -> dict:
        """
        Get the context of the register.

        :return: Context of the register

        """
        return self.__context

    def set_context(self, context: dict) -> None:
        """
        Set the context of the register.

        :param context: Context of the register

        """
        self.__context = context

    def get_value_description(self, value: str) -> str:
        """
        Get the description of the value.

        :param value: Value to get the description

        :return: Description of the value

        """
        for description_value in self.get_description()["values"]:
            for key, key_value in description_value.items():
                if key == value:
                    return key_value
        return "Unknown"

    def __str__(self) -> str:
        """
        Get the string representation of the register.

        :return: String representation of the register

        """
        if self.__context is None:
            return f"{self.__name}: None"

        return f'{self.__name}: {self.get_context()["value"]}'

    def get_type(self) -> str:
        """
        Get the type of the register.

        :return: Type of the register

        """
        return type(self).__name__

    def kind(self) -> str:
        """
        Get the kind of the register.

        :return: Kind of the register

        """
        return self.__kind
