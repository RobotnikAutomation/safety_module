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

"""Safety module loader functions."""

from importlib import import_module
from .registers.v1.register_base import RegisterBase


def snake_to_camel(snake_str: str) -> str:
    """
    Convert a snake string to a camel string.

    :param snake_str: The snake string
    :return: The camel string

    """
    return ''.join(word.capitalize() for word in snake_str.split('_'))


def get_register(version: str, config: dict) -> RegisterBase:
    """
    Load a register from a given configuration.

    :param version: The version of the register
    :param config: The configuration of the register
    :return: The register instance

    """
    # Extract the register name, description and type
    register_name = config.pop('name')
    register_description = config.pop('description')
    register_type = version + '.' + config.pop('type')
    register_module = register_type.split('.')

    # Load the register class
    global_module_name = 'safety_module.registers.' + register_type
    register = getattr(
        import_module(global_module_name),
        snake_to_camel(register_module[-1])
    )
    return register(register_name, register_description, config)


def load_registers(register_config: dict) -> list:
    """
    Load the registers from the given configuration.

    :param register_config: The configuration of the registers
    :return: The list of registers

    """
    register_return = []
    for version, register_list in register_config.items():
        for register in register_list:
            register_return.append(get_register(version, register))
    return register_return
