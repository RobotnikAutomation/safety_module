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

from . import RegisterBase


class SetReset(RegisterBase):
    def __init__(self, name, description, config):
        super().__init__(name, description, config["kind"])
        self._config = config
        self.__set_address = config["set_address"]
        self.__reset_address = config["reset_address"]
        self.set_context(
            {
                "value": None,
                "description": "Unititialized",
            }
        )

    def get_type(self):
        return "SetReset.v1"

    def write(self, value, set_value_callback):
        if value:
            set_value_callback(self.__reset_address, 0)
            set_value_callback(self.__set_address, 1)
        else:
            set_value_callback(self.__set_address, 0)
            set_value_callback(self.__reset_address, 1)
        self.set_context(
            {
                "value": value,
                "description": "Set" if value else "Unset",
            }
        )
