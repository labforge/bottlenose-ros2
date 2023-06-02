##############################################################################
#  Copyright 2023 Labforge Inc.                                              #
#                                                                            #
# Licensed under the Apache License, Version 2.0 (the "License");            #
# you may not use this project except in compliance with the License.        #
# You may obtain a copy of the License at                                    #
#                                                                            #
#     http://www.apache.org/licenses/LICENSE-2.0                             #
#                                                                            #
# Unless required by applicable law or agreed to in writing, software        #
# distributed under the License is distributed on an "AS IS" BASIS,          #
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   #
# See the License for the specific language governing permissions and        #
# limitations under the License.                                             #
##############################################################################
include_directories(
        /opt/pleora/ebus_sdk/Ubuntu-22.04-x86_64/include/
        /opt/pleora/ebus_sdk/Ubuntu-20.04-x86_64/include/
        /opt/pleora/ebus_sdk/Ubuntu-18.04-x86_64/include/)

set(_eBUS_LIB_DIRS
        /opt/pleora/ebus_sdk/Ubuntu-22.04-x86_64/lib/
        /opt/pleora/ebus_sdk/Ubuntu-20.04-x86_64/lib/
        /opt/pleora/ebus_sdk/Ubuntu-18.04-x86_64/lib/
        /opt/pleora/ebus_sdk/Ubuntu-22.04-x86_64/lib/genicam/bin/Linux64_x64/
        /opt/pleora/ebus_sdk/Ubuntu-20.04-x86_64/lib/genicam/bin/Linux64_x64/
        /opt/pleora/ebus_sdk/Ubuntu-18.04-x86_64/lib/genicam/bin/Linux64_x64/
        CACHE INTERNAL "Ebus library search paths")

set(_eBUS_LIB_LIST
        PvBase
        PvDevice
        PvStream
        PvVirtualDevice
        PvPersistence
        PvSystem
        PvBuffer
        PvGenICam
        PvSerial
        PvTransmitter
        # Genicam stuff
        GCBase_gcc48_v3_3
        GenApi_gcc48_v3_3
        log4cpp_gcc48_v3_3
        Log_gcc48_v3_3
        MathParser_gcc48_v3_3
        NodeMapData_gcc48_v3_3
        XmlParser_gcc48_v3_3
    CACHE INTERNAL "Ebus library list")

# Loop over library names
string (REPLACE " " ";" _eBUS_LIB_LIST "${_eBUS_LIB_LIST}")
foreach (_libname IN LISTS _eBUS_LIB_LIST)
    find_library(${_libname} ${_libname} HINTS ${_eBUS_LIB_DIRS} REQUIRED)
    list(APPEND eBUS_LIBRARIES ${${_libname}})
endforeach()