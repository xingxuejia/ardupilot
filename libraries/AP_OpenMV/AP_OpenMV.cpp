/*

   Inspired by work done here
   https://github.com/PX4/Firmware/tree/master/src/drivers/frsky_telemetry from Stefan Rado <px4@sradonia.net>
   https://github.com/opentx/opentx/tree/2.3/radio/src/telemetry from the OpenTX team

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/* 
   FRSKY Telemetry library
*/

#define AO_SERIALMANAGER_OPENMV_BAUD      115200
#define AO_SERIALMANAGER_OPENMV_BUFSIZE_RX    64
#define AO_SERIALMANAGER_OPENMV_BUFSIZE_TX    64

#include "AP_OpenMV.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_Common/AP_FWVersion.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Common/Location.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Baro/AP_Baro.h>
#include <stdio.h>
#include <math.h>

extern const AP_HAL::HAL& hal;

AP_OpenMV *AP_OpenMV::singleton;

AP_OpenMV::AP_OpenMV(bool _external_data) :
    use_external_data(_external_data)
{
    singleton = this;
    _port = NULL;
    _step = 0;
}

AP_OpenMV::~AP_OpenMV(void)
{
    singleton = nullptr;
}

/*
 * init - perform required initialisation
 */
bool AP_OpenMV::init()
{
    const AP_SerialManager &serial_manager = AP::serialmanager();

    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_OpenMV, 0))) {
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        _port->begin(AO_SERIALMANAGER_OPENMV_BAUD, AO_SERIALMANAGER_OPENMV_BUFSIZE_RX, AO_SERIALMANAGER_OPENMV_BUFSIZE_TX);
        return true;
    }

    return false;
}

bool AP_OpenMV::update()
{
    if (_port == NULL)
        return false;
    int16_t numc = _port->available();
    uint8_t data;
    uint8_t checksum = 0;

    for (int16_t i = 0; i < numc; i++)
    {
        data = _port->read();

        switch(_step){
        case 0:
            if (data == 0xA5)
                _step = 1;
            break;
        case 1:
            if (data == 0x5A)
                _step = 2;
            else
                _step = 0;
            break;
        case 2:
            _cx_temp = data;
            _step = 3;
            break;
        case 3:
            _cy_temp = data;
            _step = 4;
            break;
        case 4:
            _step = 0;
            checksum = _cx_temp +_cy_temp;
            if (checksum == data)
            {
                cx = _cx_temp;
                cy = _cy_temp;
                last_frame_ms = AP_HAL::millis();
                return true;
            }
            break;
        default:
            _step = 0;
        }

    }
    return false;
}


namespace AP {
    AP_OpenMV *openmv() {
        return AP_OpenMV::get_singleton();
    }
};
