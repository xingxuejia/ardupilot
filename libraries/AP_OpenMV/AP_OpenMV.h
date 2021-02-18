/*
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
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_HAL/utility/RingBuffer.h>

class AP_OpenMV {
public:
    AP_OpenMV(bool external_data=false);

    ~AP_OpenMV();

    /* Do not allow copies */
    AP_OpenMV(const AP_OpenMV &other) = delete;
    AP_OpenMV &operator=(const AP_OpenMV&) = delete;

    // init - perform required initialisation
    bool init();

    bool update(void);

    uint8_t cx;
    uint8_t cy;

    uint32_t last_frame_ms;

    // add statustext message to FrSky lib message queue
    void queue_message(MAV_SEVERITY severity, const char *text);

    // update error mask of sensors and subsystems. The mask uses the
    // MAV_SYS_STATUS_* values from mavlink. If a bit is set then it
    // indicates that the sensor or subsystem is present but not
    // functioning correctly
    uint32_t sensor_status_flags() const;

    static AP_OpenMV *get_singleton(void) {
        return singleton;
    }

    // get next telemetry data for external consumers of SPort data
    static bool get_telem_data(uint8_t &frame, uint16_t &appid, uint32_t &data);

private:
    AP_HAL::UARTDriver *_port;                  // UART used to send data to FrSky receiver
    AP_SerialManager::SerialProtocol _protocol; // protocol used - detected using SerialManager's SERIAL#_PROTOCOL parameter
    uint16_t _crc;

    uint32_t check_sensor_status_timer;
    uint32_t check_ekf_status_timer;
    uint8_t _paramID;

    uint8_t _step;
    uint8_t _cx_temp;
    uint8_t _cy_temp;
    
    static AP_OpenMV *singleton;

    // use_external_data is set when this library will
    // be providing data to another transport, such as FPort
    bool use_external_data;
    struct {
        uint8_t frame;
        uint16_t appid;
        uint32_t data;
        bool pending;
    } external_data;
};

namespace AP {
    AP_OpenMV *openmv();
};
