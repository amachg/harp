/*
 * RoboPeak RPLIDAR Driver for Arduino
 * RoboPeak.com
 * 
 * Copyright (c) 2014, RoboPeak 
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY 
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR 
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#pragma once

#include "Arduino.h"


// In-line included files, by me Tassos
/////////////////////#include "rptypes.h"/////////////////////////////
//////////////////////////////////////////////////////////////////////
#ifdef _WIN32

//fake stdint.h for VC only

typedef signed   char     int8_t;
typedef unsigned char     uint8_t;

typedef __int16           int16_t;
typedef unsigned __int16  uint16_t;

typedef __int32           int32_t;
typedef unsigned __int32  uint32_t;

typedef __int64           int64_t;
typedef unsigned __int64  uint64_t;

#else

#include <stdint.h>

#endif


//based on stdint.h
typedef int8_t         _s8;
typedef uint8_t        _u8;

typedef int16_t        _s16;
typedef uint16_t       _u16;

typedef int32_t        _s32;
typedef uint32_t       _u32;

typedef int64_t        _s64;
typedef uint64_t       _u64;

#define __small_endian

#ifndef __GNUC__
#define __attribute__(x)
#endif


// The _word_size_t uses actual data bus width of the current CPU
#ifdef _AVR_
typedef _u8            _word_size_t;
#define THREAD_PROC    
#elif defined (WIN64)
typedef _u64           _word_size_t;
#define THREAD_PROC    __stdcall
#elif defined (WIN32)
typedef _u32           _word_size_t;
#define THREAD_PROC    __stdcall
#elif defined (__GNUC__)
typedef unsigned long  _word_size_t;
#define THREAD_PROC   
#elif defined (__ICCARM__)
typedef _u32            _word_size_t;
#define THREAD_PROC  
#endif


typedef uint32_t u_result;

#define RESULT_OK              0
#define RESULT_FAIL_BIT        0x80000000
#define RESULT_ALREADY_DONE    0x20
#define RESULT_INVALID_DATA    (0x8000 | RESULT_FAIL_BIT)
#define RESULT_OPERATION_FAIL  (0x8001 | RESULT_FAIL_BIT)
#define RESULT_OPERATION_TIMEOUT  (0x8002 | RESULT_FAIL_BIT)
#define RESULT_OPERATION_STOP    (0x8003 | RESULT_FAIL_BIT)
#define RESULT_OPERATION_NOT_SUPPORT    (0x8004 | RESULT_FAIL_BIT)
#define RESULT_FORMAT_NOT_SUPPORT    (0x8005 | RESULT_FAIL_BIT)
#define RESULT_INSUFFICIENT_MEMORY   (0x8006 | RESULT_FAIL_BIT)

#define IS_OK(x)    ( ((x) & RESULT_FAIL_BIT) == 0 )
#define IS_FAIL(x)  ( ((x) & RESULT_FAIL_BIT) )

typedef _word_size_t (THREAD_PROC * thread_proc_t ) ( void * );


/////////////////////#include "rplidar_protocol.h"////////////////////
//////////////////////////////////////////////////////////////////////
// RP-Lidar Input Packets

#define RPLIDAR_CMD_SYNC_BYTE        0xA5
#define RPLIDAR_CMDFLAG_HAS_PAYLOAD  0x80


#define RPLIDAR_ANS_SYNC_BYTE1       0xA5
#define RPLIDAR_ANS_SYNC_BYTE2       0x5A

#define RPLIDAR_ANS_PKTFLAG_LOOP     0x1


#if defined(_WIN32)
#pragma pack(1)
#endif

typedef struct _rplidar_cmd_packet_t {
    _u8 syncByte; //must be RPLIDAR_CMD_SYNC_BYTE
    _u8 cmd_flag; 
    _u8 size;
    _u8 data[0];
} __attribute__((packed)) rplidar_cmd_packet_t;


typedef struct _rplidar_ans_header_t {
    _u8  syncByte1; // must be RPLIDAR_ANS_SYNC_BYTE1
    _u8  syncByte2; // must be RPLIDAR_ANS_SYNC_BYTE2
    _u32 size:30;
    _u32 subType:2;
    _u8  type;
} __attribute__((packed)) rplidar_ans_header_t;

#if defined(_WIN32)
#pragma pack()
#endif


/////////////////////#include "rplidar_cmd.h"/////////////////////////
//////////////////////////////////////////////////////////////////////
// Commands
//-----------------------------------------

// Commands without payload and response
#define RPLIDAR_CMD_STOP               0x25
#define RPLIDAR_CMD_SCAN               0x20
#define RPLIDAR_CMD_FORCE_SCAN         0x21
#define RPLIDAR_CMD_RESET              0x40


// Commands without payload but have response
#define RPLIDAR_CMD_GET_DEVICE_INFO      0x50
#define RPLIDAR_CMD_GET_DEVICE_HEALTH    0x52

#if defined(_WIN32)
#pragma pack(1)
#endif


// Response
// ------------------------------------------
#define RPLIDAR_ANS_TYPE_MEASUREMENT      0x81

#define RPLIDAR_ANS_TYPE_DEVINFO          0x4
#define RPLIDAR_ANS_TYPE_DEVHEALTH        0x6


#define RPLIDAR_STATUS_OK                 0x0
#define RPLIDAR_STATUS_WARNING            0x1
#define RPLIDAR_STATUS_ERROR              0x2

#define RPLIDAR_RESP_MEASUREMENT_SYNCBIT        (0x1<<0)
#define RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT  2
#define RPLIDAR_RESP_MEASUREMENT_CHECKBIT       (0x1<<0)
#define RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT    1

typedef struct _rplidar_response_measurement_node_t {
    _u8    sync_quality;      // syncbit:1;syncbit_inverse:1;quality:6;
    _u16   angle_q6_checkbit; // check_bit:1;angle_q6:15;
	_u16   distance_q2;
} __attribute__((packed)) rplidar_response_measurement_node_t;

typedef struct _rplidar_response_device_info_t {
    _u8   model;
    _u16  firmware_version;
    _u8   hardware_version;
    _u8   serialnum[16];
} __attribute__((packed)) rplidar_response_device_info_t;

typedef struct _rplidar_response_device_health_t {
    _u8   status;
    _u16  error_code;
} __attribute__((packed)) rplidar_response_device_health_t;

#if defined(_WIN32)
#pragma pack()
#endif
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////



struct RPLidarMeasurement
{
    float distance;
    float angle;
    uint8_t quality;
    bool  startBit;
};

class RPLidar
{
public:
    enum {
        RPLIDAR_SERIAL_BAUDRATE = 115200,  
        RPLIDAR_DEFAULT_TIMEOUT = 500,
    };
    
    RPLidar();  
    ~RPLidar();

    // open the given serial interface and try to connect to the RPLIDAR
    bool begin(HardwareSerial &serialobj);

    // close the currently opened serial interface
    void end();
  
    // check whether the serial interface is opened
    bool isOpen(); 

    // ask the RPLIDAR for its health info
    u_result getHealth(rplidar_response_device_health_t & healthinfo, _u32 timeout = RPLIDAR_DEFAULT_TIMEOUT);
    
    // ask the RPLIDAR for its device info like the serial number
    u_result getDeviceInfo(rplidar_response_device_info_t & info, _u32 timeout = RPLIDAR_DEFAULT_TIMEOUT);

    // stop the measurement operation
    u_result stop();

    // start the measurement operation
    u_result startScan(bool force = false, _u32 timeout = RPLIDAR_DEFAULT_TIMEOUT*2);

    // wait for one sample point to arrive
    u_result waitPoint(_u32 timeout = RPLIDAR_DEFAULT_TIMEOUT);
    
    // retrieve currently received sample point
    
    const RPLidarMeasurement & getCurrentPoint()
    {
        return _currentMeasurement;
    }

protected:
    u_result _sendCommand(_u8 cmd, const void * payload, size_t payloadsize);
    u_result _waitResponseHeader(rplidar_ans_header_t * header, _u32 timeout);

protected:
    HardwareSerial * _bined_serialdev;  
    RPLidarMeasurement _currentMeasurement;
};



/////////////////////#include "rplidar_cmd.h"/////////////////////////

RPLidar::RPLidar()
    : _bined_serialdev(NULL)
{
    _currentMeasurement.distance = 0;
    _currentMeasurement.angle = 0;
    _currentMeasurement.quality = 0;
    _currentMeasurement.startBit = 0;
}


RPLidar::~RPLidar()
{
    end();
}

// open the given serial interface and try to connect to the RPLIDAR
bool RPLidar::begin(HardwareSerial &serialobj)
{
    if (isOpen()) {
      end(); 
    }
    _bined_serialdev = &serialobj;
    _bined_serialdev->end();
    _bined_serialdev->begin(RPLIDAR_SERIAL_BAUDRATE);
}

// close the currently opened serial interface
void RPLidar::end()
{
    if (isOpen()) {
       _bined_serialdev->end();
       _bined_serialdev = NULL;
    }
}


// check whether the serial interface is opened
bool RPLidar::isOpen()
{
    return _bined_serialdev?true:false; 
}

// ask the RPLIDAR for its health info
u_result RPLidar::getHealth(rplidar_response_device_health_t & healthinfo, _u32 timeout)
{
    _u32 currentTs = millis();
    _u32 remainingtime;
  
    _u8 *infobuf = (_u8 *)&healthinfo;
    _u8 recvPos = 0;

    rplidar_ans_header_t response_header;
    u_result  ans;


    if (!isOpen()) return RESULT_OPERATION_FAIL;

    {
        if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_GET_DEVICE_HEALTH, NULL, 0))) {
            return ans;
        }

        if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
            return ans;
        }

        // verify whether we got a correct header
        if (response_header.type != RPLIDAR_ANS_TYPE_DEVHEALTH) {
            return RESULT_INVALID_DATA;
        }

        if ((response_header.size) < sizeof(rplidar_response_device_health_t)) {
            return RESULT_INVALID_DATA;
        }
        
        while ((remainingtime=millis() - currentTs) <= timeout) {
            int currentbyte = _bined_serialdev->read();
            if (currentbyte < 0) continue;
            
            infobuf[recvPos++] = currentbyte;

            if (recvPos == sizeof(rplidar_response_device_health_t)) {
                return RESULT_OK;
            }
        }
    }
    return RESULT_OPERATION_TIMEOUT;
}

// ask the RPLIDAR for its device info like the serial number
u_result RPLidar::getDeviceInfo(rplidar_response_device_info_t & info, _u32 timeout )
{
    _u8  recvPos = 0;
    _u32 currentTs = millis();
    _u32 remainingtime;
    _u8 *infobuf = (_u8*)&info;
    rplidar_ans_header_t response_header;
    u_result  ans;

    if (!isOpen()) return RESULT_OPERATION_FAIL;

    {
        if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_GET_DEVICE_INFO,NULL,0))) {
            return ans;
        }

        if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
            return ans;
        }

        // verify whether we got a correct header
        if (response_header.type != RPLIDAR_ANS_TYPE_DEVINFO) {
            return RESULT_INVALID_DATA;
        }

        if (response_header.size < sizeof(rplidar_response_device_info_t)) {
            return RESULT_INVALID_DATA;
        }

        while ((remainingtime=millis() - currentTs) <= timeout) {
            int currentbyte = _bined_serialdev->read();
            if (currentbyte<0) continue;    
            infobuf[recvPos++] = currentbyte;

            if (recvPos == sizeof(rplidar_response_device_info_t)) {
                return RESULT_OK;
            }
        }
    }
    
    return RESULT_OPERATION_TIMEOUT;
}

// stop the measurement operation
u_result RPLidar::stop()
{
    if (!isOpen()) return RESULT_OPERATION_FAIL;
    u_result ans = _sendCommand(RPLIDAR_CMD_STOP,NULL,0);
    return ans;
}

// start the measurement operation
u_result RPLidar::startScan(bool force, _u32 timeout)
{
    u_result ans;

    if (!isOpen()) return RESULT_OPERATION_FAIL;
    
    stop(); //force the previous operation to stop

    {
        ans = _sendCommand(force?RPLIDAR_CMD_FORCE_SCAN:RPLIDAR_CMD_SCAN, NULL, 0);
        if (IS_FAIL(ans)) return ans;

        // waiting for confirmation
        rplidar_ans_header_t response_header;
        if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
            return ans;
        }

        // verify whether we got a correct header
        if (response_header.type != RPLIDAR_ANS_TYPE_MEASUREMENT) {
            return RESULT_INVALID_DATA;
        }

        if (response_header.size < sizeof(rplidar_response_measurement_node_t)) {
            return RESULT_INVALID_DATA;
        }
    }
    return RESULT_OK;
}

// wait for one sample point to arrive
u_result RPLidar::waitPoint(_u32 timeout)
{
   _u32 currentTs = millis();
   _u32 remainingtime;
   rplidar_response_measurement_node_t node;
   _u8 *nodebuf = (_u8*)&node;

   _u8 recvPos = 0;

   while ((remainingtime=millis() - currentTs) <= timeout) {
        int currentbyte = _bined_serialdev->read();
        if (currentbyte<0) continue;

        switch (recvPos) {
            case 0: // expect the sync bit and its reverse in this byte          {
                {
                    _u8 tmp = (currentbyte>>1);
                    if ( (tmp ^ currentbyte) & 0x1 ) {
                        // pass
                    } else {
                        continue;
                    }

                }
                break;
            case 1: // expect the highest bit to be 1
                {
                    if (currentbyte & RPLIDAR_RESP_MEASUREMENT_CHECKBIT) {
                        // pass
                    } else {
                        recvPos = 0;
                        continue;
                    }
                }
                break;
          }
          nodebuf[recvPos++] = currentbyte;

          if (recvPos == sizeof(rplidar_response_measurement_node_t)) {
              // store the data ...
              _currentMeasurement.distance = node.distance_q2/4.0f;
              _currentMeasurement.angle = (node.angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;
              _currentMeasurement.quality = (node.sync_quality>>RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
              _currentMeasurement.startBit = (node.sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT);
              return RESULT_OK;
          }
        
   }

   return RESULT_OPERATION_TIMEOUT;
}



u_result RPLidar::_sendCommand(_u8 cmd, const void * payload, size_t payloadsize)
{

    rplidar_cmd_packet_t pkt_header;
    rplidar_cmd_packet_t * header = &pkt_header;
    _u8 checksum = 0;

    if (payloadsize && payload) {
        cmd |= RPLIDAR_CMDFLAG_HAS_PAYLOAD;
    }

    header->syncByte = RPLIDAR_CMD_SYNC_BYTE;
    header->cmd_flag = cmd;

    // send header first
    _bined_serialdev->write( (uint8_t *)header, 2);

    if (cmd & RPLIDAR_CMDFLAG_HAS_PAYLOAD) {
        checksum ^= RPLIDAR_CMD_SYNC_BYTE;
        checksum ^= cmd;
        checksum ^= (payloadsize & 0xFF);

        // calc checksum
        for (size_t pos = 0; pos < payloadsize; ++pos) {
            checksum ^= ((_u8 *)payload)[pos];
        }

        // send size
        _u8 sizebyte = payloadsize;
        _bined_serialdev->write((uint8_t *)&sizebyte, 1);

        // send payload
        _bined_serialdev->write((uint8_t *)&payload, sizebyte);

        // send checksum
        _bined_serialdev->write((uint8_t *)&checksum, 1);

    }

    return RESULT_OK;
}

u_result RPLidar::_waitResponseHeader(rplidar_ans_header_t * header, _u32 timeout)
{
    _u8  recvPos = 0;
    _u32 currentTs = millis();
    _u32 remainingtime;
    _u8 *headerbuf = (_u8*)header;
    while ((remainingtime=millis() - currentTs) <= timeout) {
        
        int currentbyte = _bined_serialdev->read();
        if (currentbyte<0) continue;
        switch (recvPos) {
        case 0:
            if (currentbyte != RPLIDAR_ANS_SYNC_BYTE1) {
                continue;
            }
            break;
        case 1:
            if (currentbyte != RPLIDAR_ANS_SYNC_BYTE2) {
                recvPos = 0;
                continue;
            }
            break;
        }
        headerbuf[recvPos++] = currentbyte;

        if (recvPos == sizeof(rplidar_ans_header_t)) {
            return RESULT_OK;
        }
  

    }

    return RESULT_OPERATION_TIMEOUT;
}
