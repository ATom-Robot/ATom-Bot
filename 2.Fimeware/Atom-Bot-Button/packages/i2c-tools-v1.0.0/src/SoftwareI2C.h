/*
  SoftwareI2C.h
  2012 Copyright (c) Seeed Technology Inc.  All right reserved.

  Author:Loovee
  2013-11-1

  This is a Software I2C Library, can act as I2c master mode.
  
  
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _SOFTWAREI2C_H_
#define _SOFTWAREI2C_H_

#include <rtthread.h>
#include <rtdevice.h>

#define  GETACK             1       // get ack
#define  GETNAK             0       // get nak
   
#ifndef  HIGH
    #define  HIGH           1
#endif
#ifndef  LOW
    #define  LOW            0
#endif

class SoftwareI2C
{
private:

    int pinSda;
    int pinScl;

    int recv_len;

    int sda_in_out;

private:

    inline void sdaSet(rt_uint8_t ucDta); 
    inline void sclSet(rt_uint8_t ucDta);

    inline void sendStart(void);
    inline void sendStop(void);
    inline void sendByte(rt_uint8_t ucDta);
    inline rt_uint8_t getAck(void);
    inline rt_uint8_t sendByteAck(rt_uint8_t ucDta);    // send byte and get ack

    inline unsigned long micros();

public:

    void begin(int Sda, int Scl); 
    rt_uint8_t beginTransmission(rt_uint8_t addr);
    rt_uint8_t endTransmission();

    rt_uint8_t write(rt_uint8_t dta);
    rt_uint8_t write(rt_uint8_t *dta, rt_uint8_t len);
    rt_uint8_t read();
    rt_uint8_t requestFrom(rt_uint8_t addr, rt_uint8_t len);
    rt_uint8_t available(){return recv_len;}
};

#endif
