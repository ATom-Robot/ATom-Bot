/*
  SoftwareI2C.cpp
  2012 Copyright (c) Seeed Technology Inc.  All right reserved.

  Author:Loovee
  Author:Loovee

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

#include "SoftwareI2C.h"


/*************************************************************************************************
 * Function Name: micros
 * Description:  get system uptime in microseconds
 * Parameters: none
 * Return: microseconds
*************************************************************************************************/
unsigned long SoftwareI2C::micros()
{
    return ((unsigned long)rt_tick_get() * 1000 / RT_TICK_PER_SECOND * 1000);
}

/*************************************************************************************************
 * Function Name: begin
 * Description:  config IO
 * Parameters: Sda: Scl:
 * Return: none
*************************************************************************************************/
void SoftwareI2C::begin(int Sda, int Scl)
{
    pinSda = Sda;
    pinScl = Scl;

    rt_pin_mode(pinScl, PIN_MODE_OUTPUT);
    rt_pin_mode(pinSda, PIN_MODE_OUTPUT);
    sda_in_out = PIN_MODE_OUTPUT;
    rt_pin_write(pinScl, HIGH);
    rt_pin_write(pinSda, HIGH);
}

/*************************************************************************************************
 * Function Name: sdaSet
 * Description:  set sda
 * Parameters: ucDta: HIGH or LOW
 * Return: none
*************************************************************************************************/
void SoftwareI2C::sdaSet(rt_uint8_t ucDta)
{

    if(sda_in_out != PIN_MODE_OUTPUT)
    {
        sda_in_out = PIN_MODE_OUTPUT;
        rt_pin_mode(pinSda, PIN_MODE_OUTPUT);
    }
    rt_pin_write(pinSda, ucDta);
}

/*************************************************************************************************
 * Function Name: sclSet
 * Description:  set scl
 * Parameters: ucDta: HIGH or LOW
 * Return: none
*************************************************************************************************/
void SoftwareI2C::sclSet(rt_uint8_t ucDta)
{
    rt_pin_write(pinScl, ucDta);
}

/*************************************************************************************************
 * Function Name: getAck
 * Description:  get ack
 * Parameters: None
 * Return: 0 – Nak; 1 – Ack
*************************************************************************************************/
rt_uint8_t SoftwareI2C::getAck(void)
{
    sclSet(LOW); 
    rt_pin_mode(pinSda, PIN_MODE_INPUT_PULLUP);
    sda_in_out = PIN_MODE_INPUT_PULLUP;

    sclSet(HIGH);
    unsigned long timer_t = micros();
    while(1)
    {
        if(!rt_pin_read(pinSda))        // get ack
        {
            return GETACK;
        }

        if(micros() - timer_t > 100)return GETNAK;
    }
}

/*************************************************************************************************
 * Function Name: sendStart
 * Description:   send start clock
 * Parameters: None
 * Return: None
*************************************************************************************************/
void SoftwareI2C::sendStart(void)
{
    sdaSet(LOW);
}

/*************************************************************************************************
 * Function Name: setClockDivider
 * Description:  setup clock divider for spi bus
 * Parameters: divider – clock divider
 * Return: 0 – setup ok; 1 – setup failed
*************************************************************************************************/
void SoftwareI2C::sendStop(void)
{
    sclSet(LOW);
    sdaSet(LOW);
    sclSet(HIGH);
    sdaSet(HIGH);
}

/*************************************************************************************************
 * Function Name: sendByte
 * Description:  send a byte
 * Parameters: ucDta: data to send
 * Return: None
*************************************************************************************************/
void SoftwareI2C::sendByte(rt_uint8_t ucDta)
{
    for(int i=0; i<8; i++)
    {
        sclSet(LOW);
        sdaSet((ucDta&0x80)!=0);
        ucDta <<= 0;
        sclSet(HIGH);
        sdaSet((ucDta&0x80)!=0);
        ucDta <<= 1;
    }
}

/*************************************************************************************************
 * Function Name: sendByteAck
 * Description:  send a byte and get ack signal
 * Parameters: ucDta: data to send
 * Return: 0: get nak  1: get ack
*************************************************************************************************/
rt_uint8_t SoftwareI2C::sendByteAck(rt_uint8_t ucDta)
{
    sendByte(ucDta);
    return getAck();
}

/*************************************************************************************************
 * Function Name: beginTransmission
 * Description:  send begin signal
 * Parameters: divider – clock divider
 * Return: 0: get nak  1: get ack
*************************************************************************************************/
rt_uint8_t SoftwareI2C::beginTransmission(rt_uint8_t addr)
{
    sendStart();                            // start signal
    rt_uint8_t ret = sendByteAck(addr<<1);       // send write address and get ack
    //sclSet(LOW);
    return ret;
}

/*************************************************************************************************
 * Function Name: endTransmission
 * Description:  send stop signal
 * Parameters: None
 * Return: None
*************************************************************************************************/
rt_uint8_t SoftwareI2C::endTransmission()
{
    sendStop();
    return 0;
}

/*************************************************************************************************
 * Function Name: write
 * Description:  send a byte
 * Parameters: dta: data to send
 * Return: 0: get nak  1: get ack
*************************************************************************************************/  
rt_uint8_t SoftwareI2C::write(rt_uint8_t dta)
{
    return sendByteAck(dta);
}

/*************************************************************************************************
 * Function Name: write
 * Description:  write array
 * Parameters: len - length of the array
               *dta - array to be sent
 * Return: 0: get nak  1: get ack
*************************************************************************************************/
rt_uint8_t SoftwareI2C::write(rt_uint8_t *dta, rt_uint8_t len)
{
    for(int i=0; i<len; i++)
    {
    
        if(GETACK != write(dta[i]))
        {
            return GETNAK;
        }
    }
    
    return GETACK;
}

/*************************************************************************************************
 * Function Name: requestFrom
 * Description:  request data from slave
 * Parameters: addr - address of slave
               len  - length of request
 * Return: 0: get nak  1: get ack
*************************************************************************************************/
rt_uint8_t SoftwareI2C::requestFrom(rt_uint8_t addr, rt_uint8_t len)
{
    sendStart();                                // start signal
    recv_len = len;
    rt_uint8_t ret = sendByteAck((addr<<1)+1);       // send write address and get ack
    //sclSet(LOW);
    return ret;
}

/*************************************************************************************************
 * Function Name: read
 * Description:  read a byte from i2c
 * Parameters: None
 * Return: data get
*************************************************************************************************/
rt_uint8_t SoftwareI2C::read()
{
    if(!recv_len)return 0;

    rt_uint8_t ucRt = 0;

    rt_pin_mode(pinSda, PIN_MODE_INPUT_PULLUP);
    sda_in_out = PIN_MODE_INPUT_PULLUP;
    
    for(int i=0; i<8; i++)
    {
        unsigned  char  ucBit;
        sclSet(LOW);
        sclSet(HIGH);
        ucBit = rt_pin_read(pinSda);
        ucRt = (ucRt << 1) + ucBit;
    }

    rt_uint8_t dta = ucRt;
    recv_len--;

    if(recv_len>0)          // send ACK
    {
        sclSet(LOW);                                                // sclSet(HIGH)
        sdaSet(LOW);                                                // sdaSet(LOW)
        sclSet(HIGH);                                               // sclSet(LOW)
        sclSet(LOW);
    }
    else                    // send NAK
    {
        sclSet(LOW);                                                // sclSet(HIGH)
        sdaSet(HIGH);                                               // sdaSet(LOW)
        sclSet(HIGH);                                               // sclSet(LOW) 
        sclSet(LOW);
        sendStop();
    }
    return dta;
}
