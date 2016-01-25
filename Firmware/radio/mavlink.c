// -*- Mode: C; c-basic-offset: 8; -*-
//
// Copyright (c) 2012 Andrew Tridgell, All Rights Reserved
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  o Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  o Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.
//

///
/// @file	mavlink.c
///
/// mavlink reporting code
///

#include <stdarg.h>
#include "radio.h"
#include "packet.h"
#include "timer.h"

extern __xdata uint8_t pbuf[MAX_PACKET_LENGTH];
static __pdata uint8_t seqnum;


//Special MSP message inserted by the radio
#define MSP_RADIO 199

/*
 * Calculates the MSP checksum on a packet in pbuf[] 
 * and append it after the data
 */
static void msp_crc()
{
	register uint8_t length = pbuf[3];	//Payload length
        __pdata uint8_t sum = 0;
	__pdata uint8_t i, crcpos;

	crcpos = length + 5;   //start from zero... 1byte length -> pos6


       for (i=3;i<crcpos;i++)	//start from 4th byte (since $M> does not crc'd)
	    sum ^= pbuf[i];


	pbuf[crcpos] = sum;
}


//this structure is the same as mavlink uses for radio_status

struct msp_radio {
	uint16_t rxerrors;
	uint16_t fixed;
	uint8_t rssi;
	uint8_t remrssi;
	uint8_t txbuf;
	uint8_t noise;
	uint8_t remnoise;
};

/*
static void swap_bytes(__pdata uint8_t ofs, __pdata uint8_t len)
{
	register uint8_t i;
	for (i=ofs; i<ofs+len; i+=2) {
		register uint8_t tmp = pbuf[i];
		pbuf[i] = pbuf[i+1];
		pbuf[i+1] = tmp;
	}
}
*/

/// send a MAVLink status report packet
void MAVLink_report(void)
{
        struct msp_radio *m = (struct msp_radio *)&pbuf[5];
	pbuf[0] = '$';
	pbuf[1] = 'M';
	pbuf[2] = '>';
	pbuf[3] = sizeof(struct msp_radio);
	pbuf[4] = MSP_RADIO;

        m->rxerrors = errors.rx_errors;
        m->fixed    = errors.corrected_packets;
        m->txbuf    = serial_read_space();
        m->rssi     = statistics.average_rssi;
        m->remrssi  = remote_statistics.average_rssi;
        m->noise    = statistics.average_noise;
        m->remnoise = remote_statistics.average_noise;

	msp_crc();

	if (serial_write_space() < sizeof(struct msp_radio)+6) {
		// don't cause an overflow
		return;
	}

	serial_write_buf(pbuf, sizeof(struct msp_radio)+6);
}
