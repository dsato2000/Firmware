
#pragma once

#include <stdint.h>

#include "crsf.h"
#include "dsm.h"
#include "sbus.h"
#include "st24.h"
#include "sumd.h"

#pragma pack(push, 1)
typedef  struct rc_decode_buf_ {
	union {
		crsf_frame_t crsf_frame;
		dsm_decode_t dsm;
		sbus_frame_t sbus_frame;
		ReceiverFcPacket _strxpacket;
		ReceiverFcPacketHoTT _hottrxpacket;
	};
} rc_decode_buf_t;

#define	SBUS_CHANNEL_MAX	16
typedef struct rc_out_ {
	uint16_t	pwm_value[SBUS_CHANNEL_MAX];
	uint8_t		servo_count;
} rc_out_t;
#pragma pack(pop)

extern rc_decode_buf_t rc_decode_buf;

#ifdef SBUS_OUTPUT_ENABLE
extern rc_out_t rc_out;
#endif
