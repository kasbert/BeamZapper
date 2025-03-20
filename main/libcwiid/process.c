/* Copyright (C) 2007 L. Donnie Smith <donnie.smith@gatech.edu>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <unistd.h>
#include <string.h>
#include "cwiid.h"

const char *rpt2str(int c) {
	switch (c) {
			case RPT_LED_RUMBLE: return "RPT_LED_RUMBLE";
			case RPT_RPT_MODE: return "RPT_RPT_MODE";
			case RPT_IR_ENABLE1: return "RPT_IR_ENABLE1";
			case RPT_SPEAKER_ENABLE: return "RPT_SPEAKER_ENABLE";
			case RPT_STATUS_REQ: return "RPT_STATUS_REQ";
			case RPT_WRITE: return "RPT_WRITE";
			case RPT_READ_REQ: return "RPT_READ_REQ";
			case RPT_SPEAKER_DATA: return "RPT_SPEAKER_DATA";
			case RPT_SPEAKER_MUTE: return "RPT_SPEAKER_MUTE";
			case RPT_IR_ENABLE2: return "RPT_IR_ENABLE2";
			case RPT_STATUS: return "RPT_STATUS";
			case RPT_READ_DATA: return "RPT_READ_DATA";
			case RPT_WRITE_ACK: return "RPT_WRITE_ACK";
			case RPT_BTN: return "RPT_BTN";
			case RPT_BTN_ACC: return "RPT_BTN_ACC";
			case RPT_BTN_EXT8: return "RPT_BTN_EXT8";
			case RPT_BTN_ACC_IR12: return "RPT_BTN_ACC_IR12";
			case RPT_BTN_EXT19: return "RPT_BTN_EXT19";
			case RPT_BTN_ACC_EXT16: return "RPT_BTN_ACC_EXT16";
			case RPT_BTN_IR10_EXT9: return "RPT_BTN_IR10_EXT9";
			case RPT_BTN_ACC_IR10_EXT6: return "RPT_BTN_ACC_IR10_EXT6";
			case RPT_EXT21: return "RPT_EXT21";
			case RPT_BTN_ACC_IR36_1: return "RPT_BTN_ACC_IR36_1";
			case RPT_BTN_ACC_IR36_2: return "RPT_BTN_ACC_IR36_2";
					}
	return "*** UNKNOWN RPT ***";
}


void parse_input_msg(struct wiimote *wiimote, uint8_t report_id, unsigned char *buf, ssize_t len, struct mesg_array *ma) {
	char err = 0;
	
	ma->count = 0;

	switch (report_id) {
	case RPT_STATUS:
		err = process_status(wiimote, buf, ma);
		break;
	case RPT_BTN:
		err = process_btn(wiimote, buf, ma);
		break;
	case RPT_BTN_ACC:
		err = process_btn(wiimote, buf, ma) ||
				process_acc(wiimote, buf + 2, ma);
		break;
	case RPT_BTN_EXT8:
		err = process_btn(wiimote, buf, ma) ||
				process_ext(wiimote, buf + 2, 8, ma);
		break;
	case RPT_BTN_ACC_IR12:
		err = process_btn(wiimote, buf, ma) ||
				process_acc(wiimote, buf + 2, ma) ||
				process_ir12(wiimote, buf + 5, ma);
		break;
	case RPT_BTN_EXT19:
		err = process_btn(wiimote, buf, ma) ||
				process_ext(wiimote, buf + 2, 19, ma);
		break;
	case RPT_BTN_ACC_EXT16:
		err = process_btn(wiimote, buf, ma) ||
				process_acc(wiimote, buf + 2, ma) ||
				process_ext(wiimote, buf + 5, 16, ma);
		break;
	case RPT_BTN_IR10_EXT9:
		err = process_btn(wiimote, buf, ma)  ||
				process_ir10(wiimote, buf + 2, ma) ||
				process_ext(wiimote, buf + 12, 9, ma);
		break;
	case RPT_BTN_ACC_IR10_EXT6:
		err = process_btn(wiimote, buf, ma)  ||
				process_acc(wiimote, buf + 2, ma)  ||
				process_ir10(wiimote, buf + 5, ma) ||
				process_ext(wiimote, buf + 15, 6, ma);
		break;
	case RPT_EXT21:
		err = process_ext(wiimote, buf, 21, ma);
		break;
	case RPT_BTN_ACC_IR36_1:
	case RPT_BTN_ACC_IR36_2:
		cwiid_err(wiimote, "Unsupported report type received "
							"(interleaved data)");
		err = 1;
		break;
	case RPT_READ_DATA:
		err = process_read(wiimote, buf + 2) ||
				process_btn(wiimote, buf, ma);
		break;
	case RPT_WRITE_ACK:
		err = process_write(wiimote, buf);
		break;
	default:
		cwiid_err(wiimote, "Unknown message type");
		err = 1;
		break;
	}

	if (!err && (ma->count > 0)) {
		if (update_state(wiimote, ma)) {
			cwiid_err(wiimote, "State update error");
		}
	}
}



int process_error(struct wiimote *wiimote, ssize_t len, struct mesg_array *ma)
{
	struct cwiid_error_mesg *error_mesg;

	error_mesg = &ma->array[ma->count++].error_mesg;
	error_mesg->type = CWIID_MESG_ERROR;
	if (len == 0) {
		error_mesg->error = CWIID_ERROR_DISCONNECT;
	}
	else {
		error_mesg->error = CWIID_ERROR_COMM;
	}

	/*
	if (cancel_rw(wiimote)) {
		cwiid_err(wiimote, "RW cancel error");
	}
		*/

	return 0;
}

int process_status(struct wiimote *wiimote, const unsigned char *data,
                   struct mesg_array *ma)
{
	struct cwiid_status_mesg *status_mesg;
	status_mesg = &ma->array[ma->count++].status_mesg;
	status_mesg->type = CWIID_MESG_STATUS;
	status_mesg->battery = data[5];
	if (data[2] & 0x02) {
		/* status_thread will figure out what it is */
		status_mesg->ext_type = CWIID_EXT_UNKNOWN;
	}
	else {
		status_mesg->ext_type = CWIID_EXT_NONE;
	}
	return 0;
}

int process_btn(struct wiimote *wiimote, const unsigned char *data,
                struct mesg_array *ma)
{
	struct cwiid_btn_mesg *btn_mesg;
	uint16_t buttons;

	buttons = (data[0] & BTN_MASK_0)<<8 |
	          (data[1] & BTN_MASK_1);
	if (wiimote->state.rpt_mode & CWIID_RPT_BTN) {
		if ((wiimote->state.buttons != buttons) ||
		  (wiimote->flags & CWIID_FLAG_REPEAT_BTN)) {
			btn_mesg = &ma->array[ma->count++].btn_mesg;
			btn_mesg->type = CWIID_MESG_BTN;
			btn_mesg->buttons = buttons;
		}
	}

	return 0;
}

int process_acc(struct wiimote *wiimote, const unsigned char *data,
                struct mesg_array *ma)
{
	struct cwiid_acc_mesg *acc_mesg;

	if (wiimote->state.rpt_mode & CWIID_RPT_ACC) {
		acc_mesg = &ma->array[ma->count++].acc_mesg;
		acc_mesg->type = CWIID_MESG_ACC;
		acc_mesg->acc[CWIID_X] = data[0];
		acc_mesg->acc[CWIID_Y] = data[1];
		acc_mesg->acc[CWIID_Z] = data[2];
	}

	return 0;
}

int process_ir10(struct wiimote *wiimote, const unsigned char *data,
                 struct mesg_array *ma)
{
	struct cwiid_ir_mesg *ir_mesg;
	int i;
	const unsigned char *block;

	if (wiimote->state.rpt_mode & CWIID_RPT_IR) {
		ir_mesg = &ma->array[ma->count++].ir_mesg;
		ir_mesg->type = CWIID_MESG_IR;

		for (i=0, block=data; i < CWIID_IR_SRC_COUNT; i+=2, block+=5) {
			if (block[0] == 0xFF) {
				ir_mesg->src[i].valid = 0;
			}
			else {
				ir_mesg->src[i].valid = 1;
				ir_mesg->src[i].pos[CWIID_X] = ((uint16_t)block[2] & 0x30)<<4 |
				                                (uint16_t)block[0];
				ir_mesg->src[i].pos[CWIID_Y] = ((uint16_t)block[2] & 0xC0)<<2 |
				                                (uint16_t)block[1];
				ir_mesg->src[i].size = -1;
			}

			if (block[3] == 0xFF) {
				ir_mesg->src[i+1].valid = 0;
			}
			else {
				ir_mesg->src[i+1].valid = 1;
				ir_mesg->src[i+1].pos[CWIID_X] =
				                               ((uint16_t)block[2] & 0x03)<<8 |
				                                (uint16_t)block[3];
				ir_mesg->src[i+1].pos[CWIID_Y] =
				                               ((uint16_t)block[2] & 0x0C)<<6 |
				                                (uint16_t)block[4];
				ir_mesg->src[i+1].size = -1;
			}
		}
	}

	return 0;
}

int process_ir12(struct wiimote *wiimote, const unsigned char *data,
                 struct mesg_array *ma)
{
	struct cwiid_ir_mesg *ir_mesg;
	int i;
	const unsigned char *block;

	if (wiimote->state.rpt_mode & CWIID_RPT_IR) {
		ir_mesg = &ma->array[ma->count++].ir_mesg;
		ir_mesg->type = CWIID_MESG_IR;

		for (i=0, block=data; i < CWIID_IR_SRC_COUNT; i++, block+=3) {
			if (block[0] == 0xFF) {
				ir_mesg->src[i].valid = 0;
			}
			else {
				ir_mesg->src[i].valid = 1;
				ir_mesg->src[i].pos[CWIID_X] = ((uint16_t)block[2] & 0x30)<<4 |
				                                (uint16_t)block[0];
				ir_mesg->src[i].pos[CWIID_Y] = ((uint16_t)block[2] & 0xC0)<<2 |
				                                (uint16_t)block[1];
				ir_mesg->src[i].size = block[2] & 0x0F;
			}
		}
	}

	return 0;
}

int process_ext(struct wiimote *wiimote, unsigned char *data,
                unsigned char len, struct mesg_array *ma)
{
	struct cwiid_nunchuk_mesg *nunchuk_mesg;
	struct cwiid_classic_mesg *classic_mesg;
	struct cwiid_balance_mesg *balance_mesg;
	struct cwiid_motionplus_mesg *motionplus_mesg;
	int i;

	switch (wiimote->state.ext_type) {
	case CWIID_EXT_NONE:
		cwiid_err(wiimote, "Received unexpected extension report");
		break;
	case CWIID_EXT_UNKNOWN:
		break;
	case CWIID_EXT_NUNCHUK:
		if (wiimote->state.rpt_mode & CWIID_RPT_NUNCHUK) {
			nunchuk_mesg = &ma->array[ma->count++].nunchuk_mesg;
			nunchuk_mesg->type = CWIID_MESG_NUNCHUK;
			nunchuk_mesg->stick[CWIID_X] = data[0];
			nunchuk_mesg->stick[CWIID_Y] = data[1];
			nunchuk_mesg->acc[CWIID_X] = data[2];
			nunchuk_mesg->acc[CWIID_Y] = data[3];
			nunchuk_mesg->acc[CWIID_Z] = data[4];
			nunchuk_mesg->buttons = ~data[5] & NUNCHUK_BTN_MASK;
		}
		break;
	case CWIID_EXT_CLASSIC:
		if (wiimote->state.rpt_mode & CWIID_RPT_CLASSIC) {
			classic_mesg = &ma->array[ma->count++].classic_mesg;
			classic_mesg->type = CWIID_MESG_CLASSIC;

			for (i=0; i < 6; i++) {
				data[i] = data[i];
			}

			classic_mesg->l_stick[CWIID_X] = data[0] & 0x3F;
			classic_mesg->l_stick[CWIID_Y] = data[1] & 0x3F;
			classic_mesg->r_stick[CWIID_X] = (data[0] & 0xC0)>>3 |
			                                 (data[1] & 0xC0)>>5 |
			                                 (data[2] & 0x80)>>7;
			classic_mesg->r_stick[CWIID_Y] = data[2] & 0x1F;
			classic_mesg->l = (data[2] & 0x60)>>2 |
			                  (data[3] & 0xE0)>>5;
			classic_mesg->r = data[3] & 0x1F;
			classic_mesg->buttons = ~((uint16_t)data[4]<<8 |
			                          (uint16_t)data[5]);
		}
		break;
	case CWIID_EXT_BALANCE:
		if (wiimote->state.rpt_mode & CWIID_RPT_BALANCE) {
			balance_mesg = &ma->array[ma->count++].balance_mesg;
			balance_mesg->type = CWIID_MESG_BALANCE;
			balance_mesg->right_top = ((uint16_t)data[0]<<8 |
			                           (uint16_t)data[1]);
			balance_mesg->right_bottom = ((uint16_t)data[2]<<8 |
			                              (uint16_t)data[3]);
			balance_mesg->left_top = ((uint16_t)data[4]<<8 |
			                          (uint16_t)data[5]);
			balance_mesg->left_bottom = ((uint16_t)data[6]<<8 |
			                             (uint16_t)data[7]);
		}
		break;
	case CWIID_EXT_MOTIONPLUS:
		if (wiimote->state.rpt_mode & CWIID_RPT_MOTIONPLUS) {
			motionplus_mesg = &ma->array[ma->count++].motionplus_mesg;
			motionplus_mesg->type = CWIID_MESG_MOTIONPLUS;
			motionplus_mesg->angle_rate[CWIID_PHI]   = ((uint16_t)data[5] & 0xFC)<<6 |
			                                            (uint16_t)data[2];
			motionplus_mesg->angle_rate[CWIID_THETA] = ((uint16_t)data[4] & 0xFC)<<6 |
			                                            (uint16_t)data[1];
			motionplus_mesg->angle_rate[CWIID_PSI]   = ((uint16_t)data[3] & 0xFC)<<6 |
			                                            (uint16_t)data[0];
			motionplus_mesg->low_speed[CWIID_PHI]    = ((uint8_t)data[3] & 0x01);
			motionplus_mesg->low_speed[CWIID_THETA]  = ((uint8_t)data[4] & 0x02)>>1;
			motionplus_mesg->low_speed[CWIID_PSI]    = ((uint8_t)data[3] & 0x02)>>1;
		}
		break;
	}

	return 0;
}

int process_read(struct wiimote *wiimote, unsigned char *data)
{
	struct rw_mesg rw_mesg;

	if (wiimote->rw_status != RW_READ) {
		cwiid_err(wiimote, "Received unexpected read report");
		return -1;
	}
	rw_mesg.type = RW_READ;
	rw_mesg.len = (data[0]>>4)+1;
	rw_mesg.error = data[0] & 0x0F;
	memcpy(&rw_mesg.data, data+3, rw_mesg.len);

	/*
    if (xQueueSend(wiimote->rw_queue, &rw_mesg, portMAX_DELAY) != pdPASS) {
		cwiid_err(wiimote, "RW queue overflow");
        return -1;
    }
		*/
	return 0;
}

int process_write(struct wiimote *wiimote, unsigned char *data)
{

	if (wiimote->rw_status != RW_WRITE) {
		cwiid_err(wiimote, "Received unexpected write report");
		return -1;
	}
	exec_write_seq_continue(wiimote);
	return 0;
}
