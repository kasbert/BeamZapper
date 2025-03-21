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

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <string.h>
#include "cwiid.h"

int cwiid_command(cwiid_wiimote_t *wiimote, enum cwiid_command command,
                  int flags) {
	int ret;

	switch (command) {
	case CWIID_CMD_STATUS:
		ret = cwiid_request_status(wiimote);
		break;
	case CWIID_CMD_LED:
		ret = cwiid_set_led(wiimote, flags);
		break;
	case CWIID_CMD_RUMBLE:
		ret = cwiid_set_rumble(wiimote, flags);
		break;
	case CWIID_CMD_RPT_MODE:
		ret = cwiid_set_rpt_mode(wiimote, flags);
		break;
	default:
		ret = -1;
		break;
	}

	return ret;
}

/* TODO: fix error reporting - this is public now and
 * should report its own errors */

#include <esp_log.h>
#include "esp_hidh_api.h"
#include "esp_hidh.h"

const char *TAG = "cwiid";

int cwiid_send_rpt(cwiid_wiimote_t *wiimote, uint8_t flags, uint8_t report_id,
	size_t len, const void *data)
{
 size_t map_index = 0;
 /*
 Report Map Length: 217
   VENDOR   INPUT REPORT, ID:  63, Length:  21
   VENDOR   INPUT REPORT, ID:  62, Length:  21
   VENDOR   INPUT REPORT, ID:  61, Length:  21
   VENDOR   INPUT REPORT, ID:  55, Length:  21
   VENDOR   INPUT REPORT, ID:  54, Length:  21
   VENDOR   INPUT REPORT, ID:  53, Length:  21
   VENDOR   INPUT REPORT, ID:  52, Length:  21
   VENDOR   INPUT REPORT, ID:  51, Length:  17
   VENDOR   INPUT REPORT, ID:  50, Length:  10
   VENDOR   INPUT REPORT, ID:  49, Length:   5
   VENDOR   INPUT REPORT, ID:  48, Length:   2
   VENDOR   INPUT REPORT, ID:  34, Length:   4
   VENDOR   INPUT REPORT, ID:  33, Length:  21
   VENDOR   INPUT REPORT, ID:  32, Length:   6
   VENDOR  OUTPUT REPORT, ID:  26, Length:   1
   VENDOR  OUTPUT REPORT, ID:  25, Length:   1
   VENDOR  OUTPUT REPORT, ID:  24, Length:  21
   VENDOR  OUTPUT REPORT, ID:  23, Length:   6
   VENDOR  OUTPUT REPORT, ID:  22, Length:  21
   VENDOR  OUTPUT REPORT, ID:  21, Length:   1
   VENDOR  OUTPUT REPORT, ID:  20, Length:   1
   VENDOR  OUTPUT REPORT, ID:  19, Length:   1
   VENDOR  OUTPUT REPORT, ID:  18, Length:   2
   VENDOR  OUTPUT REPORT, ID:  17, Length:   1
  GAMEPAD  OUTPUT REPORT, ID:  16, Length:   1

 */
	ESP_LOGI(TAG, "Send report %02x %s", report_id, rpt2str(report_id));

	int ret= esp_hidh_dev_set_report(wiimote->dev, map_index,  report_id, ESP_HID_REPORT_TYPE_OUTPUT, (uint8_t*)data, len);
	vTaskDelay(CONFIG_FREERTOS_HZ / 10); // 0.1s FIXME
	//printf("RET %d\n", ret);
	return ret;
}

 
#if 0
int cwiid_send_rpt(cwiid_wiimote_t *wiimote, uint8_t flags, uint8_t report,
                   size_t len, const void *data)
{
	unsigned char *buf;

	if ((buf = malloc((len+2) * sizeof *buf)) == NULL) {
		cwiid_err(wiimote, "Memory allocation error (mesg array)");
		return -1;
	}

	buf[0] = BT_TRANS_SET_REPORT | BT_PARAM_OUTPUT;
	// FIXME
	buf[0] = 0xA0 | BT_PARAM_OUTPUT;
	
	buf[1] = report;
	memcpy(buf+2, data, len);
	if (!(flags & CWIID_SEND_RPT_NO_RUMBLE)) {
		buf[2] |= wiimote->state.rumble;
	}

	len += 2;
	if (ctl_write_hid(wiimote->cwiid, wiimote->connection_handle, buf, len) < 0) {
		free(buf);
		return -1;
	}
	free(buf);

	return 0;
}
#endif

int cwiid_request_status(cwiid_wiimote_t *wiimote)
{
	unsigned char data;

	data = 0;
	if (cwiid_send_rpt(wiimote, 0, RPT_STATUS_REQ, 1, &data)) {
		cwiid_err(wiimote, "Status request error");
		return -1;
	}

	return 0;
}

int cwiid_set_led(cwiid_wiimote_t *wiimote, uint8_t led)
{
	unsigned char data;

	/* TODO: assumption: char assignments are atomic, no mutex lock needed */
	wiimote->state.led = led & 0x0F;
	data = wiimote->state.led << 4;
	if (cwiid_send_rpt(wiimote, 0, RPT_LED_RUMBLE, 1, &data)) {
		cwiid_err(wiimote, "Report send error (led)");
		return -1;
	}

	return 0;
}

int cwiid_set_rumble(cwiid_wiimote_t *wiimote, uint8_t rumble)
{
	unsigned char data;

	/* TODO: assumption: char assignments are atomic, no mutex lock needed */
	wiimote->state.rumble = rumble ? 1 : 0;
	data = wiimote->state.led << 4;
	if (cwiid_send_rpt(wiimote, 0, RPT_LED_RUMBLE, 1, &data)) {
		cwiid_err(wiimote, "Report send error (led)");
		return -1;
	}

	return 0;
}

int cwiid_set_rpt_mode(cwiid_wiimote_t *wiimote, uint8_t rpt_mode)
{
	return update_rpt_mode(wiimote, rpt_mode);
}

#define RPT_READ_REQ_LEN 6
int cwiid_read(cwiid_wiimote_t *wiimote, uint8_t flags, uint32_t offset,
               uint16_t len, void *data)
{
	unsigned char buf[RPT_READ_REQ_LEN];
	struct rw_mesg mesg;
	unsigned char *cursor;
	int ret = 0;

	/* Compose read request packet */
	buf[0]=flags & (CWIID_RW_EEPROM | CWIID_RW_REG);
	buf[1]=(unsigned char)((offset>>16) & 0xFF);
	buf[2]=(unsigned char)((offset>>8) & 0xFF);
	buf[3]=(unsigned char)(offset & 0xFF);
	buf[4]=(unsigned char)((len>>8) & 0xFF);
	buf[5]=(unsigned char)(len & 0xFF);

	/* Lock wiimote rw access */

	/* Setup read info */
	wiimote->rw_status = RW_READ;

	/* TODO: Document: user is responsible for ensuring that read/write
	 * operations are not in flight while disconnecting.  Nothing serious,
	 * just accesses to freed memory */
	/* Send read request packet */
	if (cwiid_send_rpt(wiimote, 0, RPT_READ_REQ, RPT_READ_REQ_LEN, buf)) {
		cwiid_err(wiimote, "Report send error (read)");
		ret = -1;
		goto CODA;
	}

	/* TODO:Better sanity checks (offset) */
	/* Read packets */
	for (cursor = data; cursor - (unsigned char *)data < len;
	     cursor += mesg.len) {

			/*
		if (xQueueReceive(wiimote->rw_queue, &mesg, portMAX_DELAY) != pdTRUE) {
			cwiid_err(wiimote, "Queue read error (rw)");
			ret = -1;
			goto CODA;
		}
			*/
		if (mesg.type == RW_CANCEL) {
			ret = -1;
			goto CODA;
		}
		else if (mesg.type != RW_READ) {
			cwiid_err(wiimote, "Unexpected write message");
			ret = -1;
			goto CODA;
		}

		if (mesg.error) {
			cwiid_err(wiimote, "Wiimote read error");
			ret = -1;
			goto CODA;
		}

		memcpy(cursor, &mesg.data, mesg.len);
	}

CODA:
	/* Clear rw_status */
	wiimote->rw_status = RW_IDLE;

	return ret;
}

#include <esp_log.h>

#define RPT_WRITE_LEN 21
int cwiid_write(cwiid_wiimote_t *wiimote, uint8_t flags, uint32_t offset,
                  uint16_t len, const void *data)
{
	unsigned char buf[RPT_WRITE_LEN];
	uint16_t sent=0;
	int ret = 0;

	/* Compose write packet header */
	buf[0]=flags;

	/* Lock wiimote rw access */

	/* Send packets */
	wiimote->rw_status = RW_WRITE;
	while (sent<len) {
		/* Compose write packet */
		ESP_LOGD("cwiid", "cwiid_write %d %d", (int)sent, (int)len);
		buf[1]=(unsigned char)(((offset+sent)>>16) & 0xFF);
		buf[2]=(unsigned char)(((offset+sent)>>8) & 0xFF);
		buf[3]=(unsigned char)((offset+sent) & 0xFF);
		if (len-sent >= 0x10) {
			buf[4]=(unsigned char)0x10;
		}
		else {
			buf[4]=(unsigned char)(len-sent);
		}
		memcpy(buf+5, data+sent, buf[4]);

		if (cwiid_send_rpt(wiimote, 0, RPT_WRITE, RPT_WRITE_LEN, buf)) {
			cwiid_err(wiimote, "Report send error (write)");
			ret = -1;
			goto CODA;
		}

	#if 0
		struct rw_mesg mesg;
		/* Read packets from queue */
		if (xQueueReceive(wiimote->rw_queue, &mesg, portMAX_DELAY) != pdTRUE) {
			cwiid_err(wiimote, "Queue read error (rw queue)");
			ret = -1;
			goto CODA;
		}


		if (mesg.type == RW_CANCEL) {
			ret = -1;
			goto CODA;
		}
		else if (mesg.type != RW_WRITE) {
			cwiid_err(wiimote, "Unexpected read message");
			ret = -1;
			goto CODA;
		}

		if (mesg.error) {
			cwiid_err(wiimote, "Wiimote write error");
			ret = -1;
			goto CODA;
		};

		#endif
		sent+=buf[4];
	}
	return 0;

CODA:
	/* Clear rw_status */
	wiimote->rw_status = RW_IDLE;

	return ret;
}


struct write_seq speaker_enable_seq[] = {
	{WRITE_SEQ_RPT, RPT_SPEAKER_ENABLE, (const void *)"\x04", 1, 0},
	{WRITE_SEQ_RPT,   RPT_SPEAKER_MUTE, (const void *)"\x04", 1, 0},
	{WRITE_SEQ_MEM, 0xA20009, (const void *)"\x01", 1, CWIID_RW_REG},
	{WRITE_SEQ_MEM, 0xA20001, (const void *)"\x08", 1, CWIID_RW_REG},
	{WRITE_SEQ_MEM, 0xA20001, (const void *)"\x00\x00\x00\x0C\x40\x00\x00",
	                          7, CWIID_RW_REG},
	{WRITE_SEQ_MEM, 0xA20008, (const void *)"\x01", 1, CWIID_RW_REG},
	{WRITE_SEQ_RPT,   RPT_SPEAKER_MUTE, (const void *)"\x00", 1, 0}
};

struct write_seq speaker_disable_seq[] = {
	{WRITE_SEQ_RPT,   RPT_SPEAKER_MUTE, (const void *)"\x04", 1, 0},
	{WRITE_SEQ_RPT, RPT_SPEAKER_ENABLE, (const void *)"\x00", 1, 0}
};

#define SOUND_BUF_LEN	21
int cwiid_beep(cwiid_wiimote_t *wiimote)
{
	/* unsigned char buf[SOUND_BUF_LEN] = { 0xA0, 0xCC, 0x33, 0xCC, 0x33,
	    0xCC, 0x33, 0xCC, 0x33, 0xCC, 0x33, 0xCC, 0x33, 0xCC, 0x33, 0xCC, 0x33,
	    0xCC, 0x33, 0xCC, 0x33}; */
	unsigned char buf[SOUND_BUF_LEN] = { 0xA0, 0xC3, 0xC3, 0xC3, 0xC3,
	    0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3,
	    0xC3, 0xC3, 0xC3, 0xC3};
	int i;
	int ret = 0;
	struct timespec t;

	if (exec_write_seq(wiimote, SEQ_LEN(speaker_enable_seq),
	                   speaker_enable_seq)) {
		cwiid_err(wiimote, "Speaker enable error");
		ret = -1;
	}

	for (i=0; i<100; i++) {
		clock_gettime(CLOCK_REALTIME, &t);
		t.tv_nsec += 10204081;
		/* t.tv_nsec += 7000000; */
		if (cwiid_send_rpt(wiimote, 0, RPT_SPEAKER_DATA, SOUND_BUF_LEN, buf)) {
		 	//printf("%d\n", i);
			cwiid_err(wiimote, "Report send error (speaker data)");
			ret = -1;
			break;
		}
		/* TODO: I should be shot for this, but hey, it works.
		 * longterm - find a better wait */
	}

	if (exec_write_seq(wiimote, SEQ_LEN(speaker_disable_seq),
	                   speaker_disable_seq)) {
		cwiid_err(wiimote, "Speaker disable error");
		ret = -1;
	}

	return ret;
}
