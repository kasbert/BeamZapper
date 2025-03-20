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

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "cwiid.h"

#include "esp_log.h"

cwiid_err_t cwiid_err_default;

static cwiid_err_t *cwiid_err_func = &cwiid_err_default;

int cwiid_set_err(cwiid_err_t *err)
{
	/* TODO: assuming pointer assignment is atomic operation */
	/* if it is, and the user doesn't care about race conditions, we don't
	 * either */
	cwiid_err_func = err;
	return 0;
}

void cwiid_err_default(struct wiimote *wiimote, const char *str, va_list ap)
{
	esp_log_writev(ESP_LOG_ERROR, "cwiid", str, ap);
	//vfprintf(stderr, str, ap);
	//fprintf(stderr, "\n");
}

void cwiid_err(struct wiimote *wiimote, const char *str, ...)
{
	va_list ap;

	if (cwiid_err_func) {
		va_start(ap, str);
		(*cwiid_err_func)(wiimote, str, ap);
		va_end(ap);
	}
}

int exec_write_seq(struct wiimote *wiimote, unsigned int len,
                   struct write_seq *seq)
{
	wiimote->write_seq_len = 0; // there should be locking...
	wiimote->write_seq = seq;
	wiimote->write_seq_index = 0;
	wiimote->write_seq_len = len;

	return exec_write_seq_continue(wiimote);
}

/* Send one message at a time */
int exec_write_seq_continue(struct wiimote *wiimote) {

	while (wiimote->write_seq_index < wiimote->write_seq_len) {
		int i = wiimote->write_seq_index;
		wiimote->write_seq_index++;
		switch (wiimote->write_seq[i].type) {
		case WRITE_SEQ_RPT:
			if (cwiid_send_rpt(wiimote, wiimote->write_seq[i].flags, wiimote->write_seq[i].report_offset,
				wiimote->write_seq[i].len, wiimote->write_seq[i].data)) {
				return -1;
			}
			break; //  continue sending
		case WRITE_SEQ_MEM:
			if (cwiid_write(wiimote, wiimote->write_seq[i].flags, wiimote->write_seq[i].report_offset,
				wiimote->write_seq[i].len, wiimote->write_seq[i].data)) {
				return -1;
			}
			return 0; // wait for ack
		}
	}

	return 0;
}
