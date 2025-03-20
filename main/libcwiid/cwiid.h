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

#ifndef CWIID_H
#define CWIID_H

#include <stdarg.h>
#include <stdint.h>
#include <time.h>
#include <string.h>

/* Flags */
#define CWIID_FLAG_MESG_IFC		0x01
#define CWIID_FLAG_CONTINUOUS	0x02
#define CWIID_FLAG_REPEAT_BTN	0x04
//#define CWIID_FLAG_NONBLOCK		0x08
#define CWIID_FLAG_MOTIONPLUS	0x10

/* Report Mode Flags */
#define CWIID_RPT_STATUS		0x01
#define CWIID_RPT_BTN			0x02
#define CWIID_RPT_ACC			0x04
#define CWIID_RPT_IR			0x08
#define CWIID_RPT_NUNCHUK		0x10
#define CWIID_RPT_CLASSIC		0x20
#define CWIID_RPT_BALANCE		0x40
#define CWIID_RPT_MOTIONPLUS	0x80
#define CWIID_RPT_EXT		(CWIID_RPT_NUNCHUK | CWIID_RPT_CLASSIC | \
                             CWIID_RPT_BALANCE | CWIID_RPT_MOTIONPLUS)

/* LED flags */
#define CWIID_LED1_ON	0x01
#define CWIID_LED2_ON	0x02
#define CWIID_LED3_ON	0x04
#define CWIID_LED4_ON	0x08

/* Button flags */
#define CWIID_BTN_2		0x0001
#define CWIID_BTN_1		0x0002
#define CWIID_BTN_B		0x0004
#define CWIID_BTN_A		0x0008
#define CWIID_BTN_MINUS	0x0010
#define CWIID_BTN_HOME	0x0080
#define CWIID_BTN_LEFT	0x0100
#define CWIID_BTN_RIGHT	0x0200
#define CWIID_BTN_DOWN	0x0400
#define CWIID_BTN_UP	0x0800
#define CWIID_BTN_PLUS	0x1000

#define CWIID_NUNCHUK_BTN_Z	0x01
#define CWIID_NUNCHUK_BTN_C	0x02

#define CWIID_CLASSIC_BTN_UP	0x0001
#define CWIID_CLASSIC_BTN_LEFT	0x0002
#define CWIID_CLASSIC_BTN_ZR	0x0004
#define CWIID_CLASSIC_BTN_X		0x0008
#define CWIID_CLASSIC_BTN_A		0x0010
#define CWIID_CLASSIC_BTN_Y		0x0020
#define CWIID_CLASSIC_BTN_B		0x0040
#define CWIID_CLASSIC_BTN_ZL	0x0080
#define CWIID_CLASSIC_BTN_R		0x0200
#define CWIID_CLASSIC_BTN_PLUS	0x0400
#define CWIID_CLASSIC_BTN_HOME	0x0800
#define CWIID_CLASSIC_BTN_MINUS	0x1000
#define CWIID_CLASSIC_BTN_L		0x2000
#define CWIID_CLASSIC_BTN_DOWN	0x4000
#define CWIID_CLASSIC_BTN_RIGHT	0x8000

/* Send Report flags */
#define CWIID_SEND_RPT_NO_RUMBLE    0x01

/* Data Read/Write flags */
#define CWIID_RW_EEPROM	0x00
#define CWIID_RW_REG	0x04
#define CWIID_RW_DECODE	0x00

/* Maximum Data Read Length */
#define CWIID_MAX_READ_LEN	0xFFFF

/* Array Index Defs */
#define CWIID_X		0
#define CWIID_Y		1
#define CWIID_Z		2
#define CWIID_PHI	0
#define CWIID_THETA	1
#define CWIID_PSI	2

/* Acc Defs */
#define CWIID_ACC_MAX	0xFF

/* IR Defs */
#define CWIID_IR_SRC_COUNT	4
#define CWIID_IR_X_MAX		1024
#define CWIID_IR_Y_MAX		768

/* Battery */
#define CWIID_BATTERY_MAX	0xD0

/* Classic Controller Maxes */
#define CWIID_CLASSIC_L_STICK_MAX	0x3F
#define CWIID_CLASSIC_R_STICK_MAX	0x1F
#define CWIID_CLASSIC_LR_MAX	0x1F

/* Environment Variables */
#define WIIMOTE_BDADDR	"WIIMOTE_BDADDR"

/* Callback Maximum Message Count */
#define CWIID_MAX_MESG_COUNT	5

/* Enumerations */
enum cwiid_command {
	CWIID_CMD_STATUS,
	CWIID_CMD_LED,
	CWIID_CMD_RUMBLE,
	CWIID_CMD_RPT_MODE
};

enum cwiid_mesg_type {
	CWIID_MESG_STATUS,
	CWIID_MESG_BTN,
	CWIID_MESG_ACC,
	CWIID_MESG_IR,
	CWIID_MESG_NUNCHUK,
	CWIID_MESG_CLASSIC,
	CWIID_MESG_BALANCE,
	CWIID_MESG_MOTIONPLUS,
	CWIID_MESG_ERROR,
	CWIID_MESG_UNKNOWN
};

enum cwiid_ext_type {
	CWIID_EXT_NONE,
	CWIID_EXT_NUNCHUK,
	CWIID_EXT_CLASSIC,
	CWIID_EXT_BALANCE,
	CWIID_EXT_MOTIONPLUS,
	CWIID_EXT_UNKNOWN
};

enum cwiid_error {
	CWIID_ERROR_NONE,
	CWIID_ERROR_DISCONNECT,
	CWIID_ERROR_COMM
};

struct acc_cal {
	uint8_t zero[3];
	uint8_t one[3];
};

struct balance_cal {
	uint16_t right_top[3];
	uint16_t right_bottom[3];
	uint16_t left_top[3];
	uint16_t left_bottom[3];
};

/* Message Structs */
struct cwiid_status_mesg {
	enum cwiid_mesg_type type;
	uint8_t battery;
	enum cwiid_ext_type ext_type;
};	

struct cwiid_btn_mesg {
	enum cwiid_mesg_type type;
	uint16_t buttons;
};

struct cwiid_acc_mesg {
	enum cwiid_mesg_type type;
	uint8_t acc[3];
};

struct cwiid_ir_src {
	char valid;
	uint16_t pos[2];
	int8_t size;
};

struct cwiid_ir_mesg {
	enum cwiid_mesg_type type;
	struct cwiid_ir_src src[CWIID_IR_SRC_COUNT];
};

struct cwiid_nunchuk_mesg {
	enum cwiid_mesg_type type;
	uint8_t stick[2];
	uint8_t acc[3];
	uint8_t buttons;
};

struct cwiid_classic_mesg {
	enum cwiid_mesg_type type;
	uint8_t l_stick[2];
	uint8_t r_stick[2];
	uint8_t l;
	uint8_t r;
	uint16_t buttons;
};

struct cwiid_balance_mesg {
	enum cwiid_mesg_type type;
	uint16_t right_top;
	uint16_t right_bottom;
	uint16_t left_top;
	uint16_t left_bottom;
};

struct cwiid_motionplus_mesg {
	enum cwiid_mesg_type type;
	uint16_t angle_rate[3];
	uint8_t low_speed[3];
};

struct cwiid_error_mesg {
	enum cwiid_mesg_type type;
	enum cwiid_error error;
};

union cwiid_mesg {
	enum cwiid_mesg_type type;
	struct cwiid_status_mesg status_mesg;
	struct cwiid_btn_mesg btn_mesg;
	struct cwiid_acc_mesg acc_mesg;
	struct cwiid_ir_mesg ir_mesg;
	struct cwiid_nunchuk_mesg nunchuk_mesg;
	struct cwiid_classic_mesg classic_mesg;
	struct cwiid_balance_mesg balance_mesg;
	struct cwiid_motionplus_mesg motionplus_mesg;
	struct cwiid_error_mesg error_mesg;
};

/* State Structs */
struct nunchuk_state {
	uint8_t stick[2];
	uint8_t acc[3];
	uint8_t buttons;
};

struct classic_state {
	uint8_t l_stick[2];
	uint8_t r_stick[2];
	uint8_t l;
	uint8_t r;
	uint16_t buttons;
};

struct balance_state {
	uint16_t right_top;
	uint16_t right_bottom;
	uint16_t left_top;
	uint16_t left_bottom;
};

struct motionplus_state {
	uint16_t angle_rate[3];
	uint8_t low_speed[3];
};

union ext_state {
	struct nunchuk_state nunchuk;
	struct classic_state classic;
	struct balance_state balance;
	struct motionplus_state motionplus;
};

struct cwiid_state {
	uint8_t rpt_mode;
	uint8_t led;
	uint8_t rumble;
	uint8_t battery;
	uint16_t buttons;
	uint8_t acc[3];
	struct cwiid_ir_src ir_src[CWIID_IR_SRC_COUNT];
	enum cwiid_ext_type ext_type;
	union ext_state ext;
	enum cwiid_error error;
};

typedef enum cwiid_evt_type {
	CWIID_EVENT_INITIALIZE,
	CWIID_EVENT_SCAN_START,
	CWIID_EVENT_SCAN_STOP,
	CWIID_EVENT_CONNECT,
	CWIID_EVENT_DISCONNECT,

	CWIID_EVENT_DATA,
	CWIID_EVENT_IRCAMERA_INITIALIZE
} cwiid_evt_type_t;

/* Typedefs */
typedef struct wiimote cwiid_wiimote_t;

typedef void cwiid_mesg_callback_t(cwiid_wiimote_t *, int,
                                   union cwiid_mesg [], struct timespec *);

typedef void cwiid_evt_callback_t(cwiid_evt_type_t, struct timespec *);
 
typedef void cwiid_err_t(cwiid_wiimote_t *, const char *, va_list ap);

/* get_bdinfo */
#define BT_NO_WIIMOTE_FILTER 0x01
#define BT_NAME_LEN 32



#ifdef __cplusplus
extern "C" {
#endif

/* Error reporting (library wide) */
int cwiid_set_err(cwiid_err_t *err);
void cwiid_err_default(struct wiimote *wiimote, const char *str, va_list ap);


/* Connection */

/* Operations */
int cwiid_command(cwiid_wiimote_t *wiimote, enum cwiid_command command,
                  int flags);
int cwiid_send_rpt(cwiid_wiimote_t *wiimote, uint8_t flags, uint8_t report,
                   size_t len, const void *data);
int cwiid_request_status(cwiid_wiimote_t *wiimote);
int cwiid_set_led(cwiid_wiimote_t *wiimote, uint8_t led);
int cwiid_set_rumble(cwiid_wiimote_t *wiimote, uint8_t rumble);
int cwiid_set_rpt_mode(cwiid_wiimote_t *wiimote, uint8_t rpt_mode);
int cwiid_read(cwiid_wiimote_t *wiimote, uint8_t flags, uint32_t offset,
               uint16_t len, void *data);
int cwiid_write(cwiid_wiimote_t *wiimote, uint8_t flags, uint32_t offset,
                uint16_t len, const void *data);
/* int cwiid_beep(cwiid_wiimote_t *wiimote); */

#ifdef __cplusplus
}
#endif


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
 #include <sys/types.h>	/* ssize_t */
 
 /* Wiimote specific magic numbers */
 #define WIIMOTE_NAME "Nintendo RVL-CNT-01"
 #define WIIBALANCE_NAME "Nintendo RVL-WBC-01"
 #define WIIMOTE_CLASS_0 0x04
 #define WIIMOTE_CLASS_1 0x25
 #define WIIMOTE_CLASS_2 0x00
 
 /* Wiimote port/channel/PSMs */
 #define CTL_PSM	17
 #define INT_PSM	19
 
 /* Report numbers */
 #define RPT_LED_RUMBLE			0x11
 #define RPT_RPT_MODE			0x12
 #define RPT_IR_ENABLE1			0x13
 #define RPT_SPEAKER_ENABLE		0x14
 #define RPT_STATUS_REQ			0x15
 #define RPT_WRITE				0x16
 #define RPT_READ_REQ			0x17
 #define RPT_SPEAKER_DATA		0x18
 #define RPT_SPEAKER_MUTE		0x19
 #define RPT_IR_ENABLE2			0x1A
 #define RPT_STATUS				0x20
 #define RPT_READ_DATA			0x21
 #define RPT_WRITE_ACK			0x22
 #define RPT_BTN					0x30
 #define RPT_BTN_ACC				0x31
 #define RPT_BTN_EXT8			0x32
 #define RPT_BTN_ACC_IR12		0x33
 #define RPT_BTN_EXT19			0x34
 #define RPT_BTN_ACC_EXT16		0x35
 #define RPT_BTN_IR10_EXT9		0x36
 #define RPT_BTN_ACC_IR10_EXT6	0x37
 #define RPT_EXT21				0x3D
 #define RPT_BTN_ACC_IR36_1		0x3E
 #define RPT_BTN_ACC_IR36_2		0x3F
 
 /* Button Mask (masks unknown bits in button bytes) */
 #define BTN_MASK_0			0x1F
 #define BTN_MASK_1			0x9F
 #define NUNCHUK_BTN_MASK	0x03
 
 /* Extension Values */
 #define EXT_NONE		0x2E2E
 #define EXT_PARTIAL		0xFFFF
 #define EXT_NUNCHUK		0x0000
 #define EXT_CLASSIC		0x0101
 #define EXT_BALANCE		0x0402
 #define EXT_MOTIONPLUS	0x0405
 
 /* IR Enable blocks */
 #define MARCAN_IR_BLOCK_1			"\x00\x00\x00\x00\x00\x00\x90\x00\xC0"
 #define MARCAN_IR_BLOCK_2			"\x40\x00"
 #define CLIFF_IR_BLOCK_1			"\x02\x00\x00\x71\x01\x00\xAA\x00\x64"
 #define CLIFF_IR_BLOCK_2			"\x63\x03"
 #define MAX_SENSITIVITY_IR_BLOCK_1	"\x00\x00\x00\x00\x00\x00\x90\x00\x41"
 #define MAX_SENSITIVITY_IR_BLOCK_2	"\x40\x00"
 #define WII_L1_IR_BLOCK_1			"\x02\x00\x00\x71\x01\x00\x64\x00\xFE"
 #define WII_L1_IR_BLOCK_2			"\xFD\x05"
 #define WII_L2_IR_BLOCK_1			"\x02\x00\x00\x71\x01\x00\x96\x00\xB4"
 #define WII_L2_IR_BLOCK_2			"\xB3\x04"
 #define WII_L3_IR_BLOCK_1			"\x02\x00\x00\x71\x01\x00\xAA\x00\x64"
 #define WII_L3_IR_BLOCK_2			"\x63\x03"
 #define WII_L4_IR_BLOCK_1			"\x02\x00\x00\x71\x01\x00\xC8\x00\x36"
 #define WII_L4_IR_BLOCK_2			"\x35\x03"
 #define WII_L5_IR_BLOCK_1			"\x02\x00\x00\x71\x01\x00\x72\x00\x20"
 #define WII_L5_IR_BLOCK_2			"\x1F\x03"
 
 /* Write Sequences */
 enum write_seq_type {
	 WRITE_SEQ_RPT,
	 WRITE_SEQ_MEM
 };
 
 struct write_seq {
	 enum write_seq_type type;
	 uint32_t report_offset;
	 const void *data;
	 uint16_t len;
	 uint8_t flags;
 };
 
 #define SEQ_LEN(seq) (sizeof(seq)/sizeof(struct write_seq))
 
 /* Message arrays */
 struct mesg_array {
	 uint8_t count;
	 union cwiid_mesg array[CWIID_MAX_MESG_COUNT];
 };
 
 /* RW State/Mesg */
 enum rw_status {
	 RW_IDLE,
	 RW_READ,
	 RW_WRITE,
	 RW_CANCEL
 };
 
 struct rw_mesg {
	 enum rw_status type;
	 uint8_t error;
	 uint32_t offset;
	 uint8_t len;
	 char data[16];
 };
 
 // In esp hidd
 struct esp_hidh_dev_s;
 typedef struct esp_hidh_dev_s esp_hidh_dev_t;

 /* Wiimote struct */
 struct wiimote {
	 int flags;
	 struct cwiid_state state;
	 enum rw_status rw_status;
	 struct write_seq *write_seq;
	 int write_seq_index;
	 int write_seq_len;
	 esp_hidh_dev_t *dev;
 };
 
 
  /* prototypes */
  
 /* util.c */
void cwiid_err(struct wiimote *wiimote, const char *str, ...);
int exec_write_seq(struct wiimote *wiimote, unsigned int len,
				struct write_seq *seq);
int cancel_rw(struct wiimote *wiimote);
int exec_write_seq_continue(struct wiimote *wiimote);

/* process.c */
const char *rpt2str(int c);
void parse_input_msg(struct wiimote *wiimote, uint8_t report_id, unsigned char *buf, ssize_t len, struct mesg_array *ma);

int process_error(struct wiimote *, ssize_t, struct mesg_array *);
int process_status(struct wiimote *, const unsigned char *,
				struct mesg_array *);
int process_btn(struct wiimote *, const unsigned char *, struct mesg_array *);
int process_acc(struct wiimote *, const unsigned char *, struct mesg_array *);
int process_ir10(struct wiimote *, const unsigned char *, struct mesg_array *);
int process_ir12(struct wiimote *, const unsigned char *, struct mesg_array *);
int process_ext(struct wiimote *, unsigned char *, unsigned char,
			 struct mesg_array *);
int process_read(struct wiimote *, unsigned char *);
int process_write(struct wiimote *, unsigned char *);
 
/* state.c */
int update_state(struct wiimote *wiimote, struct mesg_array *ma);
int update_rpt_mode(struct wiimote *wiimote, int8_t rpt_mode);
 
#endif
