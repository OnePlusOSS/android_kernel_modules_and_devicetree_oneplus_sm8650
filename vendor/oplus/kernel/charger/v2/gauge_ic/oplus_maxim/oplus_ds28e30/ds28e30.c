// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023-2023 Oplus. All rights reserved.
 */
#define pr_fmt(fmt) "[DS28E30]([%s][%d]): " fmt, __func__, __LINE__

#define ds28e30
#include "ds28e30.h"

#include <linux/slab.h> /* kfree() */
#include <linux/module.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/gpio/driver.h>
#include <linux/gpio/consumer.h>
#include <linux/list.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <oplus_chg_module.h>
#include <oplus_chg_ic.h>
#include "1wire_protocol.h"
#include "deep_cover_coproc.h"
#include "ecdsa_generic_api.h"
#include "ucl_defs.h"
#include "ucl_sha256.h"
#include "ecc_generate_key.h"

static int e30_test = 0;
module_param(e30_test, int, 0644);
MODULE_PARM_DESC(e30_test, "debug e30");

/* define retry times if an testing error is experienced */
#define RETRY_NUMBER	5
/* define testing number */
#define TESTING_ITEM_NUMBER	17
/* define testing item result */
#define FAMILY_CODE_RESULT	0
/* custom ID is special for each mobile maker */
#define CUSTOM_ID_RESULT	1
#define UNIQUE_ID_RESULT	2
#define MAN_ID_RESULT		3
#define STATUS_RESULT		4
#define PAGE0_RESULT		5
#define PAGE1_RESULT		6
#define PAGE2_RESULT		7
#define PAGE3_RESULT		8
#define COUNTERVALUE_RESULT	9
#define VERIFICATION_SIGNATURE_RESULT	10
#define VERIFICATION_CERTIFICATE_RESULT	11
#define PROGRAM_PAGE0_RESULT	12
#define PROGRAM_PAGE1_RESULT	13
#define PROGRAM_PAGE2_RESULT	14
#define PROGRAM_PAGE3_RESULT	15
#define DECREASINGCOUNTERVALUE_RESULT	16
#define CRC_CORRECT_BIT_NUMBER	3
short rbit_lowtime_buffer[CRC_CORRECT_BIT_NUMBER][2];
short wbit_lowtime_buffer[CRC_CORRECT_BIT_NUMBER][2];
unsigned short rbit_counter, wbit_counter;

unsigned char testingitemresult[TESTING_ITEM_NUMBER];   /* maximal testing items */

/* define specific definition for general-purpose DS28E30 */
/* definintion for MI */
#define GP_CID_LSB	0x00
#define GP_CID_MSB	0x00
#define GP_MAN_ID_LSB	0x00
#define GP_MAN_ID_MSB	0x00
#define GP_COUNTERVALUE_LSB	0xFF
#define GP_COUNTERVALUE_2LSB	0xFF
#define GP_COUNTERVALUE_MSB	0xFF

/* define constant for generating certificate */
/* for general-purpose DS28E30 */
/* TODO: use nvt battery system publickey */
/* define constant for generating/verifying default certificate of generic DS28E30 (T0 project stage) */
unsigned char gp_certificate_constant[16] = {0xEC, 0x81, 0x75, 0x28, 0x11, 0x24, 0x0D, 0x6F, 0x9F,
					0x30, 0xC8, 0x83, 0x0B, 0xFF, 0x53, 0xA0};

unsigned char gp_system_public_key_x[32] = {0x2E, 0x75, 0x76, 0xB1, 0x34, 0x3E, 0xF4, 0xE4,
					0xFB, 0x93, 0x69, 0x79, 0x2E, 0x7A, 0x2E, 0x83,
					0x97, 0x58, 0x14, 0xCA, 0x49, 0x95, 0x84, 0x84,
					0xD7, 0xFA, 0x3E, 0xB7, 0xA0, 0x65, 0x7C, 0x5C};
unsigned char gp_system_public_key_y[32] = {0x69, 0xC9, 0x37, 0xF4, 0xE0, 0x6E, 0x37, 0x1D,
					0xAF, 0x17, 0x52, 0x49, 0xF7, 0xD5, 0xCF, 0x4D,
					0x5C, 0xDF, 0x4F, 0xD2, 0x21, 0x0D, 0x20, 0x53,
					0x2D, 0x17, 0xA9, 0xF3, 0xBB, 0x08, 0x2B, 0xD2};
unsigned char  gp_authority_public_key_x[32] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
					0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
					0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
					0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
unsigned char  gp_authority_public_key_y[32] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
					0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
					0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
					0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

unsigned char gp_page_protection_status[11] = {0x00, 0, 0, 0, 0x00, 0x00, 0x00, 0x00, 0x02, 0x02, 0x03};

/* define specific definition for each customer */
#define OP_CID_LSB			0x10
#define OP_CID_MSB			0x06
#define OP_MAN_ID_LSB			0xEC
#define OP_MAN_ID_MSB			0x00
#define OP_COUNTERVALUE_LSB	0xFF
#define OP_COUNTERVALUE_2LSB	0xFF
#define OP_COUNTERVALUE_MSB	0xFF

/* define constant for generating certificate */
const unsigned char op_certificate_constant[16] = {0xA5, 0xDB, 0x67, 0xD0, 0xD6, 0x7A, 0x7A, 0xBF, 0x65,
					0x1B, 0x47, 0xF5, 0x59, 0xD7, 0xFE, 0x1A};
/* 32-byte system-level public key X */
const unsigned char op_system_public_key_x[32] = {0xDF, 0x47, 0x0F, 0xA1, 0xE3, 0xDB, 0xB9, 0x19,
					0x47, 0x33, 0xB0, 0x36, 0xCB, 0x83, 0x0A, 0x59,
					0x6D, 0xED, 0x66, 0xE6, 0x44, 0xB8, 0xC7, 0x89,
					0xE1, 0xA4, 0x1C, 0x1B, 0x0F, 0x33, 0xF5, 0xD0};
/* 32-byte system-level public key Y */
const unsigned char op_system_public_key_y[32] = {0x34, 0xB5, 0x54, 0xB1, 0x40, 0x9E, 0x95, 0x06,
					0x4B, 0x41, 0xBD, 0xCF, 0x60, 0x39, 0x65, 0x9A,
					0x3B, 0xDB, 0x0C, 0x98, 0xFD, 0x75, 0x7A, 0x11,
					0xB8, 0xC6, 0xF8, 0x85, 0x02, 0xE5, 0x75, 0xA3};
const unsigned char  op_authority_public_key_x[32] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
					0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
					0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
					0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
const unsigned char  op_authority_public_key_y[32] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
					0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
					0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
					0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

unsigned char op_page_protection_status[11] = {0x02, 0, 0, 0, 0x02, 0x02, 0x00, 0x00, 0x02, 0x02, 0x03};

bool configure_ds28e30_parameters(void);

/* define system-level publick key, authority public key  and certificate constant variables */
unsigned char system_public_key_x[32];
unsigned char system_public_key_y[32];
unsigned char authority_private_key[32];
unsigned char authority_public_key_x[32];
unsigned char authority_public_key_y[32];
unsigned char certificate_constant[16];
unsigned char expected_cid[2];
unsigned char expected_man_id[2];
unsigned char expected_page_protection_status[11];

/* Command Functions (no high level verification) */
int ds28e30_cmd_write_memory(int pg, u8 *data);
bool ds28e30_cmd_read_memory(int pg, u8 *data);
int ds28e30_cmd_read_status(int pg, u8 *pr_data, u8 *manid, u8 *hardware_version);
int ds28e30_cmd_set_page_protection(int pg, u8 prot);
int ds28e30_cmd_compute_read_page_authentication(int pg, int anon, u8 *challenge, u8 *sig);
int ds28e30_cmd_decrement_counter(void);
int ds28e30_cmd_device_disable(u8 *release_sequence);
int ds28e30_cmd_generate_ecdsa_key_pair(int use_puf, int lock_enable);
int ds28e30_cmd_verify_ecdsa_signature(u8 *sig_r, u8 *sig_s, u8 *custom_cert_fields, int cert_len);
int ds28e30_cmd_authendicated_ecdsa_write_memory(int pg, u8 *data, u8 *sig_r, u8 *sig_s);

/* High level functions */
int ds28e30_cmd_read_device_public_key(u8 *data);
int ds28e30_compute_verify_ecdsa(int pg, int anon, u8 *mempage, u8 *challenge, u8 *sig_r, u8 *sig_s);
int ds28e30_compute_verify_ecdsa_no_read(int pg, int anon, u8 *mempage, u8 *challenge, u8 *sig_r, u8 *sig_s);
/* DS28E30 application functions */
int verify_ecdsa_certificate_device(u8 *sig_r, u8 *sig_s, u8 *pub_key_x, u8 *pub_key_y, u8 *slave_romid,
				   u8 *slave_manid, u8 *system_level_pub_key_x, u8 *system_level_pub_key_y);
int ds28e30_write_memory_page_with_ecw(int pg, u8 *new_data);

/* Helper functions */
int standard_cmd_flow(u8 *write_buf, int write_len, int delayms, int expect_read_len, u8 *read_buf, int *read_len);
u8 ds28e30_get_last_result_byte(void);
void ds28e30_set_public_key(u8 *px, u8 *py);
void ds28e30_set_private_key(u8 *priv);
int ds28e30_read_romno_manid_hardware_version(void);
unsigned short docrc16(unsigned short data);
unsigned short crc16;
unsigned short wdcrc16;

/* int OWBlock(unsigned char *tran_buf, int tran_len); */
int ow_read_rom(void);
int ow_skip_rom(void);

/* misc utility functions */
unsigned char docrc8(unsigned char value);

/* ECDSA algorithm achieved by software */
int sw_compute_ecdsa_signature(u8 *message, int msg_len, u8 *sig_r, u8 *sig_s);

/* keys in byte array format, used by software compute functions */
u8 private_key[32];
u8 public_key_x[32];
u8 public_key_y[32];

/* ds28e30 state */
static u8 crc8;
u8 rom_no[8];
u8 man_id[2];
u8 hardware_version[2];
/* define global variables for authenticate ds28e30 */
unsigned char pagedata[4][32];
unsigned char device_publickey_x[32], device_publickey_y[32];
unsigned char page_certificate_r[32], page_certificate_s[32];
bool pagedata_read_status[4] = {false, false, false, false};
bool publickey_read_status = false;
bool certificate_read_status = false;
bool rom_no_manid_read_status = false;

/* last result byte */
u8 last_result_byte = RESULT_SUCCESS;

#ifdef SPIN_LOCK_ENABLE
struct mutex ds_cmd_lock;
#endif

/* ds28e30 Memory functions
 'Write Memory' command
 @param[in] pg
 page number to write
 @param[in] data
 buffer must be at least 32 bytes
 @return
 true - command successful @n
 false - command failed
 */
int ds28e30_cmd_write_memory(int pg, u8 *data)
{
	u8 write_buf[50];
	int write_len;
	u8 read_buf[255];
	int read_len;

	/*
		Reset
		Presence Pulse
		<ROM Select>
		TX: XPC Command (66h)
		TX: Length byte 34d
		TX: XPC sub-command 96h (Write Memory)
		TX: Parameter
		TX: New page data (32d bytes)
		RX: crc16 (inverted of XPC command, length, sub-command, parameter)
		TX: Release Byte
		<Delay TBD>
		RX: Dummy Byte
		RX: Length Byte (1d)
		RX: Result Byte
		RX: crc16 (inverted of length and result byte)
		Reset or send XPC command (66h) for a new sequence
	*/

	/* construct the write buffer */
	write_len = 0;
	write_buf[write_len++] = CMD_WRITE_MEM;
	write_buf[write_len++] = pg;
	memcpy(&write_buf[write_len], data, 32);
	write_len += 32;

	/* preload read_len with expected length */
	read_len = 1;

	/* default failure mode */
	last_result_byte = RESULT_FAIL_COMMUNICATION;

	if (standard_cmd_flow(write_buf, write_len, DELAY_DS28E30_EE_WRITE_TWM, read_len, read_buf, &read_len)) {
		/* get result byte */
		last_result_byte = read_buf[0];
		/* check result */
		if (read_len == 1)
			return (read_buf[0] == RESULT_SUCCESS);
	}

	/* no payload in read buffer or failed command */
	return false;
}

/*  'Read Memory' command
 @param[in] pg
 page number to read
 @param[out] data
 buffer length must be at least 32 bytes to hold memory read
 @return
 true - command successful @n
 false - command failed
 */
bool ds28e30_cmd_read_memory(int pg, u8 *data)
{
	u8 write_buf[10];
	int write_len;
	u8 read_buf[255];
	int read_len;
	unsigned char i, j, temp_read_data[MAX_TEMP_LENGTH];
	bool flag = false;

	/*
		Reset
		Presence Pulse
		<ROM Select>
		TX: XPC Command (66h)
		TX: Length byte 2d
		TX: XPC sub-command 69h (Read Memory)
		TX: Parameter (page)
		RX: crc16 (inverted of XPC command, length, sub-command, and parameter)
		TX: Release Byte
		<Delay TBD>
		RX: Dummy Byte
		RX: Length (33d)
		RX: Result Byte
		RX: Read page data (32d bytes)
		RX: crc16 (inverted, length byte, result byte, and page data)
		Reset or send XPC command (66h) for a new sequence
	*/

	/* construct the write buffer */
	write_len = 0;
	write_buf[write_len++] = CMD_READ_MEM;
	write_buf[write_len++] = pg;

	/* preload read_len with expected length */
	read_len = 33;

	memset(temp_read_data, 0xFF, (read_len + 2));
	for (i = 0; i < READ_MEMORY_RERTY_CNT; i++) {
		read_buf[0] = RESULT_FAIL_COMMUNICATION;
		flag = standard_cmd_flow(write_buf, write_len, DELAY_DS28E30_EE_READ_TRM, read_len, read_buf, &read_len);
		if (read_buf[0] == RESULT_FAIL_COMMUNICATION || read_len > READ_MEM_LENGTH)
			continue;
		if (flag == false) {
			for (j = 0; j < (read_len + 2); j++)
				temp_read_data[j] &= read_buf[j];	/* correct read data */
			crc16 = 0; /* check crc16 */
			docrc16(read_len);
			for (j = 0; j < (read_len + 2); j++)
				docrc16(temp_read_data[j]);
			if (crc16 != SKIP_CRC_CHECK)
				continue;
			memmove(read_buf, temp_read_data, read_len + 2);
			chg_info("crc16=b001, readMemory page[%d] data is corrected by the host successfully\n", write_buf[1]);
			memmove(data, &read_buf[1], PAG_DATE_LENGTH);
			flag = true;
		}
		break;
	}
	if (flag == true) {
		last_result_byte = read_buf[0]; /* get result byte */
		if (read_len == READ_MEM_LENGTH && read_buf[0] == RESULT_SUCCESS) {  /*check result */
			memmove(data, &read_buf[1], PAG_DATE_LENGTH);
			return true;
		}
	}
	return false;
}

/*  'Read Status' command
 @param[in] pg
 page to read protection
 @param[out] pr_data
 pointer to u8 buffer of length 6 for page protection data
 @param[out] manid
 pointer to u8 buffer of length 2 for manid (manufactorur ID)
 @return
 true - command successful @n
 false - command failed
 */

int ds28e30_cmd_read_status(int pg, u8 *pr_data, u8 *manid, u8 *hardware_version)
{
	u8 write_buf[10];
	u8 read_buf[255];
	int read_len = 2, write_len;

	/*
		Reset
		Presence Pulse
		<ROM Select>
		TX: XPC Command (66h)
		TX: Length byte 1d
		TX: XPC sub-command AAh (Read Status)
		RX: crc16 (inverted of XPC command, length, and sub-command)
		TX: Release Byte
		<Delay TBD>
		RX: Dummy Byte
		RX: Length Byte (11d)
		RX: Result Byte
		RX: Read protection values (6 Bytes), MANID (2 Bytes), ROM VERSION (2 bytes)
		RX: crc16 (inverted, length byte, protection values, MANID, ROM_VERSION)
		Reset or send XPC command (66h) for a new sequence
	*/

	/* construct the write buffer */
	write_len = 0;
	write_buf[write_len++] = CMD_READ_STATUS;
	write_buf[write_len++] = pg;

	/* preload read_len with expected length */
	if (pg & 0x80)
		read_len = 5;

	/* default failure mode */
	last_result_byte = RESULT_FAIL_COMMUNICATION;
	/* return standard_cmd_flow(write_buf, write_len, DELAY_DS28E30_EE_READ_TRM, read_len, read_buf, &read_len); */

	if (standard_cmd_flow(write_buf, write_len, DELAY_DS28E30_EE_READ_TRM, read_len, read_buf, &read_len)) {
		/* get result byte */
		last_result_byte = read_buf[0];
		/* should always be 2 or 5 length for status data */
		if (read_len == 2 || read_len == 5) {
			if (read_buf[0] == RESULT_SUCCESS || read_buf[0] == RESULT_DEVICE_DISABLED) {
				if (read_len == 2) {
					memcpy(pr_data, &read_buf[1], 1);
				} else {
					memcpy(manid, &read_buf[1], 2);
					memcpy(hardware_version, &read_buf[3], 2);
				}
				return true;
			}
		}
	}

	/* no payload in read buffer or failed command */
	return false;
}

/*  'Set Page Protection' command
 @param[in] pg
 page to set protection
 @param[in] prot
 protection value to set
 @return
 true - command successful @n
 false - command failed
 */
int ds28e30_cmd_set_page_protection(int pg, u8 prot)
{
	u8 write_buf[10];
	int write_len;
	u8 read_buf[255];
	int read_len;

	/*
		Reset
		Presence Pulse
		<ROM Select>
		TX: XPC Command (66h)
		TX: Length byte 3d
		TX: XPC sub-command C3h (Set Protection)
		TX: Parameter (page)
		TX: Parameter (protection)
		RX: crc16 (inverted of XPC command, length, sub-command, parameters)
		TX: Release Byte
		<Delay TBD>
		RX: Dummy Byte
		RX: Length Byte (1d)
		RX: Result Byte
		RX: crc16 (inverted, length byte and result byte)
		Reset or send XPC command (66h) for a new sequence
	*/

	/* construct the write buffer */
	write_len = 0;
	write_buf[write_len++] = CMD_SET_PAGE_PROT;
	write_buf[write_len++] = pg;
	write_buf[write_len++] = prot;

	/* preload read_len with expected length */
	read_len = 1;

	/* default failure mode */
	last_result_byte = RESULT_FAIL_COMMUNICATION;
	if (standard_cmd_flow(write_buf, write_len, DELAY_DS28E30_EE_WRITE_TWM, read_len, read_buf, &read_len)) {
		/* get result byte */
		last_result_byte = read_buf[0];
		/* check result */
		if (read_len == 1)
			return (read_buf[0] == RESULT_SUCCESS);
	}

	/* no payload in read buffer or failed command */
	return false;
}

/*  'Compute and Read Page Authentication' command
 @param[in] pg - page number to compute auth on
 @param[in] anon - anonymous flag (1) for anymous
 @param[in] challenge
 buffer length must be at least 32 bytes containing the challenge
 @param[out] data
 buffer length must be at least 64 bytes to hold ECDSA signature
 @return
 true - command successful @n
 false - command failed
 */
int ds28e30_cmd_compute_read_page_authentication(int pg, int anon, u8 *challenge, u8 *sig)
{
	u8 write_buf[200];
	int write_len;
	u8 read_buf[255];
	int read_len;

	/*
		Reset
		Presence Pulse
		<ROM Select>
		TX: XPC Command (66h)
		TX: Length byte 34d
		TX: XPC sub-command A5h (Compute and Read Page Authentication)
		TX: Parameter (page)
		TX: Challenge (32d bytes)
		RX: crc16 (inverted of XPC command, length, sub-command, parameter, and challenge)
		TX: Release Byte
		<Delay TBD>
		RX: Dummy Byte
		RX: Length byte (65d)
		RX: Result Byte
		RX: Read ECDSA Signature (64 bytes, s and then r, MSByte first, [same as ES10]),
			signature 00h's if result byte is not AA success
		RX: crc16 (inverted, length byte, result byte, and signature)
		Reset or send XPC command (66h) for a new sequence
	*/

	/* construct the write buffer */
	write_len = 0;
	write_buf[write_len++] = CMD_COMP_READ_AUTH;
	write_buf[write_len] = pg & 0x7f;
	if (anon)
		write_buf[write_len] |= 0xE0;
	write_len++;
	write_buf[write_len++] = 0x03; /* authentication parameter */
	memcpy(&write_buf[write_len], challenge, 32);
	write_len += 32;

	/* preload read_len with expected length */
	read_len = EXPECTED_LENGTH;

	/* default failure mode */
	last_result_byte = RESULT_FAIL_COMMUNICATION;

	if (standard_cmd_flow(write_buf, write_len, DELAY_DS28E30_ECDSA_GEN_TGES, read_len, read_buf, &read_len)) {
		/* get result byte */
		last_result_byte = read_buf[0];
		/* check result */
		if (read_len == EXPECTED_LENGTH) {
			if (read_buf[0] == RESULT_SUCCESS) {
				memcpy(sig, &read_buf[1], 64);
				return true;
			}
		}
	}

	/* no payload in read buffer or failed command */
	return false;
}

/*  'Decrement Counter' command
  @return
  true - command successful @n
  false - command failed
 */
int ds28e30_cmd_decrement_counter(void)
{
	int write_len;
	u8 write_buf[10];
	u8 read_buf[255];
	int read_len;

	/*
		Presence Pulse
		<ROM Select>
		TX: XPC Command (66h)
		TX: Length byte 1d
		TX: XPC sub-command C9h (Decrement Counter)
		RX: crc16 (inverted of XPC command, length, sub-command)
		TX: Release Byte
		<Delay TBD>
		RX: Dummy Byte
		RX: Length Byte (1d)
		RX: Result Byte
		RX: crc16 (inverted, length byte and result byte)
		Reset or send XPC command (66h) for a new sequence
	*/

	/* construct the write buffer */
	write_len = 0;
	write_buf[write_len++] = CMD_DECREMENT_CNT;

	/* preload read_len with expected length */
	read_len = 1;

	/* default failure mode  */
	last_result_byte = RESULT_FAIL_COMMUNICATION;

	if (standard_cmd_flow(write_buf, write_len, DELAY_DS28E30_EE_WRITE_TWM+50, read_len, read_buf, &read_len)) {
		/* get result byte */
		last_result_byte = read_buf[0];
		/* check result byte */
		if (read_len == 1)
			return (read_buf[0] == RESULT_SUCCESS);
	}

	/* no payload in read buffer or failed command */
	return false;
}

/*  'Device Disable' command
 @param[in] release_sequence
 8 byte release sequence to disable device
  @return
  true - command successful @n
  false - command failed
 */
int ds28e30_cmd_device_disable(u8 *release_sequence)
{
	u8 write_buf[10];
	int write_len;
	u8 read_buf[255];
	int read_len;

	/*
		Reset
		Presence Pulse
		<ROM Select>
		TX: XPC Command (66h)
		TX: Length byte 9d
		TX: XPC sub-command 33h (Disable command)
		TX: Release Sequence (8 bytes)
		RX: crc16 (inverted of XPC command, length, sub-command, and release sequence)
		TX: Release Byte
		<Delay TBD>
		RX: Dummy Byte
		RX: Length Byte (1d)
		RX: Result Byte
		RX: crc16 (inverted, length byte and result byte)
		Reset or send XPC command (66h) for a new sequence
	*/

	/* construct the write buffer */
	write_len = 0;
	write_buf[write_len++] = CMD_DISABLE_DEVICE;
	memcpy(&write_buf[write_len], release_sequence, 8);
	write_len += 8;

	/* preload read_len with expected length */
	read_len = 1;

	/* default failure mode  */
	last_result_byte = RESULT_FAIL_COMMUNICATION;

	if (standard_cmd_flow(write_buf, write_len, DELAY_DS28E30_EE_WRITE_TWM, read_len, read_buf, &read_len)) {
		/* get result byte */
		last_result_byte = read_buf[0];
		/* check result */
		if (read_len == 1)
			return (read_buf[0] == RESULT_SUCCESS);
	}

	/* no payload in read buffer or failed command */
	return false;
}

/*  'Read device public key' command
 @param[in]
 no param required
 @param[out] data
 buffer length must be at least 64 bytes to hold device public key read
  @return
  true - command successful @n
  false - command failed */

int ds28e30_cmd_read_device_public_key(u8 *data)
{
	if ((ds28e30_cmd_read_memory(PG_DS28E30_PUB_KEY_X, data)) == false)
		return false;
	if ((ds28e30_cmd_read_memory(PG_DS28E30_PUB_KEY_Y, data+32)) == false)
		return false;
	return true;
}

/*  'authenticated Write memory' command
 @param[in] pg
 page number to write
 @param[in] data
 buffer must be at least 32 bytes
 @param[in] certificate sig_r
 @param[in] certificate sig_s
  @return
  true - command successful @n
  false - command failed */

int ds28e30_cmd_authendicated_ecdsa_write_memory(int pg, u8 *data, u8 *sig_r, u8 *sig_s)
{
	u8 write_buf[128];
	int write_len;
	u8 read_buf[MAX_TEMP_LENGTH];
	int read_len;

	/*
		Reset
		Presence Pulse
		<ROM Select>
		TX: XPC Command (66h)
		TX: Length byte 98d
		TX: XPC sub-command 89h (authenticated Write Memory)
		TX: Parameter
		TX: New page data (32d bytes)
		TX: Certificate R&S (64 bytes)
		RX: crc16 (inverted of XPC command, length, sub-command, parameter, page data, certificate R&S)
		TX: Release Byte
		<Delay TBD>
		RX: Dummy Byte
		RX: Length Byte (1d)
		RX: Result Byte
		RX: crc16 (inverted of length and result byte)
		Reset or send XPC command (66h) for a new sequence
	*/

	/* construct the write buffer */
	write_len = 0;
	write_buf[write_len++] = CMD_AUTHENTICATE_WRITE;
	write_buf[write_len++] = pg & 0x03;
	memcpy(&write_buf[write_len], data, 32);
	write_len += 32;
	memcpy(&write_buf[write_len], sig_r, 32);
	write_len += 32;
	memcpy(&write_buf[write_len], sig_s, 32);
	write_len += 32;


	/* preload read_len with expected length */
	read_len = 1;

	/* default failure mode */
	last_result_byte = RESULT_FAIL_COMMUNICATION;

	if (standard_cmd_flow(write_buf, write_len, DELAY_DS28E30_EE_WRITE_TWM + DELAY_DS28E30_VERIFY_ECDSA_SIGNATURE_TEVS,
		read_len, read_buf, &read_len)) {
		/* get result byte */
		last_result_byte = read_buf[0];
		/* check result */
		if (read_len == 1)
			return (read_buf[0] == RESULT_SUCCESS);
	}

	/* no payload in read buffer or failed command */
	return false;
}

/*  High level function to do a full challenge/response ECDSA operation
 on specified page
 @param[in] pg
 page to do operation on
 @param[in] anon
 flag to indicate in anonymous mode (1) or not anonymous (0)
 @param[out] mempage
 buffer to return the memory page contents
 @param[in] challenge
 buffer containing challenge, must be 32 bytes
 @param[out] sig_r
 buffer for r portion of signature, must be 32 bytes
 @param[out] sig_s
 buffer for s portion of signature, must be 32 bytes
 @return
 true - command successful @n
 false - command failed
 */
int ds28e30_compute_verify_ecdsa(int pg, int anon, u8 *mempage, u8 *challenge, u8 *sig_r, u8 *sig_s)
{
	/* read destination page */
	if (!ds28e30_cmd_read_memory(pg, mempage))
		return false;

	return ds28e30_compute_verify_ecdsa_no_read(pg, anon, mempage, challenge, sig_r, sig_s);
}

/* High level function to do a full challenge/response ECDSA operation
 on specified page
 @param[in] pg
 page to do operation on
 @param[in] anon
 flag to indicate in anonymous mode (1) or not anonymous (0)
 @param[in] mempage
 buffer with memory page contents, required for verification of ECDSA signature
 @param[in] challenge
 buffer containing challenge, must be 32 bytes
 @param[out] sig_r
 buffer for r portion of signature, must be 32 bytes
 @param[out] sig_s
 buffer for s portion of signature, must be 32 bytes
 @return
 true - command successful @n
 false - command failed
 */
int ds28e30_compute_verify_ecdsa_no_read(int pg, int anon, u8 *mempage, u8 *challenge, u8 *sig_r, u8 *sig_s)
{
	u8 signature[64], message[256];
	int msg_len;
	u8 *pubkey_x, *pubkey_y;

	/* compute and read auth command */
	if (!ds28e30_cmd_compute_read_page_authentication(pg, anon, challenge, signature))
		return false;

	/* put the signature in the return buffers, signature is 's' and then 'r', MSByte first */
	memcpy(sig_s, signature, 32);
	memcpy(sig_r, &signature[32], 32);

	/* construct the message to hash for signature verification */
	/* ROM NO | Page Data | Challenge (Buffer) | Page# | MANID */
	/* ROM NO */
	msg_len = 0;
	if (anon)
		memset(&message[msg_len], 0xFF, 8);
	else
		memcpy(&message[msg_len], rom_no, 8);
	msg_len += 8;
	/* Page Data */
	memcpy(&message[msg_len], mempage, 32);
	msg_len += 32;
	/* Challenge (Buffer) */
	memcpy(&message[msg_len], challenge, 32);
	msg_len += 32;
	/* Page# */
	message[msg_len++] = pg;
	/* MANID */
	memcpy(&message[msg_len], man_id, 2);
	msg_len += 2;
	pubkey_x = public_key_x;
	pubkey_y = public_key_y;

	/* verify Signature and return result */
	return deep_cover_verify_ecdsa_signature(message, msg_len, pubkey_x, pubkey_y, sig_r, sig_s);
}

/*  Verify certificate of devices like DS28C36/DS28C36/DS28E38/DS28E30.
 @param[in] sig_r
 Buffer for R portion of certificate signature (MSByte first)
 @param[in] sig_s
 Buffer for S portion of certificate signature (MSByte first)
 @param[in] pub_x
 Public Key x to verify
 @param[in] pub_y
 Public Key y to verify
 @param[in] SLAVE_ROMID
 device's 64-bit ROMID (LSByte first)
 @param[in] SLAVE_MANID
 Maxim defined as manufacturing ID
 @param[in] system_level_pub_key_x
 32-byte buffer container the system level public key x
 @param[in] system_level_pub_key_y
 32-byte buffer container the system level public key y
  @return
  true - certificate valid @n
  false - certificate not valid
 */
int verify_ecdsa_certificate_device(u8 *sig_r, u8 *sig_s, u8 *pub_key_x, u8 *pub_key_y, u8 *slave_romid,
				   u8 *slave_manid, u8 *system_level_pub_key_x, u8 *system_level_pub_key_y)
{
	unsigned char buf[32];

	/* setup software ECDSA computation */
	deep_cover_coproc_setup(0, 0, 0, 0);
	/* create customization field */
	/* 16 zeros (can be set to other customer specific value) */
	memcpy(buf, certificate_constant, 16); /* TODO: */
	/* ROMID */
	memcpy(&buf[16], slave_romid, 8);
	/* MANID */
	memcpy(&buf[24], slave_manid, 2);
	return deep_cover_verify_ecdsa_certificate(sig_r, sig_s, pub_key_x, pub_key_y, buf,
						26, system_level_pub_key_x, system_level_pub_key_y);
}

/*
  setting expected MAN_ID, protection status, counter value, system-level public key,
  authority public key and certificate constants
*/
bool configure_ds28e30_parameters(void)
{
	unsigned short cid_value;
	bool flag = false;

	cid_value = rom_no[6] << 4;
	cid_value += (rom_no[5] & 0xF0) >> 4;
	chg_info("%s: cid_value: 0x%x\n", __func__, cid_value);

	switch (cid_value) {
	case 0x061:
		expected_cid[0] = OP_CID_LSB;
		expected_cid[1] = OP_CID_MSB;
		expected_man_id[0] = OP_MAN_ID_LSB;
		expected_man_id[1] = OP_MAN_ID_MSB;
		memcpy(expected_page_protection_status, op_page_protection_status, 11);
		memcpy(certificate_constant, op_certificate_constant, 16);
		memcpy(system_public_key_x, op_system_public_key_x, 32);
		memcpy(system_public_key_y, op_system_public_key_y, 32);
		memcpy(authority_public_key_x, op_authority_public_key_x, 32);
		memcpy(authority_public_key_y, op_authority_public_key_y, 32);
		flag = true;
		break;
	default:
		expected_cid[0] = GP_CID_LSB;
		expected_cid[1] = GP_CID_MSB;
		expected_man_id[0] = GP_MAN_ID_LSB;
		expected_man_id[1] = GP_MAN_ID_MSB;
		memcpy(expected_page_protection_status, gp_page_protection_status, 11);
		memcpy(certificate_constant, gp_certificate_constant, 16);
		memcpy(system_public_key_x, gp_system_public_key_x, 32);
		memcpy(system_public_key_y, gp_system_public_key_y, 32);
		memcpy(authority_public_key_x, gp_authority_public_key_x, 32);
		memcpy(authority_public_key_y, gp_authority_public_key_y, 32);
		break;
	}
	return flag;
}

#define HISTSOH_DATA_SIZE 16
#define HISTSOH_PAGE_SIZE 32
#define BATT_SN_SIZE 25
bool set_historic_soh_data_ds28e30(const int *set_soh_data)
{
	int flag;
	int i;
	u8 set_data[HISTSOH_PAGE_SIZE] = {0};

	if (set_soh_data == NULL) {
		chg_err("set_soh_data == NULL failed\n");
		return false;
	}

	chg_info("write pack historic_soh_data eeprom 2 is:");
	for (i = 0; i < HISTSOH_DATA_SIZE; i++) {
		chg_info("%d,", set_soh_data[i]);
		set_data[i] = (u8)(set_soh_data[i] & 0x00FF);
	}

	flag = ds28e30_cmd_write_memory(PG_USER_EEPROM_2, set_data);
	if (flag != 1) {
		chg_err("write pack historic_soh_data eeprom 2 failed\n");
		return false;
	}
	return true;
}

bool get_historic_soh_data_ds28e30(int *output_str, int len)
{
	bool flag;
	int i;
	u8 get_data[HISTSOH_PAGE_SIZE];

	len = len / sizeof(int);
	if (output_str == NULL || len > HISTSOH_PAGE_SIZE) {
		chg_err("NULL pointer detected or len %d > 32\n", len);
		return false;
	}

	/* read page data for digital signature */
	flag = ds28e30_cmd_read_memory(PG_USER_EEPROM_2, get_data);
	if (flag != true) {
		chg_err("read historic_soh_data PG_USER_EEPROM_2 failed\n");
		return false;
	}

	chg_info("read pack historic_soh_data PG_USER_EEPROM_2 is:%d", len);
	for (i = 0; i < len && i < HISTSOH_PAGE_SIZE; i++) {
		output_str[i] = (int)get_data[i];
		chg_info("%d,", output_str[i]);
	}

	return true;
}

bool get_sn_ds28e30(char *get_sn)
{
	bool flag;

	if (get_sn == NULL) {
		chg_err("get_sn == NULL failed\n");
		return false;
	}
	/* read page data for digital signature */
	flag = ds28e30_cmd_read_memory(PG_USER_EEPROM_0, pagedata[PG_USER_EEPROM_0]);
	if (flag != true) {
		chg_err("read 2 digital signature PG_USER_EEPROM_0 failed\n");
		return false;
	}

	strncpy((char *)get_sn, pagedata[PG_USER_EEPROM_0], BATT_SN_SIZE);
	get_sn[BATT_SN_SIZE] = '\0';
	chg_info("read pack sn eeprom 0 is %s\n", pagedata[PG_USER_EEPROM_0]);
	return true;
}

int get_batt_sn_ds28e30(char *batt_sn, int *sn_len)
{
	unsigned char flag;

	if (batt_sn == NULL || sn_len == NULL) {
		chg_err("batt_sn or sn_len is null");
		return false;
	}

	if (pagedata_read_status[PG_USER_EEPROM_0] == false) {
		flag = ds28e30_cmd_read_memory(PG_USER_EEPROM_0, pagedata[PG_USER_EEPROM_0]);   /* read sn/page 0 data from the given PageNumber 0 */
		if (flag != true) {
			chg_err("read sn/page 0 failed\n");
			return false;
		}
		pagedata_read_status[PG_USER_EEPROM_0] = true;
	}
	memmove(batt_sn, pagedata[PG_USER_EEPROM_0], MAXIM_BATT_SN_LEN);
	*sn_len = MAXIM_BATT_SN_LEN;
	return true;
}

/*
 Authenticate both device certificate and digital signautre
 @param[in] PageNumber
 Indicate which EEPROM page number will used, could be 0, 1, 2, 3,4,5,6,7
 @param[in] anon
 if anon='1', then device's ROMID will be displaced with all 0xFFs to generate/verify signature;
 if anon='0', unique ROMID is used
 @param[in] challenge
 reserved challenge data to generate random digital signature
  @return
  true - both certificate/digital signature is valid @n
  false or NotAuthecticated - either certificate or digital signature is invalid
*/
int authenticate_ds28e30(unsigned char sn_num[5][12], int batt_info_num, int page_number)
{
	int i;
	unsigned char flag;
	unsigned char buf[128];
	static unsigned char sig_r[32], sig_s[32];
	unsigned char challenge[32];
	unsigned char page_sn[32];
	bool sn_matched = false;
	bool parameters_matched = false;

	if (rom_no_manid_read_status == false) {
		if ((ds28e30_read_romno_manid_hardware_version()) == false) {
			chg_err("%s: read romid failed\n", __func__);
			return false;
		}
		parameters_matched = configure_ds28e30_parameters();  /* configure DS28E30 with system-level key and authority key pairs */
		if (!parameters_matched) {
			return false;
		}
		rom_no_manid_read_status = true;
	}

	if (pagedata_read_status[PG_USER_EEPROM_0] == false) {
		flag = ds28e30_cmd_read_memory(PG_USER_EEPROM_0, pagedata[PG_USER_EEPROM_0]);   /* read sn/page 0 data from the given PageNumber 0 */
		if (flag != true) {
			chg_err("%s: read sn/page 0 failed\n", __func__);
			return false;
		}
		pagedata_read_status[PG_USER_EEPROM_0] = true;
	}
	memcpy(page_sn, pagedata[PG_USER_EEPROM_0], 32);

	for (i = 2; i <= 13; i++)
		chg_info("%s: read sn[%d] %x\n", __func__, i, page_sn[i]);

	for (i = 0; i < batt_info_num; i++) {
		if (strncmp((char *)&page_sn[2], &sn_num[i][0], 12)) {
			continue;
		} else {
			chg_info("%s: sn compare succ. batt_info :%d\n", __func__, i);
			sn_matched = true;
			break;
		}
	}
	if (sn_matched == false) {
		chg_err("%s: sn compare failed\n", __func__);
		return false;
	}

	/* read the device public key X&Y */
	if (publickey_read_status == false) {
		flag = ds28e30_cmd_read_device_public_key(buf);
		if (flag != true) {
			chg_err("%s: read device publickey failed\n", __func__);
			return false;
		} else {
			memcpy(device_publickey_x, buf, 32);  /* reserve device public key X */
			memcpy(device_publickey_y, &buf[32], 32);  /* reserve device public key Y */
			publickey_read_status = true;
		}
	}
	/* read device certificate */
	if (certificate_read_status == false) {
		for (i = 0; i < 2; i++) {
			flag = ds28e30_cmd_read_memory(PG_CERTIFICATE_R+i, buf+i*32);   /* read device Certificate R&S in buf[] */
			if (flag != true) {
				chg_err("%s: %d read device certificate failed\n", __func__, i);
				return false;
			}
		}
		memcpy(page_certificate_r, buf, 32);  /* reserve device certificate R */
		memcpy(page_certificate_s, &buf[32], 32);  /* reserve device certificate S */
		certificate_read_status = true;
	}
	/* authenticate DS28E30 by two steps: digital signature verification, device certificate verification */
	/* to verify the digital signature */
	/* prepare to verify the signature */
	memcpy(public_key_x, device_publickey_x, 32);  /* copy device public key X to public key x buffer */
	memcpy(public_key_y, device_publickey_y, 32);  /* copy device public key Y to public key x buffer */
	/* read page data for digital signature */
	page_number = page_number & 0x03;    /* page number must less than 4 */
	if (pagedata_read_status[page_number] == false) {
		flag = ds28e30_cmd_read_memory(page_number, pagedata[page_number]);   /* read page data from the given PageNumber */
		if (flag != true) {
			chg_err("%s: read page %d failed\n", __func__, page_number);
			return false;
		}
		pagedata_read_status[page_number] = true;
	}
	/* generate random challenge */
	memcpy(buf, sig_r, 32);
	memcpy(&buf[32], sig_s, 32);
	ucl_sha256(challenge, buf, 64);

	/* setup software ECDSA computation */
	deep_cover_coproc_setup(0, 0, 0, 0);
	flag = ds28e30_compute_verify_ecdsa_no_read(page_number, 0, pagedata[page_number], challenge, sig_r, sig_s);
	if (flag == false) {
		chg_err("%s: digital signature verify failed\n", __func__);
		return false;  /* digital signature is not verified */
	}
	chg_err("%s: digital signature verify succ\n", __func__);
	/* verify device certificate */
	flag = verify_ecdsa_certificate_device(page_certificate_r, page_certificate_s, device_publickey_x,
		device_publickey_y, rom_no, man_id, system_public_key_x, system_public_key_y);
	if (flag == false) {
		chg_err("%s: verify device certificate failed\n", __func__);
		return false;  /* device certificate is not verified */
	}
	chg_info("%s: Authenticate succ\n", __func__);
	return true;  /* both digital signature & device certificate are verified */
}

/*
 High level function to do a authenticated write memory  page with ECW mode
 @param[in] pg
 page to do operation on
 @param[in] mempage
 buffer with new memory page contents to be written
 @return
 true - command successful @n
 false - command failed
*/

int ds28e30_write_memory_page_with_ecw(int pg, u8 *new_data)
{
	/* static unsigned char AlreadyAccessDS28E30=false; */
	u8 message[256], old_data[32], sig_r[32], sig_s[32];
	int msg_len;

	/* assume the device ROMID and MANID has been got from DS28E30 */
	/* read old page data */
	if ((ds28e30_cmd_read_memory(pg, old_data)) == false)
		return false;
	/* construct the message to compute authentication Signature */
	/* (ROM NO | Old Page Data | New Page Data | 0x80 |Page # | MANID) */

	memcpy(message, rom_no, 8);
	msg_len = 8;
	/* old data */
	memcpy(&message[msg_len], old_data, 32);
	msg_len += 32;
	/* new data */
	memcpy(&message[msg_len], new_data, 32);
	msg_len += 32;
	/* Page# */
	message[msg_len++] = (0x80 | pg);
	/* MANID */
	memcpy(&message[msg_len], man_id, 2);
	msg_len += 2;
	deep_cover_coproc_setup(0, 0, 0, 0);
	/* deep_cover_computeECDSASignature(message, msg_len, authority_private_key, sig_r, sig_s); TODO: */
	return ds28e30_cmd_authendicated_ecdsa_write_memory(pg, new_data, sig_r, sig_s);
}

/*  ds28e30 Helper functions
 Return last result byte. Useful if a function fails.
 @return
 Result byte
 */
u8 ds28e30_get_last_result_byte(void)
{
	return last_result_byte;
}

/* try to correct 1-wire write data packet if wdcrc16 is error in function command Compute and Read Page Authentication(0xA5)
if return true, the write data packet is corrected;if false, the write data packet is still wrong in wdcrc16 */
int write_data_correction(u8 *pkt, u8 pkt_len, u8 *write_buf, u8 write_len)
{
	unsigned char temp[8] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};
	int i, j, count;
	ushort temp_crc;

	/* check wdcrc16 */
	wdcrc16 = 0;
	for (i = 0; i < pkt_len; i++)
		docrc16(pkt[i]);
	if (wdcrc16 == 0xB001)
		return true;
	if (write_buf[0] != CMD_COMP_READ_AUTH) return false;  /* no correction action for other function commands */

	/* try to correct only one bit */
	for (i = 0; i < CRC_CORRECT_BIT_NUMBER; i++) {
		if (wbit_lowtime_buffer[i][1] < 40)
		  continue;
		memcpy(pkt+2, write_buf, write_len);
		pkt[wbit_lowtime_buffer[i][1] / 8] ^= temp[wbit_lowtime_buffer[i][1] % 8];
		wdcrc16 = 0;  /* check wdcrc16 */
		for (j = 0; j < pkt_len-2; j++)
		  docrc16(pkt[j]);
		wdcrc16 ^= 0xFFFF;
		temp_crc = pkt[pkt_len-1] << 8;
		temp_crc += pkt[pkt_len-2];
		temp_crc ^= wdcrc16;
		wdcrc16 = temp_crc;
		count = 0;
		for (j = 0; j < 16; j++) {
		  if ((temp_crc & 0x01) == 0) count++;
		  temp_crc >>= 1;
		}
		if (count >= 14) {
		  pkt[pkt_len-2] ^= (0xFF & wdcrc16);
		  pkt[pkt_len-1] ^= (0xFF & (wdcrc16 >> 8));
		  write_buf[(wbit_lowtime_buffer[i][1] / 8)-2] = pkt[wbit_lowtime_buffer[i][1] / 8];
		  return 2;
		}
	}

	/* try to correct two bits */
	for (i = 0; i < CRC_CORRECT_BIT_NUMBER; i++) {
		if ((wbit_lowtime_buffer[(i + 1) % CRC_CORRECT_BIT_NUMBER][1]) < 40)
		  continue;
		if ((wbit_lowtime_buffer[(i + 2) % CRC_CORRECT_BIT_NUMBER][1]) < 40)
		  continue;
		memcpy(pkt+2, write_buf, write_len);
		pkt[wbit_lowtime_buffer[0][1] / 8] ^= temp[wbit_lowtime_buffer[i][1] % 8];
		pkt[wbit_lowtime_buffer[1][1] / 8] ^= temp[wbit_lowtime_buffer[i][1] % 8];
		pkt[wbit_lowtime_buffer[2][1] / 8] ^= temp[wbit_lowtime_buffer[i][1] % 8];
		pkt[wbit_lowtime_buffer[i][1] / 8] ^= temp[wbit_lowtime_buffer[i][1] % 8];
		wdcrc16 = 0;  /* check wdcrc16 */
		for (j = 0; j < pkt_len-2; j++)
		  docrc16(pkt[j]);
		wdcrc16 ^= 0xFFFF;
		temp_crc = pkt[pkt_len-1] << 8;
		temp_crc += pkt[pkt_len-2];
		temp_crc ^= wdcrc16;
		wdcrc16 = temp_crc;
		count = 0;
		for (j = 0; j < 16; j++) {
		  if ((temp_crc & 0x01) == 0) count++;
		  temp_crc >>= 1;
		}
		if (count >= 14) {
		  pkt[pkt_len-2] ^= (0xFF & wdcrc16);
		  pkt[pkt_len-1] ^= (0xFF & (wdcrc16 >> 8));
		  memcpy(write_buf, pkt+2, write_len);
		  return 3;
		}
	}

	/* try to correct 3 bits */
	if (wbit_lowtime_buffer[0][1] < 40) return false;
	if (wbit_lowtime_buffer[1][1] < 40) return false;
	if (wbit_lowtime_buffer[2][1] < 40) return false;
	memcpy(pkt+2, write_buf, write_len);
	pkt[wbit_lowtime_buffer[0][1] / 8] ^= temp[ wbit_lowtime_buffer[0][1] % 8];
	pkt[wbit_lowtime_buffer[1][1] / 8] ^= temp[ wbit_lowtime_buffer[1][1] % 8];
	pkt[wbit_lowtime_buffer[2][1] / 8] ^= temp[ wbit_lowtime_buffer[2][1] % 8];
	wdcrc16 = 0;  /* check wdcrc16 */
	for (j = 0; j < pkt_len-2; j++)
	  docrc16(pkt[j]);
	wdcrc16 ^= 0xFFFF;
	temp_crc = pkt[pkt_len-1] << 8;
	temp_crc += pkt[pkt_len-2];
	temp_crc ^= wdcrc16;
	wdcrc16 = temp_crc;
	count = 0;
	for (j = 0; j < 16; j++) {
	  if ((temp_crc & 0x01) == 0) count++;
	  temp_crc >>=1;
	}
	if (count >= 14) {
	  pkt[pkt_len-2] ^= (0xFF & wdcrc16);
	  pkt[pkt_len-1] ^= (0xFF & (wdcrc16 >> 8));
	  memcpy(write_buf, pkt+2, write_len);
	  return 4;
	}
	return false;  /* not successfully correct write data packet */
}

/* try to correct 1-wire read data packet if crc16 is error in function command Compute and Read Page Authentication(0xA5)
if return true, the read data packet is corrected;if false, the read data packet is still wrong in crc16 */
static int read_data_correction(u8 *read_buf, int read_len, u8 *write_buf, int write_len)
{
	u8 pkt[256];
	int i, j;
	unsigned char temp[8] = {PROT_RP, PROT_WP, PROT_EM, PROT_DC, PROT_PRI, PROT_AUTH, PROT_ECH, PROT_ECW};

	/* check crc16 */
	memcpy(pkt, read_buf, read_len + EXPECTED_READ_LENGTH_2);
	crc16 = ZERO_VALUE;
	docrc16(read_len);
	for (j = 0; j < read_len + EXPECTED_READ_LENGTH_2; j++)
		docrc16(pkt[j]);
	if (crc16 == SKIP_CRC_CHECK)
		return DATA_PACKET_BITS_CORRECT;
	if (write_buf[0] != CMD_COMP_READ_AUTH)
		return DATA_PACKET_NO_ACTION;

	/* try to correct only one bit */
	for (i = 0; i < CRC_CORRECT_BIT_NUMBER; i++) {
		memcpy(pkt, read_buf, read_len + EXPECTED_READ_LENGTH_2);
		pkt[rbit_lowtime_buffer[i][1] / 8] ^= temp[rbit_lowtime_buffer[i][1] % 8];

		crc16 = ZERO_VALUE;  /* check crc16 */
		docrc16(read_len);
		for (j = 0; j < read_len + EXPECTED_READ_LENGTH_2; j++)
			docrc16(pkt[j]);
		if (crc16 != SKIP_CRC_CHECK)
			continue;
		read_buf[rbit_lowtime_buffer[i][1] / 8] ^= temp[rbit_lowtime_buffer[i][1] % 8];
		return DATA_PACKET_2_BITS_ERROR;
	}

	/* try to correct 2 bits */
	for (i = 0; i < CRC_CORRECT_BIT_NUMBER; i++) {
		memcpy(pkt, read_buf, read_len + EXPECTED_READ_LENGTH_2);
		pkt[rbit_lowtime_buffer[0][1] / 8] ^= temp[rbit_lowtime_buffer[0][1] % 8];
		pkt[rbit_lowtime_buffer[1][1] / 8] ^= temp[rbit_lowtime_buffer[1][1] % 8];
		pkt[rbit_lowtime_buffer[2][1] / 8] ^= temp[rbit_lowtime_buffer[2][1] % 8];
		pkt[rbit_lowtime_buffer[i][1] / 8] ^= temp[rbit_lowtime_buffer[i][1] % 8];

		crc16 = ZERO_VALUE;  /* check crc16 */
		docrc16(read_len);
		for (j = 0; j < read_len + EXPECTED_READ_LENGTH_2; j++)
			docrc16(pkt[j]);
		if (crc16 != SKIP_CRC_CHECK)
			continue;

		read_buf[rbit_lowtime_buffer[0][1] / 8] ^= temp[rbit_lowtime_buffer[0][1] % 8];
		read_buf[rbit_lowtime_buffer[1][1] / 8] ^= temp[rbit_lowtime_buffer[1][1] % 8];
		read_buf[rbit_lowtime_buffer[2][1] / 8] ^= temp[rbit_lowtime_buffer[2][1] % 8];
		read_buf[rbit_lowtime_buffer[i][1] / 8] ^= temp[rbit_lowtime_buffer[i][1] % 8];

		return DATA_PACKET_3_BITS_ERROR;
	}

	/* try to correct 3 bits */
	memcpy(pkt, read_buf, read_len + EXPECTED_READ_LENGTH_2);
	pkt[rbit_lowtime_buffer[0][1] / 8] ^= temp[rbit_lowtime_buffer[0][1] % 8];
	pkt[rbit_lowtime_buffer[1][1] / 8] ^= temp[rbit_lowtime_buffer[1][1] % 8];
	pkt[rbit_lowtime_buffer[2][1] / 8] ^= temp[rbit_lowtime_buffer[2][1] % 8];

	crc16 = ZERO_VALUE;  /* check crc16 */
	docrc16(read_len);
	for (j = 0; j < read_len + EXPECTED_READ_LENGTH_2; j++)
		docrc16(pkt[j]);
	if (crc16 != SKIP_CRC_CHECK)
		return DATA_PACKET_NO_ACTION;

	read_buf[rbit_lowtime_buffer[0][1] / 8] ^= temp[rbit_lowtime_buffer[0][1] % 8];
	read_buf[rbit_lowtime_buffer[1][1] / 8] ^= temp[rbit_lowtime_buffer[1][1] % 8];
	read_buf[rbit_lowtime_buffer[2][1] / 8] ^= temp[rbit_lowtime_buffer[2][1] % 8];

	return DATA_PACKET_4_BITS_ERROR;
}

/* try to correct 1-wire read ROMID if CRC-8 is error in reading ROMID function
if return not false, the read ROMID packet is corrected;if false, the read data packet is still wrong in crc16 */
static int read_romid_correction(u8 *read_buf)
{
	u8 pkt[256];
	int i;

	/* verify CRC8 */
	memcpy(pkt, read_buf, 8);
	crc8 = ZERO_VALUE;
	for (i = 0; i < 8; i++)
		docrc8(pkt[i]);
	if (crc8 == ZERO_VALUE)
		return DATA_PACKET_BITS_CORRECT;

	if (rom_no[0] == ZERO_VALUE) {
		if (read_buf[0] == DS28E30_FAM && read_buf[7] == 0x8A) {   /* the defaule ROMID's CRC8=0x8A  */
			memset(&read_buf[1], ZERO_VALUE, 6);   /* set to the default 48-bit serial number (all 0x00)  */
			return DATA_PACKET_BITS_CORRECT;
		}
	}
	return DATA_PACKET_NO_ACTION;
}

void check_romid_bit(long r_diff_ns, unsigned int vamm, unsigned char cnt)
{
	if (vamm == 1) {
		if (r_diff_ns > rbit_lowtime_buffer[0][0]) {
			for (cnt = CRC_CORRECT_BIT_NUMBER - 1; cnt > 0; cnt--) {
				rbit_lowtime_buffer[cnt][0] = rbit_lowtime_buffer[cnt - 1][0];
				rbit_lowtime_buffer[cnt][1] = rbit_lowtime_buffer[cnt - 1][1];
			}
			rbit_lowtime_buffer[0][0] = r_diff_ns;
			rbit_lowtime_buffer[0][1] = rbit_counter;
		} else if (r_diff_ns > rbit_lowtime_buffer[1][0] && r_diff_ns <= rbit_lowtime_buffer[0][0]) {
			for (cnt = CRC_CORRECT_BIT_NUMBER - 1; cnt > 1; cnt--) {
				rbit_lowtime_buffer[cnt][0] = rbit_lowtime_buffer[cnt - 1][0];
				rbit_lowtime_buffer[cnt][1] = rbit_lowtime_buffer[cnt - 1][1];
			}
			rbit_lowtime_buffer[1][0] = r_diff_ns;
			rbit_lowtime_buffer[1][1] = rbit_counter;
		} else if (r_diff_ns > rbit_lowtime_buffer[2][0]) {
			rbit_lowtime_buffer[2][0] = r_diff_ns;
			rbit_lowtime_buffer[2][1] = rbit_counter;
		}
	}
	rbit_counter++;
}

void check_womid_bit(long w_diff_ns)
{
	if (w_diff_ns > wbit_lowtime_buffer[0][0]) {
		memmove(wbit_lowtime_buffer[1], wbit_lowtime_buffer[0], sizeof(short)*(CRC_CORRECT_BIT_NUMBER-1));
		wbit_lowtime_buffer[0][0] = w_diff_ns;
		wbit_lowtime_buffer[0][1] = wbit_counter;
	}
	if (w_diff_ns > wbit_lowtime_buffer[1][0] && w_diff_ns <= wbit_lowtime_buffer[0][0]) {
		memmove(wbit_lowtime_buffer[2], wbit_lowtime_buffer[1], sizeof(short)*(CRC_CORRECT_BIT_NUMBER-2));
		wbit_lowtime_buffer[1][0] = w_diff_ns;
		wbit_lowtime_buffer[1][1] = wbit_counter;
	}
	if (w_diff_ns > wbit_lowtime_buffer[2][0]) {
		wbit_lowtime_buffer[2][0] = w_diff_ns;
		wbit_lowtime_buffer[2][1] = wbit_counter;
	}
}

static void correct_first_received_crc16(unsigned char *pkt, unsigned char pkt_len)
{
	unsigned char expected_crc16[2];
	int i;
	unsigned char temp = 0x01;
	if (pkt[pkt_len - 2] == 0xFF  && pkt[pkt_len - 1] == 0xFF)
		return;
	/* if received crc16=0xFFFF, then no operation calculate the expected crc16 for the sending data packet */
	crc16 = 0;
	for (i = 0; i < pkt_len - 2; i++)
		docrc16(pkt[i]);
	crc16 = crc16 ^ 0xFFFF;  /* inverted crc16 code */
	expected_crc16[0] = crc16 & 0xFF;
	expected_crc16[1] = (crc16 >> 8) & 0xFF;
	if (expected_crc16[0] == pkt[pkt_len - 2] && expected_crc16[1] == pkt[pkt_len - 1])
		return;
	for (i = 0; i < 8; i++) {
		if ((expected_crc16[0] & temp) != 0 && (pkt[pkt_len - 2] & temp) ==0) return;
		if ((expected_crc16[1] & temp) != 0 && (pkt[pkt_len - 1] & temp) ==0) return;
		temp = temp << 1;
	}
	pkt[pkt_len - 2] &= expected_crc16[0];
	pkt[pkt_len - 1] &= expected_crc16[1];
}
/*  @internal
 Sent/receive standard flow command
 @param[in] write_buf
 Buffer with write contents (preable payload)
 @param[in] write_len
 Total length of data to write in 'write_buf'
 @param[in] delay_ms
 Delay in milliseconds after command/preable.  If == 0 then can use
 repeated-start to re-access the device for read of result byte.
 @param[in] expect_read_len
 Expected result read length
 @param[out] read_buf
 Buffer to hold data read from device. It must be at least 255 bytes long.
 @param[out] read_len
 Pointer to an integer to contain the length of data read and placed in read_buf
 Preloaded with expected read length for 1-Wire mode. If (0) but expected_read=true
 then the first byte read is the length of data to read.
  @return
  true - command successful @n
  false - command failed
 @endinternal
 */

int standard_cmd_flow(u8 *write_buf, int write_len, int delayms, int expect_read_len, u8 *read_buf, int *read_len)
{
	u8 pkt[256];
	int pkt_len = 0;
	int i;
	/* int start_offset = 0; */

#ifdef SPIN_LOCK_ENABLE
	mutex_lock(&ds_cmd_lock);
#endif
	/* Reset/presence */
	/* Rom COMMAND (set from select options) */
	if (!ow_skip_rom()) {
#ifdef SPIN_LOCK_ENABLE
		mutex_unlock(&ds_cmd_lock);
#endif
		return false;
	}
	/* set result byte to no response */
	last_result_byte = RESULT_FAIL_COMMUNICATION;

	/* Construct write block, start with XPC command */
	pkt[pkt_len++] = XPC_COMMAND;

	/* Add length */
	pkt[pkt_len++] = write_len;

	/* write (first byte will be sub-command) */
	memcpy(&pkt[pkt_len], write_buf, write_len);
	pkt_len += write_len;

	/* initial 1-wire read/write data correction global variables */
	wbit_counter = ZERO_VALUE;
	rbit_counter = ZERO_VALUE;
	memset((u8 *)rbit_lowtime_buffer, RESULT_FAIL_VERIFY, sizeof(short) * CRC_CORRECT_BIT_NUMBER * 2);
	memset((u8 *)wbit_lowtime_buffer, RESULT_FAIL_VERIFY, sizeof(short) * CRC_CORRECT_BIT_NUMBER * 2);

	/* send packet to DS28E30 */
	for(i = 0; i < pkt_len; i++)
		write_byte(pkt[i]);

	/* read two CRC bytes */
	pkt[pkt_len++] = read_byte();
	pkt[pkt_len++] = read_byte();
	if (get_maxim_romid_crc_support())
		correct_first_received_crc16(pkt, pkt_len);
	i = write_data_correction(pkt, pkt_len, write_buf, write_len);  /* try to correct 1-wire write data if wdcrc16 is error */
	if (i >= 2)
	  chg_info("%s: correct %d-bit error at write data packet!", __func__, i-1);

	/* check crc16 */
	crc16 = 0;
	for (i = 0; i < pkt_len; i++)
		docrc16(pkt[i]);
	/* check if ROMID is populated after power up, skip CRC check if rom_no[0] = =0x00. */
	if (rom_no[0] != 0) {
		if (crc16 != SKIP_CRC_CHECK) {
#ifdef SPIN_LOCK_ENABLE
			mutex_unlock(&ds_cmd_lock);
#endif
			chg_info("%s: Tx CRC failed\n", __func__);
			return false;
		}
	}

	/* Send release byte, start strong pull-up */
	write_byte(STRONG_PULL_UP);

	/* optional delay */
	if (delayms)
		delay_ms(delayms);

	/* turn off strong pull-up */
	/*   OWLevel(MODE_NORMAL); */
	/* read FF and the length byte */
	pkt[0] = read_byte();
	pkt[1] = read_byte();
	if (get_maxim_romid_crc_support()) {
		if (write_buf[0] == CMD_COMP_READ_AUTH)
			pkt[1] = EXPECTED_READ_LENGTH_65;
		if (write_buf[0] == CMD_READ_MEM)
			pkt[1] = EXPECTED_READ_LENGTH_33;
		if (write_buf[0] == CMD_READ_STATUS) {
			if ((STATUS_DATA_LENGTH_VERIFY & write_buf[1]) == ZERO_VALUE)
				pkt[1] = EXPECTED_READ_LENGTH_2;
			else
				pkt[1] = EXPECTED_READ_LENGTH_5;
		}
		if (write_buf[0] == CMD_WRITE_MEM)
			pkt[1] = EXPECTED_READ_LENGTH_1;
		if (write_buf[0] == CMD_SET_PAGE_PROT)
			pkt[1] = EXPECTED_READ_LENGTH_1;
		if (write_buf[0] == CMD_DECREMENT_CNT)
			pkt[1] = EXPECTED_READ_LENGTH_1;
	}
	*read_len = pkt[1];

	/* make sure there is a valid length */
	if (*read_len < RESULT_FAIL_LENGTH && *read_len > ZERO_VALUE) {
		if (get_maxim_romid_crc_support()) {
			rbit_counter = ZERO_VALUE;  /* initialize 1-wire read data correction global variables */
			memset((u8 *)rbit_lowtime_buffer, RESULT_FAIL_VERIFY, sizeof(short) * CRC_CORRECT_BIT_NUMBER * 2);
		}
		/* read packet */
		for (i = 0; i <  *read_len+2; i++)
			read_buf[i] = read_byte();
		if (get_maxim_romid_crc_support()) {
			i = read_data_correction(read_buf, *read_len, write_buf, write_len);  /* try to correct 1-wire read data if crc16 is error */
			if (i >= 2)
				chg_info("%s: correct %d-bit error at read data packet!", __func__, i-1);
		}
		/* check crc16 */
		crc16 = 0;
		docrc16(*read_len);
		for (i = 0; i < (*read_len + 2); i++)
			docrc16(read_buf[i]);

		if (crc16 != SKIP_CRC_CHECK) {
#ifdef SPIN_LOCK_ENABLE
			mutex_unlock(&ds_cmd_lock);
#endif
			chg_info("%s: Rx CRC failed\n", __func__);
			return false;
		}

		if (expect_read_len != *read_len) {
#ifdef SPIN_LOCK_ENABLE
			mutex_unlock(&ds_cmd_lock);
#endif
			return false;
		}
	} else {
#ifdef SPIN_LOCK_ENABLE
		mutex_unlock(&ds_cmd_lock);
#endif
		return false;
	}

	/* success */
#ifdef SPIN_LOCK_ENABLE
	mutex_unlock(&ds_cmd_lock);
#endif
	return true;
}

/* Calculate a new crc16 from the input data shorteger.  Return the current */
/* crc16 and also update the global variable crc16 */
static short oddparity[16] = {0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0};

unsigned short docrc16(unsigned short data)
{
	data = (data ^ (crc16 & 0xff)) & 0xff;
	crc16 >>= 8;

	if (oddparity[data & 0xf] ^ oddparity[data >> 4])
		crc16 ^= 0xc001;

	data <<= 6;
	crc16 ^= data;
	data <<= 1;
	crc16 ^= data;

	return crc16;
}

/*  read device ROM NO, MANID and hardware version
 @return
 true read ROM ID, man_id and hardware version and confirmed @n
 false failure to read ds28e30
 */

int ds28e30_read_romno_manid_hardware_version(void)
{
	u8 i, temp = 0, buf[10], pg = 0, flag;

	chg_info("%s entry", __func__);
	rom_no[0] = 0x00;

	flag = ow_read_rom();  /* search DS28E30 */
	if (flag == false)
		return false;   /*error in reading ROMID */
	if ((rom_no[0] & 0x7F) == DS28E30_FAM) {
		for (i = 0; i < 6; i++)
			temp |= rom_no[1 + i];  /* check if the device is power up at the first time */
		if (temp == 0) {  /* power up the device, then read ROMID again */
			chg_info("%s temp==0", __func__);
			rom_no[0] = 0x00;
			ds28e30_cmd_read_status(pg, buf, man_id, hardware_version);  /* page number=0 */
			flag = ow_read_rom();      /* read ROMID from DS28E30 */
			if (flag == false)
				return false;   /*error in reading ROMID */
			temp = 0;
			for (i = 0; i < 6; i++)
				temp |= rom_no[1 + i];  /* check if the device is power up at the first time */
			if (temp == 0)
				return false;  /* power up the device, then read ROMID again */
			flag = ds28e30_cmd_read_status(0x80 | pg, buf, man_id, hardware_version);  /* page number=0 */
			chg_info("%s temp==0 flag %d", __func__, flag);
			return flag;
		} else {
			flag = ds28e30_cmd_read_status(0x80 | pg, buf, man_id, hardware_version);  /* page number=0 */
			chg_info("%s temp!=0 flag %d", __func__, flag);
			return flag;
		}
	}
	return false;
}

/*  Set public key in module.  This will be used in other helper functions.
 @param[in] pubkey_x
 buffer for x portion of public key, must be 32 bytes
 @param[in] pubkey_y
 buffer for y portion of public key, must be 32 bytes
 */
void ds28e30_set_public_key(u8 *pubkey_x, u8 *pubkey_y)
{
	memcpy(public_key_x, pubkey_x, 32);
	memcpy(public_key_y, pubkey_y, 32);
}

/*
 Set public key in module.  This will be used in other helper functions.
 @param[in] priv
 buffer for private key, must be 32 bytes
 */
void ds28e30_set_private_key(u8 *priv)
{
	memcpy(private_key, priv, 32);
}

/*******************************************************************
@brief Helper function to compute Signature using the specified private key.
@param[in] message
Messge to hash for signature verification
@param[in] msg_len
Length of message in bytes
@param[in] key
(0, 1) to indicate private key A,B
@param[out] sig_r
signature portion r
@param[out] sig_s
signature portion s
@return
true - signature verified @n
false - signature not verified
*******************************************************************/
int sw_compute_ecdsa_signature(u8 *message, int msg_len, u8 *sig_r, u8 *sig_s)
{
	int configuration;
	struct ucl_type_ecdsa_signature signature;

	/* hook up r/s to the signature structure */
	signature.r = sig_r;
	signature.s = sig_s;

	/* construct configuration */
	configuration = (SECP256R1 << UCL_CURVE_SHIFT) ^ (UCL_MSG_INPUT << UCL_INPUT_SHIFT)
					 ^ (UCL_SHA256 << UCL_HASH_SHIFT);

	/* create signature and return result */
	return (ucl_ecdsa_signature(&signature, private_key, ucl_sha256, message, msg_len, &secp256r1, configuration) == 0);
}
/*
The 'OWReadROM' function does a Read-ROM.  This function
uses the read-ROM function 33h to read a ROM number and verify CRC8.
Returns:   true (1) : OWReset successful and Serial Number placed
                      in the global ROM, CRC8 valid
           false (0): OWReset did not have presence or CRC8 invalid
 */
int ow_read_rom(void)
{
	u8 buf[16];
	int i;
#ifdef SPIN_LOCK_ENABLE
	mutex_lock(&ds_cmd_lock);
#endif

	if (ow_reset() == 1) {
		write_byte(0x33); /* READ ROM command */
		if (get_maxim_romid_crc_support()) {
			rbit_counter = ZERO_VALUE;  /* initialize 1-wire read data correction global variables */
			memset((u8 *)rbit_lowtime_buffer, RESULT_FAIL_VERIFY, sizeof(short) * CRC_CORRECT_BIT_NUMBER * 2);
		}
		for (i = 0; i < 8; i++)
			buf[i] = read_byte();
		if (get_maxim_romid_crc_support()) {
			i = read_romid_correction(buf);  /* try to correct 1-wire read ROMID if CRC-8 is error */
			chg_info("%s: correct error at OWReadROM() function i = %d \n", __func__, i);
		}
		chg_info("RomID = %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x\n",
			buf[0], buf[1], buf[2], buf[3],
			buf[4], buf[5], buf[6], buf[7]);
		/* verify CRC8 */
		crc8 = 0;
		for (i = 0; i < 8; i++)
			docrc8(buf[i]);
		if ((crc8 == 0) && (buf[0] != 0)) {
			memcpy(rom_no, &buf[0], 8);
#ifdef SPIN_LOCK_ENABLE
			mutex_unlock(&ds_cmd_lock);
#endif
			chg_info("DS28E30_standard_cmd_flow: read ROMID successfully!\n");
			return true;
		}
	}
#ifdef SPIN_LOCK_ENABLE
	mutex_unlock(&ds_cmd_lock);
#endif
	chg_err("DS28E30_standard_cmd_flow: error in reading ROMID!\n");
	return false;
}


/*
The 'OWSkipROM' function does a skip-ROM.  This function
uses the Skip-ROM function CCh.
Returns:   true (1) : OWReset successful and skip rom sent.
           false (0): OWReset did not have presence
*/

int ow_skip_rom(void)
{
	if (ow_reset() == 1) {
		write_byte(OW_SKIP_ROM);
		return true;
	}

	return false;
}

static unsigned char dscrc_table[] = {
	0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
	157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
	35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
	190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
	70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
	219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
	101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
	248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
	140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
	17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
	175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
	50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
	202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
	87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
	233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
	116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53};

/* --------------------------------------------------------------------------
Calculate the CRC8 of the byte value provided with the current
global 'crc8' value.
Returns current global crc8 value
*/
unsigned char docrc8(unsigned char value)
{
	/* See Application Note 27 */
	/* TEST BUILD */
	crc8 = dscrc_table[crc8 ^ value];
	return crc8;
}

MODULE_DESCRIPTION("oplus ds28e30 driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:oplus-ds28e30");
