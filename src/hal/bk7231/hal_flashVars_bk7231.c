/*
	This module saves variable data to a flash region in an erase effient way.

	Design:
	variables to be small - we want as many writes between erases as possible.
	sector will be all FF (erased), and data added into it.
	reading consists of searching the first non FF byte from the end, then
	reading the preceding bytes as the variables.
	last byte of data is len (!== 0xFF!)

*/

#include "include.h"
#include "mem_pub.h"
#include "drv_model_pub.h"
#include "net_param_pub.h"
#include "flash_pub.h"
#include "../hal_flashVars.h"

#include "BkDriverFlash.h"
#include "BkDriverUart.h"

#include "../../logging/logging.h"

#define LOG_FEATURE LOG_FEATURE_CFG
#define FLASH_VARS_MAGIC_V0 0xfefefefe

extern FLASH_VARS_STRUCTURE flash_vars;

uint32_t flash_vars_start;
uint32_t flash_vars_end;
const uint32_t flash_vars_part_len = 0x2000; // two blocks in BK7231
const uint32_t flash_vars_sector_len = 0x1000; // erase size in BK7231

bool g_flashVars_safeToWrite;

FLASH_VARS_STRUCTURE flash_vars = {
	.footer = FLASH_VARS_FOOTER
};

FLASH_VARS_HEADER flash_header = {
	.magic = FLASH_VARS_MAGIC,
	.version = FLASH_VARS_VERS,
	.vars_len = sizeof(FLASH_VARS_STRUCTURE),
	.number_ch = MAX_RETAIN_CHANNELS,
#ifdef ENABLE_POWERMETERING
	.emetering_offset = offsetof(FLASH_VARS_STRUCTURE, emetering),
#else
	.emetering_offset = 0,
#endif // ENABLE_POWERMETERING
#ifdef ENABLE_DRIVER_LED
	.led_offset = offsetof(FLASH_VARS_STRUCTURE, led),
#endif // ENABLE_DRIVER_LED
	.led_offset = 0,
	.footer = FLASH_VARS_FOOTER
};

// flash vars offset starts after part header
static uint32_t flash_vars_next;

// to upgrade existing flash vars partitions
bool flash_vars_read_v0() {

	#define MAX_RETAIN_CHANNELS_V0 12
	typedef struct flash_vars_structure_v0 {
		uint16_t boot_count;
		uint16_t boot_success_count;
		int16_t savedValues[MAX_RETAIN_CHANNELS_V0];
		uint32_t emetering[8];
		uint8_t rgb[3];
		uint8_t len; // length of the whole structure (i.e. 2+2+1 = 5)  MUST NOT BE 255
	} FLASH_VARS_STRUCTURE_V0;

	FLASH_VARS_STRUCTURE_V0 data = {0};

	UINT32 status;
	DD_HANDLE flash_hdl = ddev_open(FLASH_DEV_NAME, &status, 0);
	ASSERT(DD_HANDLE_UNVALID != flash_hdl);

	uint32_t start_addr = flash_vars_start + flash_vars_part_len;
	unsigned int tmp = 0xffffffff;
	GLOBAL_INT_DECLARATION();

	bk_flash_enable_security(FLASH_PROTECT_NONE);

	do {
		start_addr -= sizeof(tmp);
		GLOBAL_INT_DISABLE();
		ddev_read(flash_hdl, (char*)&tmp, sizeof(tmp), start_addr);
		GLOBAL_INT_RESTORE();
	} while ((tmp == 0xFFFFFFFF) && (start_addr > flash_vars_start + sizeof(tmp)));
	if (tmp == 0xffffffff) {
		ddev_close(flash_hdl);
		bk_flash_enable_security(FLASH_PROTECT_ALL);
		ADDLOGF_ERROR("%s - not found", __func__);
		return false;
	}

	start_addr += sizeof(tmp);

	int len = 0;
	while ((tmp & 0xFF000000) == 0xFF000000) {
		tmp <<= 8;
		start_addr--;
	}
	len = (tmp >> 24) & 0xff;
	start_addr -= len;

	if (len != sizeof(data)) {
		ADDLOGF_ERROR("%s - unexpected length (%d)", len);
		ddev_close(flash_hdl);
		bk_flash_enable_security(FLASH_PROTECT_ALL);
		return false;
	}
	// read the DATA portion into the structure
	GLOBAL_INT_DISABLE();
	ddev_read(flash_hdl, (char*)&data, len, start_addr);
	GLOBAL_INT_RESTORE();
	ddev_close(flash_hdl);
	bk_flash_enable_security(FLASH_PROTECT_ALL);
	
	flash_vars.boot_count = data.boot_count;
	flash_vars.boot_success_count = data.boot_success_count;
	// truncate if smaller
	memcpy(&flash_vars.savedValues, data.savedValues, sizeof(flash_vars.savedValues));
#ifdef ENABLE_POWERMETERING
	memcpy(&flash_vars.emetering, data.emetering, sizeof(flash_vars.emetering));
	memcpy(&flash_vars.emetering, data.emetering, sizeof(flash_vars.emetering));
#endif // ENABLE_POWERMETERING
#ifdef ENABLE_DRIVER_LED
	flash_vars.led.bEnableAll = data.savedValues[MAX_RETAIN_CHANNELS_V0 - 4];
	flash_vars.led.mode = data.savedValues[MAX_RETAIN_CHANNELS_V0 - 3];
	flash_vars.led.temperature = data.savedValues[MAX_RETAIN_CHANNELS_V0 - 2];
	flash_vars.led.brightness = data.savedValues[MAX_RETAIN_CHANNELS_V0 - 1];
	memcpy(&flash_vars.led.rgb, data.rgb, sizeof(flash_vars.led.rgb));
#endif // ENABLE_DRIVER_LED

	return true;
}

// called in flash vars read, returns len of stored vars data
static bool flash_vars_part_valid() {
	UINT32 status;
	DD_HANDLE flash_hdl;
	GLOBAL_INT_DECLARATION();

	flash_hdl = ddev_open(FLASH_DEV_NAME, &status, 0);
	ASSERT(DD_HANDLE_UNVALID != flash_hdl);
	bk_flash_enable_security(FLASH_PROTECT_NONE);

	GLOBAL_INT_DISABLE();
	ddev_read(flash_hdl, (char*)&flash_header, sizeof(flash_header), flash_vars_start);
	GLOBAL_INT_RESTORE();
	ddev_close(flash_hdl);
	bk_flash_enable_security(FLASH_PROTECT_ALL);

	switch (flash_header.magic) {
	case FLASH_VARS_MAGIC:
		if (flash_header.version != FLASH_VARS_VERS) {
			ADDLOGF_WARN("%s - versions don't match got %i want %i, may need upgrading", __func__, flash_header.version, FLASH_VARS_VERS);
			return false;
		}

		// check that the current header makes sense for what it claims to save
		uint32_t working_area = flash_header.vars_len - sizeof(flash_vars.footer);
		bool header_sanity = (working_area >= offsetof(FLASH_VARS_STRUCTURE, savedValues) + flash_header.number_ch * sizeof(flash_vars.savedValues[0])
#ifdef ENABLE_POWERMETERING
			&& working_area >= flash_header.emetering_offset + sizeof(flash_vars.emetering)
#endif // ENABLE_POWERMETERING
#ifdef ENABLE_DRIVER_LED
			&& working_area >= flash_header.led_offset + sizeof(flash_vars.led)
#endif // ENABLE_DRIVER_LED
			);
		if (!header_sanity) {
			ADDLOGF_WARN("%s - header details (vars_len: %i, number_ch: %i)", __func__, flash_header.vars_len, flash_header.number_ch);
			return false;
		}
		return true;
	
	case FLASH_VARS_MAGIC_V0:
		// return false, part needs upgrading even if we read previous vars
		ADDLOGF_WARN("%s - needs upgrading from old type flash vars", __func__);
		if (!flash_vars_read_v0())
			ADDLOGF_WARN("%s - upgrade failed", __func__);
		return false;

	default:
		ADDLOGF_ERROR("%s - 0x%X, unknown or blank flash vars part", __func__, flash_header.magic);
		return false;
	}
}

// reads vars from flash if it fails it will set the offset
// to read outside flash vars to trigger a part erase on the next
// flash vars write
static bool flash_vars_read() {
	UINT32 status;
	DD_HANDLE flash_hdl;
	// grab header footer and vars length
	uint32_t start_addr = flash_vars_start + sizeof(FLASH_VARS_HEADER);
	uint32_t read_entries = 0;
	uint32_t footer;
	GLOBAL_INT_DECLARATION();

	if (!flash_vars_part_valid()) {
		// magic is bad, set flash_vars_next to be at the end of the flash part
		// space, will trigger the vars partition erase on next write
		ADDLOGF_ERROR("%s - Header is marked bad", __func__);
		flash_vars_next = flash_vars_end;
		return false;
	}

	flash_hdl = ddev_open(FLASH_DEV_NAME, &status, 0);
	ASSERT(DD_HANDLE_UNVALID != flash_hdl);
	bk_flash_enable_security(FLASH_PROTECT_NONE);

	// keep loading footers until it is bad or we have no more to load
	do {
		read_entries++;
		start_addr += flash_header.vars_len;
		GLOBAL_INT_DISABLE();
		ddev_read(flash_hdl, (char*)&footer, sizeof(footer), start_addr - sizeof(footer));
		GLOBAL_INT_RESTORE();
	} while ((footer == FLASH_VARS_FOOTER) && (start_addr + flash_header.vars_len <= flash_vars_end));

	if (footer != FLASH_VARS_FOOTER && read_entries == 1) {
		// last entry's footer was corrupt, as expected
		// but we never read a good entry, not expected
		ADDLOGF_ERROR("%s - No flash vars, last footer 0x%X", __func__, footer);
		ddev_close(flash_hdl);
		bk_flash_enable_security(FLASH_PROTECT_ALL);
		flash_vars_next = flash_vars_end;
		return false;
	} else {
		// one backed off is the next write
		start_addr -= sizeof(FLASH_VARS_STRUCTURE);
		if (footer != 0xFFFFFFFF) {
			// not the expected bad footer reset just incase
			ADDLOGF_ERROR("%s - Unknown footer 0x%X", __func__, footer);
			flash_vars_next = flash_vars_end;
		} else {
			flash_vars_next = start_addr; 
		}
		// one more backed off is the read address
		start_addr -= sizeof(FLASH_VARS_STRUCTURE);
	}

	// checks for changes that need a partition reset
	if (flash_header.number_ch > MAX_RETAIN_CHANNELS) {
		// drop extra channels
		ADDLOGF_WARN("%s - firmware has only %i saved channels, down from %i", __func__, MAX_RETAIN_CHANNELS, flash_header.number_ch);
		flash_header.number_ch = MAX_RETAIN_CHANNELS;
		flash_vars_next = flash_vars_end;
	} else if (flash_header.number_ch < MAX_RETAIN_CHANNELS) {
		ADDLOGF_WARN("%s - firmware has %i saved channels, up from %i", __func__, MAX_RETAIN_CHANNELS, flash_header.number_ch);
		flash_vars_next = flash_vars_end;
	}

	if (flash_header.vars_len != sizeof(flash_vars)) {
		// shrink flash vars size
		ADDLOGF_WARN("%s - firmware flash vars size is %i bytes, shrunk from %i", __func__, sizeof(flash_vars), flash_header.vars_len);
		flash_vars_next = flash_vars_end;
	}
#ifdef ENABLE_POWERMETERING
	if (flash_header.emetering_offset != offsetof(FLASH_VARS_STRUCTURE, emetering))
		flash_vars_next = flash_vars_end;
#endif // ENABLE_DRIVER_LED
#ifdef ENABLE_DRIVER_LED
	if (flash_header.led_offset != offsetof(FLASH_VARS_STRUCTURE, led))
		flash_vars_next = flash_vars_end;
#endif // ENABLE_DRIVER_LED

	GLOBAL_INT_DISABLE();
	// read start plus any saved values based on the number of previously saved channels
	// or new lower maximum
	ddev_read(flash_hdl, (char*)&flash_vars, offsetof(FLASH_VARS_STRUCTURE, savedValues) + flash_header.number_ch * sizeof(flash_vars.savedValues[0]), start_addr);
	if (flash_header.emetering_offset)
#ifdef ENABLE_POWERMETERING
		ddev_read(flash_hdl, (char*)&flash_vars.emetering, sizeof(flash_vars.emetering), start_addr + flash_header.emetering_offset);
#else
		ADDLOGF_WARN("%s - firmware dropped support for emetering", __func__);
#endif // ENABLE_DRIVER_LED
	if (flash_header.led_offset)
#ifdef ENABLE_DRIVER_LED
		ddev_read(flash_hdl, (char*)&flash_vars.led, sizeof(flash_vars.led), start_addr + flash_header.led_offset);
#else
		ADDLOGF_WARN("%s - firmware dropped support for led", __func__);
#endif // ENABLE_DRIVER_LED
	GLOBAL_INT_RESTORE();

	ddev_close(flash_hdl);
	bk_flash_enable_security(FLASH_PROTECT_ALL);

	ADDLOGF_DEBUG("new addr after read 0x%X, boot_count %d, success count %d",
		flash_vars_next, flash_vars.boot_count, flash_vars.boot_success_count
	);
	return true;
}

// called by HAL FlashVars IncreaseBootCount
// sets up location of flash vars
// returns failed if flash read fails
static void flash_vars_init() {
	bk_logic_partition_t* pt;
	pt = bk_flash_get_info(BK_PARTITION_NET_PARAM);
	// there is an EXTRA sctor used for some form of wifi?
	// on T variety, this is 0x1e3000
	flash_vars_start = pt->partition_start_addr + pt->partition_length + 0x1000;
	flash_vars_end = flash_vars_start + flash_vars_part_len;
}

// erase vars partition
// write new vars partition header
// set vars offset
static void flash_vars_reset_part() {
	uint32_t param;
	UINT32 status;
	DD_HANDLE flash_hdl;
	FLASH_VARS_HEADER header = {
		.magic = FLASH_VARS_MAGIC,
		.version = FLASH_VARS_VERS,
		.vars_len = sizeof(FLASH_VARS_STRUCTURE),
		.number_ch = MAX_RETAIN_CHANNELS,
#ifdef ENABLE_POWERMETERING
		.emetering_offset = offsetof(FLASH_VARS_STRUCTURE, emetering),
#else
		.emetering_offset = 0,
#endif // ENABLE_POWERMETERING
#ifdef ENABLE_DRIVER_LED
		.led_offset = offsetof(FLASH_VARS_STRUCTURE, led),
#endif // ENABLE_DRIVER_LED
		.led_offset = 0,
		.footer = FLASH_VARS_FOOTER
	};

	GLOBAL_INT_DECLARATION();

	bk_flash_enable_security(FLASH_PROTECT_NONE);

	flash_hdl = ddev_open(FLASH_DEV_NAME, &status, 0);
	ASSERT(DD_HANDLE_UNVALID != flash_hdl);

	ADDLOGF_INFO("%s - erasing vars partition", __func__);
	for (uint32_t i = 0; i < flash_vars_part_len / flash_vars_sector_len; i++)
	{
		param = flash_vars_start + (i * flash_vars_sector_len);
		GLOBAL_INT_DISABLE();
		ddev_control(flash_hdl, CMD_FLASH_ERASE_SECTOR, (void*)&param);
		GLOBAL_INT_RESTORE();
	}

	ADDLOGF_INFO("%s - writing new partition header", __func__);
	GLOBAL_INT_DISABLE();
	ddev_write(flash_hdl, (char*)&header, sizeof(header), flash_vars_start);
	GLOBAL_INT_RESTORE();

	ddev_close(flash_hdl);
	bk_flash_enable_security(FLASH_PROTECT_ALL);

	flash_vars_next = flash_vars_start + sizeof(header);
	ADDLOGF_DEBUG("%s - vars addr set to %i", __func__, flash_vars_next);
}

// check size fits in partition
// reset partition if it does not
// write vars data to write offset
// increased write offset
static bool _flash_vars_write() {
	UINT32 status;
	DD_HANDLE flash_hdl;

	if (flash_vars_next + sizeof(flash_vars) > flash_vars_end) {
		ADDLOGF_INFO("%s - partition marked for reset", __func__);
		flash_vars_reset_part();
	}

	GLOBAL_INT_DECLARATION();

	flash_hdl = ddev_open(FLASH_DEV_NAME, &status, 0);
	ASSERT(DD_HANDLE_UNVALID != flash_hdl);
	bk_flash_enable_security(FLASH_PROTECT_NONE);

	GLOBAL_INT_DISABLE();
	ddev_write(flash_hdl, (char*)&flash_vars, sizeof(flash_vars), flash_vars_next);
	GLOBAL_INT_RESTORE();
	ddev_close(flash_hdl);
	bk_flash_enable_security(FLASH_PROTECT_ALL);

	flash_vars_next += sizeof(flash_vars);

	ADDLOGF_DEBUG("new addr 0x%X, boot_count %d, success count %d",
		flash_vars_next, flash_vars.boot_count, flash_vars.boot_success_count);
	return true;
}

// only time to force a write is for boot count
static bool flash_vars_write(bool forcedWrite) {
	if (g_flashVars_safeToWrite || forcedWrite)
		return _flash_vars_write();
	ADDLOGF_WARN("%s - failed to write to flash, not safe");
	return false;
}

//#define DISABLE_FLASH_VARS_VARS

// call at startup ONLY
// will increase boot count ONLY if flash vars are read correctly
// will ERASE all flash vars in memory and pull flash values
void HAL_FlashVars_IncreaseBootCount() {
#ifndef DISABLE_FLASH_VARS_VARS
	flash_vars_init();
	if (!flash_vars_read())
		return;

	flash_vars.boot_count++;
	if (flash_vars.boot_count - flash_vars.boot_success_count > RESTARTS_REQUIRED_FOR_SAFE_MODE) {
		ADDLOGF_WARN("####### Failed Count %d #######", flash_vars.boot_count - flash_vars.boot_success_count);
		return;
	}
	ADDLOGF_INFO("####### Boot Count %d #######", flash_vars.boot_count);
	flash_vars_write(true);
#endif
}

void HAL_FlashVars_SaveChannel(int index, int value) {
#ifndef DISABLE_FLASH_VARS_VARS
	if (index < 0 || index >= MAX_RETAIN_CHANNELS) {
		ADDLOGF_WARN("####### Flash Save Can't Save Channel %d as %d (not enough space in array) #######", index, value);
		return;
	}

	flash_vars.savedValues[index] = value;
	ADDLOGF_INFO("####### Flash Save Channel %d as %d #######", index, value);
	flash_vars_write(false);
#endif
}
#ifdef ENABLE_DRIVER_LED
void HAL_FlashVars_ReadLED(byte* mode, short* brightness, short* temperature, byte* rgb, byte* bEnableAll) {
#ifndef DISABLE_FLASH_VARS_VARS
	* bEnableAll = flash_vars.led.bEnableAll;
	*mode = flash_vars.led.mode;
	*temperature = flash_vars.led.temperature;
	*brightness = flash_vars.led.brightness;
	rgb[0] = flash_vars.led.red;
	rgb[1] = flash_vars.led.green;
	rgb[2] = flash_vars.led.blue;
#endif
}
#define SAVE_CHANGE_IF_REQUIRED_AND_COUNT(target, source, counter) \
	if((target) != (source)) { \
		(target) = (source); \
		counter++; \
	}

void HAL_FlashVars_SaveLED(byte mode, short brightness, short temperature, byte r, byte g, byte b, byte bEnableAll) {
#ifndef DISABLE_FLASH_VARS_VARS
	int iChangesCount = 0;

	SAVE_CHANGE_IF_REQUIRED_AND_COUNT(flash_vars.led.brightness, brightness, iChangesCount);
	SAVE_CHANGE_IF_REQUIRED_AND_COUNT(flash_vars.led.temperature, temperature, iChangesCount);
	SAVE_CHANGE_IF_REQUIRED_AND_COUNT(flash_vars.led.mode, mode, iChangesCount);
	SAVE_CHANGE_IF_REQUIRED_AND_COUNT(flash_vars.led.bEnableAll, bEnableAll, iChangesCount);
	SAVE_CHANGE_IF_REQUIRED_AND_COUNT(flash_vars.led.red, r, iChangesCount);
	SAVE_CHANGE_IF_REQUIRED_AND_COUNT(flash_vars.led.green, g, iChangesCount);
	SAVE_CHANGE_IF_REQUIRED_AND_COUNT(flash_vars.led.blue, b, iChangesCount);

	if (iChangesCount > 0) {
		ADDLOGF_INFO("####### Flash Save LED #######");
		if (!flash_vars_write(false))
			return;
	}
#endif
}
#endif // ENABLE_DRIVER_LED

#ifdef ENABLE_POWERMETERING
// WARNING: don't do this if I use this make it proper add to emetering or something modular
short HAL_FlashVars_ReadUsage() {
#ifndef DISABLE_FLASH_VARS_VARS
	return  flash_vars.savedValues[MAX_RETAIN_CHANNELS - 1];
#endif
}
void HAL_FlashVars_SaveTotalUsage(short usage) {
#ifndef DISABLE_FLASH_VARS_VARS
	flash_vars.savedValues[MAX_RETAIN_CHANNELS - 1] = usage;
	ADDLOGF_INFO("####### Flash Save Usage #######");
	if (!flash_vars_write(false))
		return;
#endif
}
#endif // ENABLE_POWERMETERING
// call once started (>30s?)
void HAL_FlashVars_SaveBootComplete() {
#ifndef DISABLE_FLASH_VARS_VARS
	// mark that we have completed a boot.
	ADDLOGF_INFO("####### Set Boot Complete #######");

	flash_vars.boot_success_count = flash_vars.boot_count;
	g_flashVars_safeToWrite = true;
	if (!flash_vars_write(false))
		return;
#endif
}

// call to return the number of boots since a HAL_FlashVars_SaveBootComplete
int HAL_FlashVars_GetBootFailures() {
#ifndef DISABLE_FLASH_VARS_VARS
	// if boot_count was never updated, mark for safe mode
	if (!flash_vars.boot_count)
		return RESTARTS_REQUIRED_FOR_SAFE_MODE + 1;
	return flash_vars.boot_count - flash_vars.boot_success_count;
#endif
	return 0;
}

int HAL_FlashVars_GetBootCount() {
	return flash_vars.boot_count;
}
int HAL_FlashVars_GetChannelValue(int ch) {
	if (ch < 0 || ch >= MAX_RETAIN_CHANNELS) {
		ADDLOGF_INFO("####### Flash Save Can't Get Channel %d (not enough space in array) #######", ch);
		return 0;
	}
	return flash_vars.savedValues[ch];
}
