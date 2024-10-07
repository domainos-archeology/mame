
#include <cstdint>

#ifndef MAME_ANSI_DISK_CORE_H
#define MAME_ANSI_DISK_CORE_H

// These pins correspond to the ansi cable, not to any particular hardware
// platform.  There will likely be a translation layer needed.
typedef enum {
	CONTROL_BUS_0,
	CONTROL_BUS_1,
	CONTROL_BUS_2,
	CONTROL_BUS_3,
	CONTROL_BUS_4,
	CONTROL_BUS_5,
	CONTROL_BUS_6,
	CONTROL_BUS_7,

	SELECT_OUT_ATTN_IN_STROBE,
	COMMAND_REQUEST,
	PARAMETER_REQUEST,
	BUS_DIRECTION_OUT,
	PORT_ENABLE,
	READ_GATE,
	WRITE_GATE,
	BUS_ACKNOWLEDGE,
	INDEX,
	SECTOR_MARK,
	ATTENTION,
	BUSY,

	// these are differential pins in the ansi spec but the core treats them as
	// 0/1 logicals pins.
	READ_DATA,
	READ_REFERENCE_CLOCK,
	WRITE_CLOCK,
	WRITE_DATA,
} ANSIPin;

typedef enum {
	ANSI_CMD_REPORT_ILLEGAL_COMMAND     = 0x00,
	ANSI_CMD_CLEAR_FAULT                = 0x01,
	ANSI_CMD_CLEAR_ATTENTION            = 0x02,
	ANSI_CMD_SEEK                       = 0x03,
	ANSI_CMD_REZERO				        = 0x04,
	ANSI_CMD_REPORT_SENSE_BYTE_2        = 0x0D,
	ANSI_CMD_REPORT_SENSE_BYTE_1        = 0x0E,
	ANSI_CMD_REPORT_GENERAL_STATUS      = 0x0F,
// --
	ANSI_CMD_REPORT_ATTRIBUTE           = 0x10,
	ANSI_CMD_SET_ATTENTION              = 0x11,
	ANSI_CMD_SELECTIVE_RESET            = 0x14,
	ANSI_CMD_SEEK_TO_LANDING_ZONE       = 0x15,
	ANSI_CMD_REFORMAT_TRACK             = 0x16,
// --
	ANSI_CMD_REPORT_CYL_ADDR_HIGH       = 0x29,
	ANSI_CMD_REPORT_CYL_ADDR_LOW        = 0x2A,
	ANSI_CMD_REPORT_READ_PERMIT_HIGH    = 0x2B,
	ANSI_CMD_REPORT_READ_PERMIT_LOW     = 0x2C,
	ANSI_CMD_REPORT_WRITE_PERMIT_HIGH   = 0x2D,
	ANSI_CMD_REPORT_WRITE_PERMIT_LOW    = 0x2E,
	ANSI_CMD_REPORT_TEST_BYTE           = 0x2F,
// --
	ANSI_CMD_ATTENTION_CONTROL          = 0x40,
	ANSI_CMD_WRITE_CONTROL              = 0x41,
	ANSI_CMD_LOAD_CYL_ADDR_HIGH         = 0x42,
	ANSI_CMD_LOAD_CYL_ADDR_LOW          = 0x43,
	ANSI_CMD_SELECT_HEAD                = 0x44,
// --
	ANSI_CMD_LOAD_ATTRIBUTE_NUMBER      = 0x50,
	ANSI_CMD_LOAD_ATTRIBUTE             = 0x51,
	ANSI_CMD_READ_CONTROL               = 0x53,
	ANSI_CMD_OFFSET_CONTROL             = 0x54,
	ANSI_CMD_SPIN_CONTROL               = 0x55,
	ANSI_CMD_LOAD_SECT_PER_TRACK_HIGH   = 0x56, // MSB
	ANSI_CMD_LOAD_SECT_PER_TRACK_MEDIUM = 0x57, // MedSB
	ANSI_CMD_LOAD_SECT_PER_TRACK_LOW    = 0x58, // LSB
	ANSI_CMD_LOAD_BYTES_PER_SECT_HIGH   = 0x59, // MSB
	ANSI_CMD_LOAD_BYTES_PER_SECT_MEDIUM = 0x5A, // MedSB
	ANSI_CMD_LOAD_BYTES_PER_SECT_LOW    = 0x5B, // LSB
// --
	ANSI_CMD_LOAD_READ_PERMIT_HIGH      = 0x6B,
	ANSI_CMD_LOAD_READ_PERMIT_LOW       = 0x6C,
	ANSI_CMD_LOAD_WRITE_PERMIT_HIGH     = 0x6D,
	ANSI_CMD_LOAD_WRITE_PERMIT_LOW      = 0x6E,
	ANSI_CMD_LOAD_TEST_BYTE             = 0x6F,
} ANSICmd;

// general status bits
#define GS_NOT_READY         0x01
#define GS_CONTROL_BUS_ERROR 0x02
#define GS_ILLEGAL_COMMAND   0x04
#define GS_ILLEGAL_PARAMETER 0x08
#define GS_SENSE_BYTE_1      0x10
#define GS_SENSE_BYTE_2      0x20
#define GS_BUSY_EXECUTING    0x40
#define GS_NORMAL_COMPLETE   0x80

// sense byte 1 bits
#define SB1_SEEK_ERROR          0x01
#define SB1_RW_FAULT            0x02
#define SB1_POWER_FAULT         0x04
#define SB1_RW_PERMIT_VIOLATION 0x08
#define SB1_SPEED_ERROR         0x10
#define SB1_COMMAND_REJECT      0x20
#define SB1_OTHER_ERRORS        0x40
#define SB1_VENDOR_ERRORS       0x80

// sense byte 2 bits
#define SB2_INITIAL_STATE                          0x01
#define SB2_READY_TRANSITION                       0x02
#define SB2_DEV_RESERVED_TO_THIS_POINT             0x04
#define SB2_FORCED_RELEASE                         0x08
#define SB2_DEV_RESERVED_TO_ALT_PORT               0x10
#define SB2_DEVICE_ATTR_TABLE_MODIFIED             0x20
#define SB2_POSITIONED_WITHIN_WRITE_PROTECTED_AREA 0x40
#define SB2_VENDOR_ATTNS						   0x80

class ANSIDisk;

class ANSIDiskPlatform {
public:
	void GetDiskInfo   (uint16_t *type, uint32_t *cylinders, uint32_t *heads, uint32_t *sectors_per_track, uint32_t *bytes_per_sector);
	void Shutdown      ();

	int  ReadPin       (ANSIPin pin);
	void AssertPin     (ANSIPin pin);
	void DeassertPin   (ANSIPin pin);
	void RegisterPinIRQ(ANSIPin pin, void (*callback)(ANSIDisk* disk, ANSIPin pin, int state));

	void ReadBlockData (uint32_t block, uint8_t *buffer, uint16_t buffer_len);
	void WriteBlockData(uint32_t block, uint8_t *buffer, uint16_t buffer_len);
	void Log           (const char* fmt, ...);
};

class ANSIDisk {
public:
	ANSIDisk(ANSIDiskPlatform* platform);

	void Initialize();
	void Destroy();

private:
	void set_sb2_bits(uint8_t bits);
	void clear_sb2_bits(uint8_t bits);

	uint16_t m_type;
	uint16_t m_cylinders;
	uint16_t m_heads;
	uint16_t m_sectors;
	uint32_t m_sectorbytes;
	uint32_t m_sector_count;

	bool m_attention;
	bool m_selected;
	bool m_write_enabled;
	bool m_attention_enabled;

	// these two cylinder bytes are for the actual current cylinder address
	uint8_t m_current_cylinder_high;
	uint8_t m_current_cylinder_low;
	// and these next two are the values loaded by the load_cylinder commands.
	// they will become the current ones above upon a successful seek.
	uint8_t m_load_cylinder_high;
	uint8_t m_load_cylinder_low;

	uint8_t m_attribute_number;
	uint8_t m_test_byte;

	uint8_t m_selected_head;
	uint8_t m_general_status;
	uint8_t m_sense_byte_1;
	uint8_t m_sense_byte_2;

	ANSIDiskPlatform *m_platform;
};

#endif // MAME_ANSI_DISK_CORE_H
