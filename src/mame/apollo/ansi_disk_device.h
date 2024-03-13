#include "emu.h"
#include "imagedev/harddriv.h"

#ifndef MAME_ANSI_DISK_DEVICE_H
#define MAME_ANSI_DISK_DEVICE_H

// software ends up sizing the micropolis to the same size as the priam, so we ignore it.
// #define ANSI_DISK_TYPE_38_MB 0x103 // Micropolis 1203 (38MB unformatted Dtype = 103)
#define ANSI_DISK_TYPE_32_MB 0x104 // Priam 3450 (35MB unformatted Dtype = 104)
#define ANSI_DISK_TYPE_64_MB 0x105 // Priam 7050 (70MB unformatted Dtype = 105)
// There's another (unknown) disk type supported in /sau2/disk (Dtype low = 35, 4 heads, 502 cylinders, 15 blocks/track)
#define ANSI_DISK_TYPE_DEFAULT ANSI_DISK_TYPE_64_MB  // new disks will have this type (and size)
#define ANSI_DISK_TYPE_HACK_DN3500 0xffff// allow us to do the right thing at bootup sharing the disk image with a big esdi disk


// ANSI commands
#define ANSI_CMD_REPORT_ILLEGAL_COMMAND     0x00
#define ANSI_CMD_CLEAR_FAULT                0x01
#define ANSI_CMD_CLEAR_ATTENTION            0x02
#define ANSI_CMD_SEEK                       0x03
#define ANSI_CMD_REZERO				        0x04
#define ANSI_CMD_REPORT_SENSE_BYTE_2        0x0D
#define ANSI_CMD_REPORT_SENSE_BYTE_1        0x0E
#define ANSI_CMD_REPORT_GENERAL_STATUS      0x0F
// --
#define ANSI_CMD_REPORT_ATTRIBUTE           0x10
#define ANSI_CMD_SET_ATTENTION              0x11
#define ANSI_CMD_SELECTIVE_RESET            0x14
#define ANSI_CMD_SEEK_TO_LANDING_ZONE       0x15
#define ANSI_CMD_REFORMAT_TRACK             0x16
// --
#define ANSI_CMD_REPORT_CYL_ADDR_HIGH       0x29
#define ANSI_CMD_REPORT_CYL_ADDR_LOW        0x2A
#define ANSI_CMD_REPORT_READ_PERMIT_HIGH    0x2B
#define ANSI_CMD_REPORT_READ_PERMIT_LOW     0x2C
#define ANSI_CMD_REPORT_WRITE_PERMIT_HIGH   0x2D
#define ANSI_CMD_REPORT_WRITE_PERMIT_LOW    0x2E
#define ANSI_CMD_REPORT_TEST_BYTE           0x2F
// --
#define ANSI_CMD_ATTENTION_CONTROL          0x40
#define ANSI_CMD_WRITE_CONTROL              0x41
#define ANSI_CMD_LOAD_CYL_ADDR_HIGH         0x42
#define ANSI_CMD_LOAD_CYL_ADDR_LOW          0x43
#define ANSI_CMD_SELECT_HEAD                0x44
// --
#define ANSI_CMD_LOAD_ATTRIBUTE_NUMBER      0x50
#define ANSI_CMD_LOAD_ATTRIBUTE             0x51
#define ANSI_CMD_READ_CONTROL               0x53
#define ANSI_CMD_OFFSET_CONTROL             0x54
#define ANSI_CMD_SPIN_CONTROL               0x55
#define ANSI_CMD_LOAD_SECT_PER_TRACK_HIGH   0x56 // MSB
#define ANSI_CMD_LOAD_SECT_PER_TRACK_MEDIUM 0x57 // MedSB
#define ANSI_CMD_LOAD_SECT_PER_TRACK_LOW    0x58 // LSB
#define ANSI_CMD_LOAD_BYTES_PER_SECT_HIGH   0x59 // MSB
#define ANSI_CMD_LOAD_BYTES_PER_SECT_MEDIUM 0x5A // MedSB
#define ANSI_CMD_LOAD_BYTES_PER_SECT_LOW    0x5B // LSB
// --
#define ANSI_CMD_LOAD_READ_PERMIT_HIGH      0x6B
#define ANSI_CMD_LOAD_READ_PERMIT_LOW       0x6C
#define ANSI_CMD_LOAD_WRITE_PERMIT_HIGH     0x6D
#define ANSI_CMD_LOAD_WRITE_PERMIT_LOW      0x6E
#define ANSI_CMD_LOAD_TEST_BYTE             0x6F

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


// forward declaration of image class
DECLARE_DEVICE_TYPE(ANSI_DISK_DEVICE, ansi_disk_device)

class ansi_disk_device : public harddisk_image_base_device
{
public:
	typedef delegate<void (ansi_disk_device *, bool)> attention_cb;
	typedef delegate<void (ansi_disk_device *, uint8_t)> read_data_cb;
	typedef delegate<void (ansi_disk_device *)> pulse_cb;

	// construction/destruction
	ansi_disk_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// image-level overrides
	virtual bool support_command_line_image_creation() const noexcept override { return true; }
	virtual const char *file_extensions() const noexcept override { return "awd"; }
	virtual const char *image_type_name() const noexcept override { return "winchester"; }
	virtual const char *image_brief_type_name() const noexcept override { return "disk"; }

	virtual image_init_result call_create(int format_type, util::option_resolution *format_options) override;

	void set_attention_cb(attention_cb cb);
	void set_read_data_cb(read_data_cb cb);
	void set_index_pulse_cb(pulse_cb cb);
	void set_sector_pulse_cb(pulse_cb cb);

	// I can't get dev_write_line's to work, so using these pairs of functions insteead.
	void assert_read_gate();
	void deassert_read_gate();

	void assert_write_gate();
	void write_sector_next_byte(uint8_t data);
	void deassert_write_gate();

	void select();
	void deselect();

    uint8_t execute_command(uint8_t command, uint8_t parameter);

protected:
	// device-level overrides
	virtual void device_resolve_objects() override;
	virtual void device_start() override;
	virtual void device_reset() override;

	void ansi_disk_config(uint16_t disk_type);

	// XXX(toshok)once read/write record are here, switch this back to private
public:
	emu_timer *m_time_dependent_timer;
	void start_time_dependent_command(attotime duration);
	TIMER_CALLBACK_MEMBER(finish_time_dependent_command);

	// the timer that drives the read_sector code
	emu_timer *m_read_timer;
	TIMER_CALLBACK_MEMBER(read_sector_next_byte);

	// our attention line
	bool m_attention;
	void set_attention_line(bool state);

	attention_cb cur_attention_cb;
	read_data_cb cur_read_data_cb;
	pulse_cb cur_index_pulse_cb;
	pulse_cb cur_sector_pulse_cb;

    template <typename Format, typename... Params> void logerror(Format &&fmt, Params &&... args) const;

	void set_sb1(uint8_t value);
	void clear_sb1(uint8_t value);
	void set_sb2(uint8_t value);
	void clear_sb2(uint8_t value);

	uint16_t m_type;
	uint16_t m_cylinders;
	uint16_t m_heads;
	uint16_t m_sectors;
	uint32_t m_sectorbytes;
	uint32_t m_sector_count;

	device_image_interface *m_image;

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
	bool m_write_enabled;
	bool m_attention_enabled;

	// configuration data
    uint8_t report_attribute();
    void load_attribute(uint8_t attribute_value);

    bool m_attributes_initialized;
	uint8_t m_ansi_attributes[0x48];

	uint32_t m_cursor;
	char m_buffer[2000]; // really only need 1056 here.

	attotime m_sector_clock_freq;
	emu_timer *m_sector_timer;
	TIMER_CALLBACK_MEMBER(sector_callback);
	int m_pulsed_sector;
};

#endif // MAME_ANSI_DISK_DEVICE_H
