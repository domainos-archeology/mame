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
#define ANSI_DISK_TYPE_HACK_DN3500 0xffff// allow us to do the right thing at bootup sharing the disk image with a big esdi disk from a dn3500


// forward declaration of image class
DECLARE_DEVICE_TYPE(ANSI_DISK_DEVICE, ansi_disk_device)

class ansi_disk_device : public harddisk_image_base_device
{
public:
	typedef delegate<void (ansi_disk_device *, bool)> bool_cb;
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

	void set_attention_cb(bool_cb cb);
	void set_busy_cb(bool_cb cb);
	void set_read_data_cb(read_data_cb cb);
	void set_ref_clock_tick_cb(pulse_cb cb);
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

	void set_bus_direction_out(uint8_t v);
	void set_command_request(uint8_t v);
	void set_parameter_request(uint8_t v);

protected:
	// device-level overrides
	virtual void device_resolve_objects() override;
	virtual void device_start() override;
	virtual void device_reset() override;

	void ansi_disk_config(uint16_t disk_type);

private:
	// not all possible state changes are listed here.  only the ones that are
	// not immediately responded to.
	enum State {
		// inactive state - we're here if PORT_ENABLE is not active.
		STATE_INACTIVE,

		// active state - we were in STATE_INACTIVE and PORT_ENABLE was just
		// enabled.
		STATE_ACTIVE,

	    // drive is selected by the host.
		STATE_SELECTED,

		// part of command handshake.  we've gotten the command byte and it's
		// tagged with an out parameter, so we're waiting for Parameter_Request
		// and will then read it and execute the command.
		STATE_AWAITING_PARAMETER_OUT,

		// part of the command handshake.  we've gotten the command byte and
		// it's tagged with an in parameter, so we should already be executing
		// the command (or have finished.)
		STATE_AWAITING_PARAMETER_IN,
	};
	emu_timer *m_time_dependent_timer;
	void start_time_dependent_command(attotime duration);
	TIMER_CALLBACK_MEMBER(finish_time_dependent_command);

	// the timer that drives the read_sector code
	emu_timer *m_read_timer;
	TIMER_CALLBACK_MEMBER(read_clock_tick);

	// the timer that drives the write_sector code
	emu_timer *m_ref_timer;
	TIMER_CALLBACK_MEMBER(ref_clock_tick);

	// our attention line
	bool m_attention;
	void set_attention_line(bool state);

	bool m_busy;
	void set_busy_line(bool state);

	bool_cb cur_attention_cb;
	bool_cb cur_busy_cb;
	read_data_cb cur_read_data_cb;
	pulse_cb cur_ref_clock_tick_cb;
	pulse_cb cur_index_pulse_cb;
	pulse_cb cur_sector_pulse_cb;

	bool m_selected;

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
	char m_buffer[1056]; // really should be HARD_DISK_SECTOR_SIZE, once that moves someplace common.

	attotime m_sector_clock_freq;
	emu_timer *m_sector_timer;
	TIMER_CALLBACK_MEMBER(sector_callback);
	int m_pulsed_sector;
};

#endif // MAME_ANSI_DISK_DEVICE_H
