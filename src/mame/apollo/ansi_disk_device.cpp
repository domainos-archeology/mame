#include "ansi_disk_device.h"

#define VERBOSE 1
#include "apollo_dn300.h"

// XXX dedup this with the one in apollo_dn300_disk.cpp
#define HARD_DISK_SECTOR_SIZE 1056

DEFINE_DEVICE_TYPE(ANSI_DISK_DEVICE, ansi_disk_device, "ansi_disk_device", "ANSI disk")

ansi_disk_device::ansi_disk_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: harddisk_image_base_device(mconfig, ANSI_DISK_DEVICE, tag, owner, clock)
	, m_attention(false)
	, cur_attention_cb()
	, cur_read_data_cb()
	, cur_index_pulse_cb()
	, cur_sector_pulse_cb()
	, m_type(0)
	, m_cylinders(0)
	, m_heads(0)
	, m_sectors(0)
	, m_sectorbytes(0)
	, m_sector_count(0)
	, m_image(nullptr)
    , m_current_cylinder_high(0)
    , m_current_cylinder_low(0)
    , m_load_cylinder_high(0)
    , m_load_cylinder_low(0)
    , m_attribute_number(0)
	, m_test_byte(0)
    , m_selected_head(0)
    , m_general_status(0)
    , m_sense_byte_1(0)
    , m_sense_byte_2(0)
    , m_write_enabled(true)
    , m_attention_enabled(true)
    // XXX more things here moved from the controller
{
}

void ansi_disk_device::set_attention_cb(attention_cb cb)
{
    cur_attention_cb = cb;
}

void ansi_disk_device::set_read_data_cb(read_data_cb cb)
{
    cur_read_data_cb = cb;
}

void ansi_disk_device::set_index_pulse_cb(pulse_cb cb)
{
    cur_index_pulse_cb = cb;
}

void ansi_disk_device::set_sector_pulse_cb(pulse_cb cb)
{
    cur_sector_pulse_cb = cb;
}

/***************************************************************************
 ansi_disk_config - configure disk parameters
 ***************************************************************************/

void ansi_disk_device::ansi_disk_config(uint16_t disk_type)
{
	logerror("ansi_disk_config: configuring disk with type %x\n", disk_type);

	switch (disk_type)
	{
		// Priam 8" disks used in the dn300 (7050 and 3040) both look to have the same rpm,
		// so index pulse period = 16.67ms.  given 12 sectors, that's 1.39ms per sector.
        case ANSI_DISK_TYPE_64_MB: // Priam 7050 (Unformatted 70MB)
            m_cylinders = 1049;
            m_heads = 5;
            m_sectors = 12;
			m_sector_clock_freq = attotime::from_usec(1390);
            break;

        case ANSI_DISK_TYPE_32_MB: // Priam 3450 (Unformatted 35MB)
            m_cylinders = 525;
            m_heads = 5;
            m_sectors = 12;
			m_sector_clock_freq = attotime::from_usec(1390);
            break;

        case ANSI_DISK_TYPE_HACK_DN3500: // Maxtor 380 MB (348-MB FA formatted)
            m_cylinders = 1223;
            m_heads = 15;
            m_sectors = 18;
			// XXX no clue what value to use here, so let's just keep it what the
			// priam drives used.  just in case there's some code that depends on
			// it.
			m_sector_clock_freq = attotime::from_usec(1390);
            break;
    }

	m_type = disk_type;
	m_sectorbytes = HARD_DISK_SECTOR_SIZE;
	m_sector_count = m_cylinders * m_heads * m_sectors;
}

/*-------------------------------------------------
 logerror - log an error message (w/o device tags)
 -------------------------------------------------*/

template <typename Format, typename... Params>
void ansi_disk_device::logerror(Format &&fmt, Params &&... args) const
{
	machine().logerror(std::forward<Format>(fmt), std::forward<Params>(args)...);
}

void ansi_disk_device::device_resolve_objects()
{
    SLOG1(("ansi_disk_device(%s/%p)::device_resolve_objects", tag(), this));
}

/*-------------------------------------------------
    device start callback
-------------------------------------------------*/

void ansi_disk_device::device_start()
{
	logerror("ansi_disk_device(%p)::device_start\n", this);

	m_image = this;

	if (!m_image->is_open())
	{
		logerror("device_start_ansi_disk: no disk\n");
	}
	else
	{
		logerror("device_start_ansi_disk: with disk image %s\n", m_image->basename());
	}

	// default disk type
	ansi_disk_config(ANSI_DISK_TYPE_DEFAULT);
    m_time_dependent_timer = timer_alloc(FUNC(ansi_disk_device::finish_time_dependent_command), this);
    m_read_timer = timer_alloc(FUNC(ansi_disk_device::read_sector_next_byte), this);

	m_pulsed_sector = 0;
	m_sector_timer = timer_alloc(FUNC(ansi_disk_device::sector_callback), this);
}

/*-------------------------------------------------
    device reset callback
-------------------------------------------------*/

void ansi_disk_device::device_reset()
{
	logerror("ansi_disk_device(%p)::device_reset\n", this);

	if (exists() && !fseek(0, SEEK_END))
	{
		uint32_t disk_size = uint32_t(ftell() / HARD_DISK_SECTOR_SIZE);
		uint16_t disk_type;

        if (disk_size >= 300000) {
            disk_type = ANSI_DISK_TYPE_HACK_DN3500;
        } else if (disk_size >= 60000) {
            disk_type = ANSI_DISK_TYPE_64_MB;
        } else {
            disk_type = ANSI_DISK_TYPE_32_MB;
        }
		if (disk_type != m_type) {
			logerror("device_reset_ansi_disk: disk size=%d blocks, disk type=%x\n", disk_size, disk_type);
			ansi_disk_config(disk_type);
		}
	}
}

/*-------------------------------------------------
   disk image create callback
-------------------------------------------------*/

image_init_result ansi_disk_device::call_create(int format_type, util::option_resolution *format_options)
{
	logerror("device_create_ansi_disk: creating ANSI Disk with %d blocks\n", m_sector_count);

	unsigned char sectordata[HARD_DISK_SECTOR_SIZE]; // empty block data
	memset(sectordata, 0x55, sizeof(sectordata));

	for (int x = 0; x < m_sector_count; x++)
	{
		if (fwrite(sectordata, HARD_DISK_SECTOR_SIZE)
				< HARD_DISK_SECTOR_SIZE)
		{
			return image_init_result::FAIL;
		}
	}

	return image_init_result::PASS;
}

uint8_t ansi_disk_device::report_attribute()
{
    /* ensure our attributes have been initialized */
    if (!m_attributes_initialized) {
        m_attributes_initialized = true;
        memset(m_ansi_attributes, 0, sizeof(m_ansi_attributes));

        m_ansi_attributes[0x00] = 0x00;          // User ID - user defined
	    m_ansi_attributes[0x01] = m_type >> 8;   // Model ID High - vendor defined
        m_ansi_attributes[0x02] = m_type & 0xff; // Model ID Low - vendor defined
        m_ansi_attributes[0x03] = 0x00;          // Revision ID - vendor defined

	    m_ansi_attributes[0x0D] = 0x00; // Device Type ID - device dependent
	    m_ansi_attributes[0x0E] = 0x00; // Table Modification - action dependent
	    m_ansi_attributes[0x0F] = 0x00; // Table ID - vendor defined

        uint32_t bytes_per_track = m_sectors * HARD_DISK_SECTOR_SIZE;
	    m_ansi_attributes[0x10] = (bytes_per_track >> 16) & 0xff;       // MSB of # of bytes per track
	    m_ansi_attributes[0x11] = (bytes_per_track >> 8) & 0xff;        // MedSB of # of bytes per track
	    m_ansi_attributes[0x12] = bytes_per_track & 0xff;               // LSB of # of bytes per track
	    m_ansi_attributes[0x13] = (HARD_DISK_SECTOR_SIZE >> 16) & 0xff; // MSB of # of bytes per sector
	    m_ansi_attributes[0x14] = (HARD_DISK_SECTOR_SIZE >> 8) & 0xff;  // MedSB of # of bytes per sector
	    m_ansi_attributes[0x15] = HARD_DISK_SECTOR_SIZE & 0xff;         // LSB of # of bytes per sector
	    m_ansi_attributes[0x16] = 0x00;                                 // MSB of # of sector pulses per track
	    m_ansi_attributes[0x17] = m_sectors >> 8;                       // MedSB of # of sector pulses per track
	    m_ansi_attributes[0x18] = m_sectors & 0xff;                     // LSB of # of sector pulses per track
	    m_ansi_attributes[0x19] = 0x00;                                 // Sectoring method

	    m_ansi_attributes[0x20] = m_cylinders >> 8;              // MSB of # of cylinders
	    m_ansi_attributes[0x21] = m_cylinders & 0xff;            // LSB of # of cylinders
	    m_ansi_attributes[0x22] = m_heads;                       // Number of heads

	    m_ansi_attributes[0x30] = 0x00;                          // Encoding method #1
	    m_ansi_attributes[0x31] = 0x00;                          // Preamble #1 number of bytes
	    m_ansi_attributes[0x32] = 0x00;                          // Preamble #1 pattern
	    m_ansi_attributes[0x33] = 0x00;                          // Sync #1 pattern
	    m_ansi_attributes[0x34] = 0x00;                          // Postamble #1 number of bytes
	    m_ansi_attributes[0x35] = 0x00;                          // Postamble #1 pattern
	    m_ansi_attributes[0x36] = 0x00;                          // Gap #1 number of bytes
	    m_ansi_attributes[0x37] = 0x00;                          // Gap #1 pattern

	    m_ansi_attributes[0x40] = 0x00;                          // Encoding Method #2
	    m_ansi_attributes[0x41] = 0x00;                          // Preamble #2 number of bytes
	    m_ansi_attributes[0x42] = 0x00;                          // Preamble #2 pattern
	    m_ansi_attributes[0x43] = 0x00;                          // Sync #2 pattern
	    m_ansi_attributes[0x44] = 0x00;                          // Postamble #2 number of bytes
	    m_ansi_attributes[0x45] = 0x00;                          // Postamble #2 pattern
	    m_ansi_attributes[0x46] = 0x00;                          // Gap #2 number of bytes
	    m_ansi_attributes[0x47] = 0x00;                          // Gap #2 pattern
    }
    return m_ansi_attributes[m_attribute_number];
}

void ansi_disk_device::load_attribute(uint8_t attribute_value)
{
    if (m_attribute_number > 0x47) {
        SLOG1(("  + illegal attribute number %d", m_attribute_number))
        return;
    }
    m_ansi_attributes[m_attribute_number] = attribute_value;
}

void ansi_disk_device::assert_read_gate()
{
	m_sector_timer->enable(false);

	int sector = m_pulsed_sector;
    int cylinder = (m_current_cylinder_high << 8) | m_current_cylinder_low;
    int track = cylinder * m_heads + m_selected_head;
    int track_offset = track * m_sectors;
    int sector_offset = track_offset + sector;

    SLOG1(("DN300_DISK:    CMD_READ_RECORD for sector %d on cylinder %d and head %d", sector, cylinder, m_selected_head));
    SLOG1(("DN300_DISK:    linearized as logical sector address %d", sector));

    if (!m_image) {
        SLOG1(("%p: disk image is null?", this));
		return;
    }

    // read from the image synchronously, but report the bytes back to the controller
    // on a timer.
    m_image->fseek(sector_offset * HARD_DISK_SECTOR_SIZE, SEEK_SET);
    m_image->fread(m_buffer, HARD_DISK_SECTOR_SIZE);

    m_cursor = 0;
    m_read_timer->adjust(attotime::from_usec(1), 0, attotime::from_usec(1));
}

TIMER_CALLBACK_MEMBER(ansi_disk_device::read_sector_next_byte)
{
    if (m_cursor > HARD_DISK_SECTOR_SIZE) {
        SLOG1(("DN300_DISK:    done with read and the controller didn't finish the op"));
        m_read_timer->adjust(attotime::never);
        return;
    }

    cur_read_data_cb(this, m_buffer[m_cursor++]);
}

void ansi_disk_device::deassert_read_gate()
{
    SLOG1(("DN300_DISK:  read finished"));
    m_read_timer->reset();
	m_sector_timer->enable();
}

void ansi_disk_device::assert_write_gate()
{
    if (!m_image) {
        SLOG1(("%p: disk image is null?", this));
		return;
    }

    // write to the buffer until we've accumulated everything, then do a single write
    // to the image.
    m_cursor = 0;
}

void ansi_disk_device::write_sector_next_byte(uint8_t data)
{
    if (m_cursor > HARD_DISK_SECTOR_SIZE) {
        SLOG1(("DN300_DISK:    done with write and the controller didn't finish the op"));
        return;
    }

    m_buffer[m_cursor++] = data;
}

void ansi_disk_device::deassert_write_gate()
{
	int sector = m_pulsed_sector;
    int cylinder = (m_current_cylinder_high << 8) | m_current_cylinder_low;
    int track = cylinder * m_heads + m_selected_head;
    int track_offset = track * m_sectors;
    int sector_offset = track_offset + sector;

    SLOG1(("DN300_DISK:    finish_write_sector for sector %d on cylinder %d and head %d", sector, cylinder, m_selected_head));
    SLOG1(("DN300_DISK:    linearized as logical sector address %d", sector));

    m_image->fseek(sector_offset * HARD_DISK_SECTOR_SIZE, SEEK_SET);
    m_image->fwrite(m_buffer, HARD_DISK_SECTOR_SIZE);

    SLOG1(("DN300_DISK:  write finished"));
}

void ansi_disk_device::select()
{
	SLOG1(("DN300_DISK:    select"));
	m_sector_timer->adjust(m_sector_clock_freq, 0, m_sector_clock_freq);
}

void ansi_disk_device::deselect()
{
	m_sector_timer->reset();
}

// excepts both the command and a parameter out byte.  returns what should be the parameter in byte.
// if the command doesn't require a parameter in byte, return m_general_status.
uint8_t ansi_disk_device::execute_command(uint8_t command, uint8_t parameter)
{
    switch (command) {
        case ANSI_CMD_REPORT_ILLEGAL_COMMAND:
			SLOG1(("DN300_DISK:    ansicmd REPORT_ILLEGAL_COMMAND"));
            // This command shall force the Illegal Command Bit to be set in the
            // General Status Byte (see Section 4.4). The General Status Byte,
            // with the Illegal Command Bit equal to one, is returned to the host
            // by the Parameter Byte of the command sequence.
            m_general_status |= GS_ILLEGAL_COMMAND;
            return m_general_status;

        case ANSI_CMD_CLEAR_FAULT:
			SLOG1(("DN300_DISK:    ansicmd CLEAR_FAULT"));
            // This command shall cause all fault status bits of the selected
            // device to be reset, provided the fault condition has passed. If
            // the fault condition persits the appropriate status bit shall
            // continue to be equal to one. The General Status Byte, cleared of
            // previous fault status, shall be returned by the Parameter Byte of
            // the command sequence.
            // The Clear Fault Command shall also reset the Attention Condition
            // caused by the fault condition, again only if the fault condition no
            // longer exists.

            m_general_status &= ~(
                GS_CONTROL_BUS_ERROR |
                GS_ILLEGAL_COMMAND |
                GS_ILLEGAL_PARAMETER
            );

            clear_sb1(SB1_SEEK_ERROR | SB1_RW_FAULT | SB1_POWER_FAULT | SB1_COMMAND_REJECT);

            set_attention_line(false);

            return m_general_status;

        case ANSI_CMD_CLEAR_ATTENTION:
			SLOG1(("DN300_DISK:    ansicmd CLEAR_ATTENTION"));
            // This command shall cause the Attention Condition to be reset in the
            // selected device. The General Status Byte shall be returned by the
            // Parameter Byte of the command sequence.
            //
            // If the error or other condition that caused the Attention Condition
            // persists, the Attention Condition shall not be set again. If,
            // however, the condition is reset and the error reoccurs, the
            // Attention Condition shall be set again.
            m_sense_byte_2 &= ~(
                SB2_INITIAL_STATE |
                SB2_READY_TRANSITION |
                SB2_DEVICE_ATTR_TABLE_MODIFIED
            );

            set_attention_line(false);

            return m_general_status;

        case ANSI_CMD_SEEK:
			SLOG1(("DN300_DISK:    ansicmd SEEK"));
            // This command shall cause the selected device to seek to the
            // cylinder identified as the target cylinder by the Load Cylinder
            // Address Commands (see Sections 4.1.3 and 4.1.4). The General
            // Status Byte shall be returned to the host by the Parameter Byte of
            // the command sequence with the Busy Executing bit set (see Section
            // 4.4.1.7).
            // The Seek Command shall set the Attention Condition and the Illegal
            // Parameter Bit in the General Status Byte if the target cylinder
            // address is outside the cylinder address range of the device.
            // Upon completion of any seek (including a zero length seek) the
            // device shall clear the Busy Executing bit in the General Status
            // Byte and set the Attention Condition.

			m_current_cylinder_high = m_load_cylinder_high;
			m_current_cylinder_low = m_load_cylinder_low;

            start_time_dependent_command(attotime::from_msec(5)); // look up this timing...
            return m_general_status;

        case ANSI_CMD_REZERO:
			SLOG1(("DN300_DISK:    ansicmd REZERO"));
            // This command shall cause the selected device to position the moving
            // head(s) over cylinder zero. The General Status byte shall be
            // returned to the host by the Parameter Byte of the command sequence
            // with the Busy Executing bit set (see Section 4.4.1.7).
            //
            // Upon the completion of the positioning of the moving head(s) over
            // cylinder zero the device shall clear the Busy Executing bit in the
            // General Status byte and set the Attention Condition.
	        m_current_cylinder_high = 0;
	        m_current_cylinder_low = 0;

            start_time_dependent_command(attotime::from_msec(5)); // look up this timing...
            return m_general_status;

        case ANSI_CMD_REPORT_SENSE_BYTE_2:
			SLOG1(("DN300_DISK:    ansicmd REPORT_SENSE_BYTE_2"));
            // The command shall cause the selected device to return Sense Byte 2
            // by the Parameter Byte of the command sequence. No other action
            // shall be taken in the device.
            return m_sense_byte_2;

        case ANSI_CMD_REPORT_SENSE_BYTE_1:
			SLOG1(("DN300_DISK:    ansicmd REPORT_SENSE_BYTE_1"));
            // This command shall cause the selected device to return Sense Byte 1
            // by the Parameter Byte of the command sequence. No other action
            // shall be taken in the device.
            return m_sense_byte_1;

        case ANSI_CMD_REPORT_GENERAL_STATUS:
			SLOG1(("DN300_DISK:    ansicmd REPORT_GENERAL_STATUS"));
            // This command shall cause the selected device to return the general
            // Status Byte by the Parameter Byte of the command sequence. This
            // command shall not perform any other function in the device and acts
            // as a "no-op" in order to allow the host to monitor the device's
            // General Status Byte without changing any device condition.
            return m_general_status;

        case ANSI_CMD_REPORT_ATTRIBUTE:
			SLOG1(("DN300_DISK:    ansicmd REPORT_ATTRIBUTE"));
            // This command shall cause the selected device to return a byte of
            // information that is the Device Attribute whose number was defined
            // in the Load Attribute Number Command (see Section 4.1.6). The
            // contents of the byte is defined by Table 4-3 and Section 4.3.
            SLOG1(("ANSI_CMD_REPORT_ATTRIBUTE attribute=%02x", m_attribute_number));
            return report_attribute();

        case ANSI_CMD_SET_ATTENTION:
			SLOG1(("DN300_DISK:    ansicmd SET_ATTENTION"));
            // This command shall cause the selected device to set the Attention
            // Condition. No other action shall be caused.
            // The General Status Byte shall be transferred to the host by the
            // Parameter Byte of the command sequence.

            start_time_dependent_command(attotime::from_msec(5)); // look up this timing...
            return m_general_status; // XXX is this supposed to be returned with the BUSY_EXECUTING bit or not?

        case ANSI_CMD_SELECTIVE_RESET:
			SLOG1(("DN300_DISK:    ansicmd SELECTIVE_RESET"));
            // This command shall cause the selected device to reach Initial State
            // (see Section 3.2.1). This is a time dependent command and as such
            // shall set the Busy Executing bit prior to the assertion of the
            // acknowledge to parameter request and shall be reflected in the
            // returned General Status Byte. Upon completion of.the parameter
            // byte transfer the device shall go to the initial state and all
            // resetable parameter. attentions. errors. etc •• shall be reset.
            // When the initial state is reached bit 0 of Sense Byte 2 will be set
            // and bit 6 of the General Status Byte shall be cleared. (This
            // causes the setting of the Attention Condition).
            //
            // TODO
            SLOG1(("ANSI_CMD_SELECTIVE_RESET unimplemented"))
            start_time_dependent_command(attotime::from_msec(5)); // look up this timing...
            return parameter;

        case ANSI_CMD_REFORMAT_TRACK:
			SLOG1(("DN300_DISK:    ansicmd REFORMAT_TRACK"));
            // This command shall cause the selected device to reconfigure the
            // arrangement of Sector Pulse generation according to parameters
            // received via the Load Sector Pulses Per Track Commands (see
            // Sections 4.1.9 to 4.1.11). The General Status Byte shall be
            // returned to the host by the Parameter Byte of the command
            // sequence.
            // The Partition Track Command is a Time Dependent Command and as such
            // shall set the Busy Executing bit in the General Status Byte
            // returned by this command (see Section 4.4.1.7) and it is to remain
            // set while this command execution is in process. Also. the device
            // shall exercise appropriate control over the Busy signal at the
            // interface (see Section 3.2.7).
            // Upon the completion of execution of this command the Bytes Per
            // Sector and the Sector Per Track will be updated in the Attribute
            // Table and also bit 6 of Attribute byte OE Hex will be cleared and
            // this shall set the Attention Condition.
            // The Partition Track Command shall set the Attention Condition and
            // the Illegal Parameter Bit in the General Status Byte if the Sector
            // Pulses Per Track create a set that is outside the range of the
            // device.
            // activating Read Gate or Write Gate while this command is executing
            // is a violation of protocol.
            //
            // TODO
            SLOG1(("ANSI_CMD_REFORMAT_TRACK unimplemented"))
            start_time_dependent_command(attotime::from_msec(5)); // look up this timing...
            return m_general_status; // XXX is this supposed to be returned with the BUSY_EXECUTING bit or not?

        case ANSI_CMD_REPORT_CYL_ADDR_HIGH:
			SLOG1(("DN300_DISK:    ansicmd REPORT_CYL_ADDR_HIGH"));
            // This command shall cause the selected device to return a byte of
            // information that is the most significant byte of a 16 bit .number
            // that, indicates the cylinder address of the current position of the
            // moving heads. This number shall not reflect the most recent
            // cylinder address set by the Set Cylinder Address Commands (see
            // Sections 4.1.3 and 4.1.4) ,unless there has been an intervening Seek
            // Command completed (see Section 4.2.4).
            // If executed during a seek operation, the information returned shall
            // be ascertained by the vendor specification.
            // The information shall be transferred by the Parameter Byte of the
            // command sequence.
            SLOG1(("ANSI_CMD_REPORT_CYL_ADDR_HIGH %02x", m_current_cylinder_high));
			return m_current_cylinder_high;

        case ANSI_CMD_REPORT_CYL_ADDR_LOW:
			SLOG1(("DN300_DISK:    ansicmd REPORT_CYL_ADDR_LOW"));
            // This command shall cause the selected device to return a byte of
            // information that is the least significant byte of a 16 bit number
            // that indicates the cylinder address of the current position of the
            // moving heads. This number shall not reflect the most recent
            // cylinder address set by the Set Cylinder Address Commands (see
            // Sections 4.1.3 and 4.1.4) unless there has been an intervening Seek
            // Command completed (see Section 4.2.4).
            // If executed during a seek operation, the information returned shall
            // be ascertained by the vendor specification.
            // The information shall be transferred by the Parameter Byte of the
            // command sequence.
            //
            SLOG1(("ANSI_CMD_REPORT_CYL_ADDR_LOW %02x", m_current_cylinder_low));
            return m_current_cylinder_low;

        case ANSI_CMD_REPORT_TEST_BYTE:
			SLOG1(("DN300_DISK:    ansicmd REPORT_TEST_BYTE"));
            // This command shall cause the selected device to return a copy of
            // the Test Byte transferred to the device via the Load 'Test Byte
            // Command. (See Section 4.1.12.)
            // The Test Byte shall be transferred by the Parameter Byte of the
            // command sequence.
            //
            // TODO
            SLOG1(("ANSI_CMD_REPORT_TEST_BYTE %02x", m_test_byte));
			return m_test_byte;

        case ANSI_CMD_ATTENTION_CONTROL:
			SLOG1(("DN300_DISK:    ansicmd ATTENTION_CONTROL"));
            // This command shall condition the selected device to enable or
            // disable its attention circuitry based on the value of the Parameter
            // Byte as shown below.
            // 7 6 5 4 3 2 1 0
            // | o o o o o o o
            // |
            // o - Enable Attention
            // 1 - Disable Attention
            //
            // This command allows the host to selectively ignore attention
            // requests from certain devices on the interface. This might be done
            // in response to a device that generates spurious attention requests
            // due to a malfuntion.
            // The Enable Attention Command shall cause the selected device to
            // gate its internal Attention Condition onto the party line ("wired
            // OR") Attention Signal. The Disable Attention Command shall cause
            // the selected device to disable the gating of the internal Attention
            // Condition onto the party line Attention Signal. This command shall
            // have no impact on the function of the radial status returned with
            // the Attention In Strobe Signal (see Signal 3.2.3.2).
            // Devices shall be initilized with the Attention circuitry enabled.
            //
            m_attention_enabled = (parameter & 0x80) ? false : true;
            return parameter;

        case ANSI_CMD_WRITE_CONTROL:
			SLOG1(("DN300_DISK:    ansicmd WRITE_CONTROL"));
            // This command shall condition the selected device to enable or
            // disable its write circuitry based on the value of the parameter
            // Byte as shown below:
            // 7 6 5 4 3 2 1 0
            // | o o o o o o o
            // |
            // 1 - Write Enable
            // o - Write Disable
            //
            // This command is used in conjunction with the Write Gate Signal and
            // therefore merely enables the write circuitry while the Write Gate
            // Signal activates the circuitry at the proper time. An active Write
            // Gate Signal while the device's write circuitry is disabled shall
            // result in no data being recorded.
            // Devices shall be initialized with the write circuitry disabled.
            // A Write Control Command execute during a write operation is a
            // violation of protocol.
            //
            m_write_enabled = (parameter & 0x80) != 0;
            return parameter;

        case ANSI_CMD_LOAD_CYL_ADDR_HIGH:
			SLOG1(("DN300_DISK:    ansicmd LOAD_CYL_ADDR_HIGH"));
            // This command shall condition the selected device to accept the
            // Parameter Byte as the most significant Byte of a cylinder address.
            // This command is used in conjunction with the Seek Command (see
            // Section 4.2.4) and therefore is a means of supplying the most
            // significant byte of a target cylinder address.
            // This command shall not cause any head motion. Loading a cylinder
            // address outside the range of a device shall not cause an error
            // unless a subsequent Seek Command is issued to that illegal
            // cylinder.
            // Devices shall be initialized with the target cylinder address equal
            // to zero.
            //
            SLOG1(("ANSI_CMD_LOAD_CYL_ADDR_HIGH %02x", parameter));
			m_load_cylinder_high = parameter;
            return parameter;

        case ANSI_CMD_LOAD_CYL_ADDR_LOW:
			SLOG1(("DN300_DISK:    ansicmd LOAD_CYL_ADDR_LOW"));
            // This command shall condition the selected device to accept the
            // Parameter Byte as the least signficant byte of a cylinder address.
            // This command is used in conjunction with the Seek Command (see
            // Section 4.2.4) and therefore is a means of supplying the least
            // significant byte of a target cylinder address.
            // This command shall not cause any head motion. Loading a cylinder
            // address outside the range of a device shall not cause an error
            // unless a subsequent seek command is issued to that illegal
            // cylinder.
            // Devices shall be initialized with the target cylinder address equal
            // to zero.
            //
            SLOG1(("ANSI_CMD_LOAD_CYL_ADDR_LOW %02x", parameter));
			m_load_cylinder_low = parameter;
            return parameter;

        case ANSI_CMD_SELECT_HEAD:
			SLOG1(("DN300_DISK:    ansicmd SELECT_HEAD"));
            // This command shall condition the selected device to. accept the
            // Parameter Byte as the binary address of the head selected for read
            // or write operations. This command shall enable the moving heads
            // and shall disable the fixed heads.
            // A Select Moving Head Command issued during a read or write
            // operation is a violation of protocol.
            // The device shall set the Attention Condition and the Illegal
            // Parameter Bit in the General Status Byte upon receipt of a head
            // address outside the head address range of the device.
            // Devices shall be initialized with moving head zero selected.
            m_selected_head = parameter;
            return parameter;

        case ANSI_CMD_LOAD_ATTRIBUTE_NUMBER:
			SLOG1(("DN300_DISK:    ansicmd LOAD_ATTRIBUTE_NUMBER"));
            // This command shall condition the selected device to accept the
            // Parameter Byte as the number of a Device Attribute as defined in
            // Table 4-3. This command prepares the device for a subsequent Load
            // Device Attribute Command or Report Device Attribute Command (see
            // Sections 4.1.7 and 4.2.9). This command may be issued at any time.
			m_attribute_number = parameter;
            return parameter;

        case ANSI_CMD_LOAD_ATTRIBUTE:
			SLOG1(("DN300_DISK:    ansicmd LOAD_ATTRIBUTE"));
            // This command shall condition the selected device to accept the
            // Parameter Byte as the new value of a Device Attribute. The number
            // of the Device Attribute must have been previously defined by the
            // Load Attribute Number Command (see Section 4.1.6).
            SLOG1(("ANSI_CMD_LOAD_ATTRIBUTE attribute=%02x, value=%02x", m_attribute_number, parameter))
            load_attribute(parameter);
			SLOG1(("  + done"));
            return parameter;

        case ANSI_CMD_SPIN_CONTROL:
			SLOG1(("DN300_DISK:    ansicmd SPIN_CONTROL"));
            // This command shall condition the seleted device to enter a spin up
            // or spin down cycle based on the value of the Parameter Byte as
            // shown below.
            // 7 6 5 4 3 2 1 0
            // | o o o o o o o
            // |
            // 1 - Spin Up
            // o - Spin Down
            // A spin up cycle shall consist of starting the rotation of the
            // spindle. A spin down cycle shall consist of stopping the rotation
            // of the spindle.
            // Upon completion of a spin control cycle the device shall set the
            // Attention Condition. Issuing a spin up command to a device whose
            // spindle is already at full speed or issuing a spin down command to
            // a device whose spindle has already stopped shall also set the
            // Attention Condition.
            // The Spin Control Command is a Time Dependent Command and as such
            // shall set the Busy Executing bit in the General Status Byte (see
            // Section 4.4.1.7) while command execution is in process. Also, the
            // device shall exercise appropriate control over the Busy signal at
            // the interface (see Section 3.2.7).
            // A spin down cycle shall cause the repositioning of the moving
            // head(s) over the landing zone and stop the rotation of the
            // spindle. If the device detects that it cannot successfully seek to
            // the landing zone it shall set the Attention Condition and set bit 0
            // of Sense Byte 1.
            // See vendor specification for initial state of the Spin Control.

            start_time_dependent_command(attotime::from_msec(10));
            return parameter;

        case ANSI_CMD_LOAD_SECT_PER_TRACK_HIGH:
            SLOG1(("DN300_DISK:    ansicmd LOAD_SECT_PER_TRACK_HIGH unimplemented"))
            return parameter;

        case ANSI_CMD_LOAD_SECT_PER_TRACK_MEDIUM:
		    SLOG1(("DN300_DISK:    ansicmd LOAD_SECT_PER_TRACK_MEDIUM unimplemented"))
            return parameter;

        case ANSI_CMD_LOAD_SECT_PER_TRACK_LOW:
		    SLOG1(("DN300_DISK:    ansicmd LOAD_SECT_PER_TRACK_LOW unimplemented"))
            return parameter;

        case ANSI_CMD_LOAD_BYTES_PER_SECT_HIGH:
		    SLOG1(("DN300_DISK:    ansicmd LOAD_BYTES_PER_SECT_HIGH unimplemented"))
            return parameter;

        case ANSI_CMD_LOAD_BYTES_PER_SECT_MEDIUM:
		    SLOG1(("DN300_DISK:    ansicmd LOAD_BYTES_PER_SECT_MEDIUM unimplemented"))
            return parameter;

        case ANSI_CMD_LOAD_BYTES_PER_SECT_LOW:
		    SLOG1(("DN300_DISK:    ansicmd LOAD_BYTES_PER_SECT_LOW unimplemented"))
            return parameter;

        case ANSI_CMD_LOAD_READ_PERMIT_HIGH:
		    SLOG1(("DN300_DISK:    ansicmd LOAD_READ_PERMIT_HIGH unimplemented"))
            return parameter;

        case ANSI_CMD_LOAD_READ_PERMIT_LOW:
		    SLOG1(("DN300_DISK:    ansicmd LOAD_READ_PERMIT_LOW unimplemented"))
            return parameter;

        case ANSI_CMD_LOAD_WRITE_PERMIT_HIGH:
		    SLOG1(("DN300_DISK:    ansicmd LOAD_WRITE_PERMIT_HIGH unimplemented"))
            return parameter;

        case ANSI_CMD_LOAD_WRITE_PERMIT_LOW:
		    SLOG1(("DN300_DISK:    ansicmd LOAD_WRITE_PERMIT_LOW unimplemented"))
            return parameter;

        case ANSI_CMD_LOAD_TEST_BYTE:
		    SLOG1(("DN300_DISK:    ansicmd LOAD_TEST_BYTE %02x", parameter));
			m_test_byte = parameter;
			return m_general_status;

        default:
            SLOG1(("unknown ANSI command %02x", command));
            return parameter;
    }
}

void ansi_disk_device::set_sb1(uint8_t value) {
    if ((m_sense_byte_1 & value) == 0) {
        m_sense_byte_1 |= value;
        // all sb1 bits set attention on 0->1 transition
        set_attention_line(true);
    }
}

void ansi_disk_device::clear_sb1(uint8_t value) {
    m_sense_byte_1 &= ~value;
}

void ansi_disk_device::set_sb2(uint8_t value) {
    if ((m_sense_byte_2 & value) == 0) {
        m_sense_byte_2 |= value;
        // only certain sb2 bits set attention on 0->1 transition
        if (value | (
            SB2_INITIAL_STATE |
            SB2_READY_TRANSITION |
            SB2_FORCED_RELEASE |
            SB2_DEVICE_ATTR_TABLE_MODIFIED |
            SB2_VENDOR_ATTNS
        )) {
            set_attention_line(true);
        }
    }
}

void ansi_disk_device::clear_sb2(uint8_t value) {
    m_sense_byte_2 &= ~value;
}

void ansi_disk_device::start_time_dependent_command(attotime duration) {
    m_general_status |= GS_BUSY_EXECUTING;
    m_time_dependent_timer->adjust(duration, 0);
}

void ansi_disk_device::set_attention_line(bool state) {
	if (m_attention != state) {
	    m_attention = state;
	    cur_attention_cb(this, state);
	}
}

TIMER_CALLBACK_MEMBER(ansi_disk_device::finish_time_dependent_command) {
    SLOG1(("ansi_disk_device(%p)::finish_time_dependent_command", this));
    m_general_status |= GS_NORMAL_COMPLETE;
    if (m_attention_enabled) {
        set_attention_line(true);
    }
}

TIMER_CALLBACK_MEMBER(ansi_disk_device::sector_callback) {
	m_pulsed_sector = (m_pulsed_sector+1) % m_sectors;
	if (m_pulsed_sector == 0) {
		cur_index_pulse_cb(this);
	} else {
		cur_sector_pulse_cb(this);
	}
}
