
// see http://bitsavers.org/pdf/priam/Priam_Device_Level_Interface_Jun83.pdf
// and http://www.bitsavers.org/pdf/apollo/002398-04_Domain_Engineering_Handbook_Rev4_Jan87.pdf

// the drive controller looks to be entirely custom for the hard drive (probably
// a thin layer over it, with the driver doing most of the work).
//
// for the floppy drive, however, it uses a standard chip - NEC765ac

#include "emu.h"

#define VERBOSE 2
#include "apollo_dn300.h"

static uint8_t hopefully_this_is_right[0x48] = {
	/* 0x00 */ 0x00, // User ID - user defined
	/* 0x01 */ 0x00, // Model ID High - vendor defined

	/* 0x02 */ 0x03, // Model ID Low - vendor defined
	// DISK checks for model id 0x03, 0x04, 0x05, 0x35

	/* 0x03 */ 0x00, // Revision ID - vendor defined
	/* 0x04-0x0C */
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00,
	/* 0x0D */ 0x00, // Device Type ID - device dependent
	/* 0x0E */ 0x00, // Table Modification - action dependent
	/* 0x0F */ 0x00, // Table ID - vendor defined

	/* 0x10 */ 0x00, // MSB of # of bytes per track
	/* 0x11 */ 0x00, // MedSB of # of bytes per track
	/* 0x12 */ 0x00, // LSB of # of bytes per track
	/* 0x13 */ 0x00, // MSB of # of bytes per sector
	/* 0x14 */ 0x00, // MedSB of # of bytes per sector
	/* 0x15 */ 0x00, // LSB of # of bytes per sector
	/* 0x16 */ 0x00, // MSB of # of sector pulses per track
	/* 0x17 */ 0x00, // MedSB of # of sector pulses per track
	/* 0x18 */ 0x00, // LSB of # of sector pulses per track
	/* 0x19 */ 0x00, // Sectoring method
	/* 0x1a-0x1f */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

	/* 0x20 */ 0x00, // MSB of # of cylinders
	/* 0x21 */ 0x00, // LSB of # of cylinders
	/* 0x22 */ 0x00, // Number of heads
	/* 0x23-0x2f */
	0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00,

	/* 0x30 */ 0x00, // Encoding method #1
	/* 0x31 */ 0x00, // Preamble #1 number of bytes
	/* 0x32 */ 0x00, // Preamble #1 pattern
	/* 0x33 */ 0x00, // Sync #1 pattern
	/* 0x34 */ 0x00, // Postamble #1 number of bytes
	/* 0x35 */ 0x00, // Postamble #1 pattern
	/* 0x36 */ 0x00, // Gap #1 number of bytes
	/* 0x37 */ 0x00, // Gap #1 pattern
	/* 0x38-0x3f */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

	/* 0x40 */ 0x00, // Encoding Method #2
	/* 0x41 */ 0x00, // Preamble #2 number of bytes
	/* 0x42 */ 0x00, // Preamble #2 pattern
	/* 0x43 */ 0x00, // Sync #2 pattern
	/* 0x44 */ 0x00, // Postamble #2 number of bytes
	/* 0x45 */ 0x00, // Postamble #2 pattern
	/* 0x46 */ 0x00, // Gap #2 number of bytes
	/* 0x47 */ 0x00, // Gap #2 pattern
};

DEFINE_DEVICE_TYPE(APOLLO_DN300_DISK, apollo_dn300_disk_device, APOLLO_DN300_DISK_TAG, "Apollo DN300 Winchester/Floppy controller")

apollo_dn300_disk_device::apollo_dn300_disk_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock):
    device_t(mconfig, APOLLO_DN300_DISK, tag, owner, clock),
    drq_cb(*this),
    m_cpu(*this, MAINCPU),
	m_fdc(*this, APOLLO_DN300_FLOPPY_TAG),
    m_wdc_ansi_cmd(0),
    m_wdc_ansi_parm(0),
    m_wdc_ansi_attribute_number(0),
	m_wdc_ansi_attributes(NULL),
	m_wdc_ansi_test_byte(0),
    m_wdc_sector(0),
    m_wdc_current_cylinder_high(0),
    m_wdc_current_cylinder_low(0),
    m_wdc_load_cylinder_high(0),
    m_wdc_load_cylinder_low(0),
    m_wdc_head(0),
    m_wdc_interrupt_control(0),
    m_controller_command(0),
    m_wdc_status_high(0),
    m_wdc_status_low(0),
    m_wdc_selected_head(0),
    m_wdc_selected_drive(0),
    m_wdc_general_status(0),
    m_wdc_sense_byte_1(0),
    m_wdc_sense_byte_2(0),
    m_wdc_write_enabled(false),
    m_wdc_attention_enabled(true),
	m_disk_fp(NULL),
	m_sysboot_fp(NULL)
{
    char* DISK_IMAGE = getenv("DISK_IMAGE");
    char* SYSBOOT_OVERRIDE_IMAGE = getenv("SYSBOOT_OVERRIDE_IMAGE");

    if (DISK_IMAGE) {
        m_disk_fp = fopen(DISK_IMAGE, "r");
    }
    if (SYSBOOT_OVERRIDE_IMAGE) {
        m_sysboot_fp = fopen(SYSBOOT_OVERRIDE_IMAGE, "r");
    }

    if (m_disk_fp == NULL) {
        logerror("couldn't open disk image\n");
    }
    if (m_sysboot_fp == NULL) {
        logerror("couldn't open sysboot\n");
    }

	m_wdc_ansi_attributes = (uint8_t*)malloc(0x48);
	memmove(m_wdc_ansi_attributes, hopefully_this_is_right, 0x48);
}

void apollo_dn300_disk_device::device_start()
{
    drq_cb.resolve();

    // save_item(NAME(drq));
}

void apollo_dn300_disk_device::device_reset()
{
}

// winchester registers
#define WDC_REG_ANSI_CMD            0x00
#define WDC_REG_ANSI_PARM           0x02
#define WDC_REG_SECTOR              0x06
#define WDC_REG_CYLINDER_HIGH       0x08
#define WDC_REG_CYLINDER_LOW        0x09
#define WDC_REG_HEAD                0x0a
#define WDC_REG_INTERRUPT_CONTROL   0x0c
#define WDC_REG_CONTROLLER_COMMAND  0x0e

#define WDC_REG_ATTENTION_STATUS    0x00
#define WDC_REG_DRIVE_NUM_OF_STATUS 0x02
#define WDC_REG_STATUS_HIGH         0x06
#define WDC_REG_STATUS_LOW          0x07

// floppy registers
#define FDC_REG_FLOPPY_STATUS       0x10
#define FDC_REG_FLOPPY_DATA    		0x12
#define FDC_REG_FLOPPY_CONTROL 		0x14

#define WDC_IRQCTRL_ENABLE_END_OF_OP    0x01
#define WDC_IRQCTRL_ENABLE_STATUS_AVAIL 0x02
#define WDC_IRQCTRL_ENABLE_ATTENTION    0x04
#define WDC_IRQCTRL_ENABLE_OVERALL      0x08

// Controller commands
#define WDC_CONTROLLER_CMD_NOOP              0x00
#define WDC_CONTROLLER_CMD_READ_RECORD       0x01
#define WDC_CONTROLLER_CMD_WRITE_RECORD      0x02
#define WDC_CONTROLLER_CMD_FORMAT_TRACK      0x03
#define WDC_CONTROLLER_CMD_SEEK              0x04
#define WDC_CONTROLLER_CMD_EXEC_ANSI_CMD     0x05
#define WDC_CONTROLLER_CMD_EXEC_DRIVE_SELECT 0x06
#define WDC_CONTROLLER_CMD_EXEC_ATTENTION    0x07
#define WDC_CONTROLLER_CMD_SELECT_HEAD       0x08

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
#define WDC_GS_NOT_READY         0x01
#define WDC_GS_CONTROL_BUS_ERROR 0x02
#define WDC_GS_ILLEGAL_COMMAND   0x04
#define WDC_GS_ILLEGAL_PARAMETER 0x08
#define WDC_GS_SENSE_BYTE_1      0x10
#define WDC_GS_SENSE_BYTE_2      0x20
#define WDC_GS_BUSY_EXECUTING    0x40
#define WDC_GS_NORMAL_COMPLETE   0x80

// sense byte 1 bits
#define WDC_SB1_SEEK_ERROR     0x01
#define WDC_SB1_RW_FAULT       0x02
#define WDC_SB1_POWER_FAULT    0x04
// nothing on bit 3
#define WDC_SB1_SPEED_ERROR    0x10
#define WDC_SB1_COMMAND_REJECT 0x20

// sense byte 2 bits
#define WDC_SB2_INITIAL_STATE    0x01
#define WDC_SB2_READY_TRANSITION 0x02
// nothing on bit 2-4
#define WDC_SB2_DEVICE_ATTR_TABLE_MODIFIED 0x20
#define WDC_SB2_POSITIONED_WITHIN_WRITE_PROTECTED_AREA 0x40

// controller status word
// high byte:
#define CONTROLLER_STATUS_HIGH_CONTROLLER_BUSY             0x80
#define CONTROLLER_STATUS_HIGH_DRIVE_BUSY                  0x40
#define CONTROLLER_STATUS_HIGH_DRIVE_ATTENTION             0x20
#define CONTROLLER_STATUS_HIGH_STATUS_AVAILABLE_INTERRUPT  0x10
#define CONTROLLER_STATUS_HIGH_END_OF_OP_INTERRUPT         0x08
#define CONTROLLER_STATUS_HIGH_FLOPPY_INTERRUPTING         0x04
// low byte:
#define CONTROLLER_STATUS_LOW_TIMEOUT                      0x80
#define CONTROLLER_STATUS_LOW_OVERRUN                      0x40
#define CONTROLLER_STATUS_LOW_CRC_ERROR                    0x20
#define CONTROLLER_STATUS_LOW_BUS_PARITY_ERROR             0x10
#define CONTROLLER_STATUS_LOW_ILLEGAL_CONFIG               0x08
#define CONTROLLER_STATUS_LOW_STATUS_TIMEOUT               0x04
#define CONTROLLER_STATUS_LOW_DMA_PARITY_ERROR             0x02

void
apollo_dn300_disk_device::map(address_map &map)
{
	map(0x00, 0x0F).rw(FUNC(apollo_dn300_disk_device::wdc_read), FUNC(apollo_dn300_disk_device::wdc_write));
	map(0x10, 0x11).r(FUNC(apollo_dn300_disk_device::fdc_msr_r));
	map(0x12, 0x13).rw(FUNC(apollo_dn300_disk_device::fdc_fifo_r), FUNC(apollo_dn300_disk_device::fdc_fifo_w));
	map(0x14, 0x15).w(FUNC(apollo_dn300_disk_device::fdc_dsr_w));

	map(0x20, 0x21).w(FUNC(apollo_dn300_disk_device::calendar_ctrl_w));
	map(0x22,0x23).w(FUNC(apollo_dn300_disk_device::calendar_data_w));
	map(0x24,0x25).r(FUNC(apollo_dn300_disk_device::calendar_data_r));
	// calendar stuff goes here eventually
}


void
apollo_dn300_disk_device::calendar_ctrl_w(offs_t, uint8_t data, uint8_t mem_mask)
{
	COMBINE_DATA(&m_calendar_ctrl);
}

void
apollo_dn300_disk_device::calendar_data_w(offs_t, uint8_t data, uint8_t mem_mask)
{
	COMBINE_DATA(&m_calendar_data);
}

uint8_t
apollo_dn300_disk_device::calendar_data_r(offs_t, uint8_t mem_mask)
{
	return m_calendar_data & mem_mask;
}

void
apollo_dn300_disk_device::fdc_dsr_w(offs_t, uint8_t data, uint8_t mem_mask)
{
    SLOG1(("DN300_DISK: fdc_dsr_w = %02x & %02x", data, mem_mask));
	m_fdc->dsr_w(data&mem_mask);
}

uint8_t
apollo_dn300_disk_device::fdc_msr_r(offs_t offset, uint8_t mem_mask)
{
	uint8_t data = m_fdc->msr_r() & mem_mask;
    SLOG1(("DN300_DISK: fdc_msr_r = %02x", data));
	return data;
}

void
apollo_dn300_disk_device::fdc_fifo_w(offs_t, uint8_t data, uint8_t mem_mask)
{
    SLOG1(("DN300_DISK: fdc_fifo_w = %02x & %02x", data, mem_mask));
	m_fdc->fifo_w(data&mem_mask);
}

uint8_t
apollo_dn300_disk_device::fdc_fifo_r(offs_t, uint8_t mem_mask)
{
	uint8_t data = m_fdc->fifo_r() & mem_mask;
	SLOG1(("DN300_DISK: fdc_fifo_r = %02x", data));
	return data;
}


void apollo_dn300_disk_device::wdc_write(offs_t offset, uint8_t data, uint8_t mem_mask)
{
    switch (offset) {
        // winchester register writes
        case WDC_REG_ANSI_CMD:
            SLOG1(("DN300_DISK: wdc ANSI_COMMAND write = %02x", data));
            m_wdc_ansi_cmd = data;
            break;
        case WDC_REG_ANSI_PARM:
            SLOG1(("DN300_DISK: wdc ANSI_PARM write = %02x", data));
            m_wdc_ansi_parm = data;
            break;
        case WDC_REG_SECTOR:
            SLOG1(("DN300_DISK: wdc SECTOR write = %02x", data));
            m_wdc_sector = data;
            break;
        case WDC_REG_CYLINDER_HIGH:
            SLOG1(("DN300_DISK: wdc CYLINDER_HIGH write = %02x", data));
            m_wdc_load_cylinder_high = data;
            break;
        case WDC_REG_CYLINDER_LOW:
            SLOG1(("DN300_DISK: wdc CYLINDER_LOW write = %02x", data));
            m_wdc_load_cylinder_low = data;
            break;
        case WDC_REG_HEAD:
            SLOG1(("DN300_DISK: HEAD write = %02x", data));
            m_wdc_head = data;
            break;
        case WDC_REG_INTERRUPT_CONTROL:
            SLOG1(("DN300_DISK: wdc INTERRUPT_CONTROL write = %02x", data));
            m_wdc_interrupt_control = data;
            break;
        case WDC_REG_CONTROLLER_COMMAND:
            SLOG1(("DN300_DISK: wdc CONTROLLER_COMMAND write = %02x", data));
            m_controller_command = data;
            execute_command();
            break;

        default:
            SLOG1(("DN300_DISK: unknown wdc write to offset %02x = %02x & %08x", offset, data, mem_mask));
            break;
    }
}

uint8_t apollo_dn300_disk_device::wdc_read(offs_t offset, uint8_t mem_mask)
{
    switch (offset) {
        // winchester register reads
        case WDC_REG_ATTENTION_STATUS:
            SLOG1(("DN300_DISK: wdc ATTENTION_STATUS read = %02x", m_wdc_general_status));
            // reading this clears some status bits
            m_wdc_status_high &= ~(
				CONTROLLER_STATUS_HIGH_STATUS_AVAILABLE_INTERRUPT |
				CONTROLLER_STATUS_HIGH_DRIVE_ATTENTION
			);
            return m_wdc_general_status;
        case WDC_REG_ANSI_PARM:
            SLOG1(("DN300_DISK: wdc ANSI_PARM read = %02x", m_wdc_ansi_parm));
            return m_wdc_ansi_parm;
        case WDC_REG_STATUS_HIGH:
            SLOG1(("DN300_DISK: wdc STATUS_HIGH read = %02x & %02x", m_wdc_status_high, mem_mask));
            return m_wdc_status_high;
        case WDC_REG_STATUS_LOW:
            SLOG1(("DN300_DISK: wdc STATUS_LOW read = %02x", m_wdc_status_low));
            return m_wdc_status_low;

        default:
            SLOG1(("DN300_DISK: unknown wdc read at offset %02x & %08x", offset, mem_mask));
            return 0;
    }
}


void apollo_dn300_disk_device::execute_command()
{
    // clear the status bits that clear on command
    m_wdc_status_high &= ~0x08;
    m_wdc_status_low &= ~0xfa;

    SLOG1(("execute_command: %02x", m_controller_command))
    switch (m_controller_command) {
        case WDC_CONTROLLER_CMD_NOOP:
            break;

        case WDC_CONTROLLER_CMD_READ_RECORD: {
            int cylinder = (m_wdc_current_cylinder_high << 8) | m_wdc_current_cylinder_low;
            // 15/18 here match the values in the dn3500 disk image.
            // 15 = the drive's number of heads (and == TracksPerCylinder)
            int track = cylinder * 15 + m_wdc_head;
            // 18 = SectorsPerTrack
            int sector_offset = track * 18;
            int sector = sector_offset + m_wdc_sector;
            SLOG1(("CMD_READ_RECORD for sector %d on cylinder %d and head %d", m_wdc_sector, cylinder, m_wdc_head));
            SLOG1(("linearized as logical sector address %d\n", sector));
            SLOG1(("reading header from disk image"));
            fseek(m_disk_fp, 1056 * sector, SEEK_SET);
            fread(m_read_buffer, 32, 1, m_disk_fp);
            if (track == 0 && (m_wdc_sector >= 2 && m_wdc_sector <= 11 && m_sysboot_fp)) {
                SLOG1(("reading data from sysboot override"));
                fseek(m_sysboot_fp, 1024 * (m_wdc_sector-2), SEEK_SET);
                memset(&m_read_buffer[32], 0, 1024); // just in case we get a short read.
                fread(&m_read_buffer[32], 1024, 1, m_sysboot_fp);
            } else {
                SLOG1(("reading data from disk image"));
                fread(&m_read_buffer[32], 1024, 1, m_disk_fp);
            }
            m_read_cursor = 0;

#define PULSE_DRQ() do { drq_cb(true); drq_cb(false); } while (0)
            // 0x10 for the first operation
            for (int i = 0; i < 0x10; i++) {
                PULSE_DRQ();
            }

            // 0x200 for the second operation
            for (int i = 0; i < 0x200; i++) {
                PULSE_DRQ();
            }

			if (m_wdc_interrupt_control & WDC_IRQCTRL_ENABLE_END_OF_OP) {
	            m_wdc_status_high |= CONTROLLER_STATUS_HIGH_END_OF_OP_INTERRUPT;
				// m_cpu->set_input_line(APOLLO_DN300_IRQ_DISK, ASSERT_LINE);
			}
            break;
        }
        case WDC_CONTROLLER_CMD_WRITE_RECORD:
            SLOG1(("CMD_WRITE_RECORD unimplemented"))
            break;

        case WDC_CONTROLLER_CMD_FORMAT_TRACK:
            SLOG1(("CMD_FORMAT_TRACK unimplemented"))
            break;

        case WDC_CONTROLLER_CMD_SEEK:
            SLOG1(("CMD_SEEK to cylinder %02x%02x", m_wdc_load_cylinder_high, m_wdc_load_cylinder_low))
            // Guessing this is equivalent to the ansi seek command?

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

            m_wdc_general_status |= WDC_GS_BUSY_EXECUTING;

            // if this were an async emulator we'd return here,
            // but instead we jump immediately to steps performed after the
            // operation is done:
			m_wdc_current_cylinder_high = m_wdc_load_cylinder_high;
			m_wdc_current_cylinder_low = m_wdc_load_cylinder_low;

            m_wdc_general_status &= ~WDC_GS_BUSY_EXECUTING;
            if (m_wdc_attention_enabled) {
                m_wdc_status_high |= CONTROLLER_STATUS_HIGH_DRIVE_ATTENTION;
				SLOG1(("HELLO2 %02x", m_wdc_interrupt_control));
				// if (m_wdc_interrupt_control & WDC_IRQCTRL_ENABLE_OVERALL) {
					m_wdc_status_high |= CONTROLLER_STATUS_HIGH_DRIVE_ATTENTION;
					// m_wdc_general_status |= 0xb;
					// status available?
					if (m_wdc_interrupt_control & WDC_IRQCTRL_ENABLE_STATUS_AVAIL) {
						m_wdc_status_high |= CONTROLLER_STATUS_HIGH_STATUS_AVAILABLE_INTERRUPT;
						m_wdc_status_high &= ~CONTROLLER_STATUS_HIGH_DRIVE_ATTENTION;
			            m_wdc_general_status |= WDC_GS_NORMAL_COMPLETE;
						// m_cpu->set_input_line(APOLLO_DN300_IRQ_DISK, ASSERT_LINE);
					} else if (m_wdc_interrupt_control & WDC_IRQCTRL_ENABLE_END_OF_OP) {
						m_wdc_status_high |= CONTROLLER_STATUS_HIGH_END_OF_OP_INTERRUPT;
						// m_cpu->set_input_line(APOLLO_DN300_IRQ_DISK, ASSERT_LINE);
					}
					SLOG1(("set_input_line"));
					// m_cpu->set_input_line(APOLLO_DN300_IRQ_DISK, ASSERT_LINE);
				// }
            }

            break;

        case WDC_CONTROLLER_CMD_EXEC_ANSI_CMD:
            execute_ansi_command();
            break;

        case WDC_CONTROLLER_CMD_EXEC_DRIVE_SELECT:
            m_wdc_selected_drive = m_wdc_ansi_parm;
            break;

        case WDC_CONTROLLER_CMD_EXEC_ATTENTION:
            SLOG1(("CMD_EXEC_ATTENTION unimplemented"))
            break;

        case WDC_CONTROLLER_CMD_SELECT_HEAD:
            // Guessing this is equivalent to the ansi select head command?

            // This command shall condition the selected device to. accept the
            // Parameter Byte as the binary address of the head selected for read
            // or write operations. This command shall enable the moving heads
            // and shall disable the fixed heads.
            // A Select Moving Head Command issued during a read or write
            // operation is a violation of protocol.
            // The device shall set the Attention Condition and the Illegal
            // Parameter Bit in the General Status Byte upon receipt of a head
            // address out.side the head address range of the device.
            // Devices shall be initialized with moving head zero selected.

            m_wdc_general_status |= WDC_GS_BUSY_EXECUTING;

            // if this were an async emulator we'd return here,
            // but instead we jump immediately to steps performed after the
            // operation is done:
            m_wdc_selected_head = m_wdc_head;

            m_wdc_general_status &= ~WDC_GS_BUSY_EXECUTING;
            if (m_wdc_attention_enabled) {
                // m_wdc_status_high |= CONTROLLER_STATUS_HIGH_DRIVE_ATTENTION;
				SLOG1(("HELLO1 %02x", m_wdc_interrupt_control));
				if ((m_wdc_interrupt_control & WDC_IRQCTRL_ENABLE_OVERALL) &&
				    (m_wdc_interrupt_control & WDC_IRQCTRL_ENABLE_ATTENTION)) {
						SLOG1(("set_input_line"));
					m_cpu->set_input_line(APOLLO_DN300_IRQ_DISK, ASSERT_LINE);
				}
            }

            break;

        default:
            SLOG1(("unknown controller command %02x", m_controller_command));
            break;
    }
}

void apollo_dn300_disk_device::execute_ansi_command()
{
    switch (m_wdc_ansi_cmd) {
        case ANSI_CMD_REPORT_ILLEGAL_COMMAND:
            // This command shall force the Illegal Command Bit to be set in the
            // General Status Byte (see Section 4.4). The General Status Byte,
            // with the Illegal Command Bit equal to one, is returned to the host
            // by the Parameter Byte of the command sequence.
            m_wdc_general_status |= WDC_GS_ILLEGAL_COMMAND;
            m_wdc_ansi_parm = m_wdc_general_status;
            break;

        case ANSI_CMD_CLEAR_FAULT:
            // This command shall cause all fault status bits of the selected
            // device to be reset, provided the fault condition has passed. If
            // the fault condition persits the appropriate status bit shall
            // continue to be equal to one. The General Status Byte, cleared of
            // previous fault status, shall be returned by the Parameter Byte of
            // the command sequence.
            // The Clear Fault Command shall also reset the Attention Condition
            // caused by the fault condition, again only if the fault condition no
            // longer exists.

            m_wdc_general_status &= ~(
                WDC_GS_CONTROL_BUS_ERROR |
                WDC_GS_ILLEGAL_COMMAND |
                WDC_GS_ILLEGAL_PARAMETER
            );
            m_wdc_sense_byte_1 &= ~(
                WDC_SB1_SEEK_ERROR |
                WDC_SB1_RW_FAULT |
                WDC_SB1_POWER_FAULT |
                WDC_SB1_COMMAND_REJECT
            );

            m_wdc_ansi_parm = m_wdc_general_status;
            break;

        case ANSI_CMD_CLEAR_ATTENTION:
            // This command shall cause the Attention Condition to be reset in the
            // selected device. The General Status Byte shall be returned by the
            // Parameter Byte of the command sequence.
            //
            // If the error or other condition that caused the Attention Condition
            // persists, the Attention Condition shall not be set again. If,
            // however, the condition is reset and the error reoccurs, the
            // Attention Condition shall be set again.

            m_wdc_general_status &= ~WDC_GS_NORMAL_COMPLETE;
            m_wdc_sense_byte_2 &= ~(
                WDC_SB2_INITIAL_STATE |
                WDC_SB2_READY_TRANSITION |
                WDC_SB2_DEVICE_ATTR_TABLE_MODIFIED
            );

            m_wdc_ansi_parm = m_wdc_general_status;
            break;

        case ANSI_CMD_SEEK:
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
            SLOG1(("ANSI_CMD_SEEK unimplemented"))
            break;

        case ANSI_CMD_REZERO:
            // This command shall cause the selected device to position the moving
            // head(s) over cylinder zero. The General Status byte shall be
            // returned to the host by the Parameter Byte of the command sequence
            // with the Busy Executing bit set (see Section 4.4.1.7).
            //
            // Upon the completion of the positioning of the moving head(s) over
            // cylinder zero the device shall clear the Busy Executing bit in the
            // General Status byte and set the Attention Condition.

            m_wdc_general_status |= WDC_GS_BUSY_EXECUTING;

            // if this were an async emulator we'd return here:
            //
            // m_ansi_param = m_general_status;
            // return;
            //
            // but instead we jump immediately to steps performed after the
            // operation is done:
			m_wdc_current_cylinder_high = 0;
			m_wdc_current_cylinder_low = 0;

            m_wdc_general_status &= ~WDC_GS_BUSY_EXECUTING;
            if (m_wdc_attention_enabled) {
                m_wdc_status_high |= CONTROLLER_STATUS_HIGH_DRIVE_ATTENTION;
            }

            m_wdc_ansi_parm = m_wdc_general_status;
            break;

        case ANSI_CMD_REPORT_SENSE_BYTE_2:
            // The command shall cause the selected device to return Sense Byte 2
            // by the Parameter Byte of the command sequence. No other action
            // shall be taken in the device.
            m_wdc_ansi_parm = m_wdc_sense_byte_2;
            break;
        case ANSI_CMD_REPORT_SENSE_BYTE_1:
            // This command shall cause the selected device to return Sense Byte 1
            // by the Parameter Byte of the command sequence. No other action
            // shall be taken in the device.
            m_wdc_ansi_parm = m_wdc_sense_byte_1;
            break;
        case ANSI_CMD_REPORT_GENERAL_STATUS:
            // This command shall cause the selected device to return the general
            // Status Byte by the Parameter Byte of the command sequence. This
            // command shall not perform any other function in the device and acts
            // as a "no-op" in order to allow the host to monitor the device's
            // General Status Byte without changing any device condition.
            m_wdc_ansi_parm = m_wdc_general_status;
            break;

        case ANSI_CMD_REPORT_ATTRIBUTE:
            // This command shall cause the selected device to return a byte of
            // information that is the Device Attribute whose number was defined
            // in the Load Attribute Number Command (see Section 4.1.6). The
            // contents of the byte is defined by Table 4-3 and Section 4.3.
            SLOG1(("ANSI_CMD_REPORT_ATTRIBUTE attribute=%02x", m_wdc_ansi_attribute_number));
			m_wdc_ansi_parm = m_wdc_ansi_attributes[m_wdc_ansi_attribute_number];
            break;

        case ANSI_CMD_SET_ATTENTION:
            // This command shall cause the selected device to set the Attention
            // Condition. No other action shall be caused.
            // The General Status Byte shall be transferred to the host by the
            // Parameter Byte of the command sequence.

            m_wdc_general_status |= WDC_GS_NORMAL_COMPLETE; // is this right?
            m_wdc_status_high |= CONTROLLER_STATUS_HIGH_DRIVE_ATTENTION;

            m_wdc_ansi_parm = m_wdc_general_status;
            break;

        case ANSI_CMD_SELECTIVE_RESET:
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
            break;

        case ANSI_CMD_REFORMAT_TRACK:
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
            break;

        case ANSI_CMD_REPORT_CYL_ADDR_HIGH:
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
            SLOG1(("ANSI_CMD_REPORT_CYL_ADDR_HIGH %02x", m_wdc_current_cylinder_high));
			m_wdc_ansi_parm = m_wdc_current_cylinder_high;
            break;

        case ANSI_CMD_REPORT_CYL_ADDR_LOW:
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
            SLOG1(("ANSI_CMD_REPORT_CYL_ADDR_LOW %02x", m_wdc_current_cylinder_low));
			m_wdc_ansi_parm = m_wdc_current_cylinder_low;
            break;

        case ANSI_CMD_REPORT_TEST_BYTE:
            // This command shall cause the selected device to return a copy of
            // the Test Byte transferred to the device via the Load 'Test Byte
            // Command. (See Section 4.1.12.)
            // The Test Byte shall be transferred by the Parameter Byte of the
            // command sequence.
            //
            // TODO
            SLOG1(("ANSI_CMD_REPORT_TEST_BYTE %02x", m_wdc_ansi_test_byte));
			m_wdc_ansi_parm = m_wdc_ansi_test_byte;
            break;

        case ANSI_CMD_ATTENTION_CONTROL:
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
            m_wdc_attention_enabled = (m_wdc_ansi_parm & 0x80) ? false : true;
            break;

        case ANSI_CMD_WRITE_CONTROL:
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
            m_wdc_write_enabled = (m_wdc_ansi_parm & 0x80) != 0;
            break;

        case ANSI_CMD_LOAD_CYL_ADDR_HIGH:
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
            SLOG1(("ANSI_CMD_LOAD_CYL_ADDR_HIGH %02x", m_wdc_ansi_parm));
			m_wdc_load_cylinder_high = m_wdc_ansi_parm;
            break;

        case ANSI_CMD_LOAD_CYL_ADDR_LOW:
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
            SLOG1(("ANSI_CMD_LOAD_CYL_ADDR_LOW %02x", m_wdc_ansi_parm));
			m_wdc_load_cylinder_low = m_wdc_ansi_parm;
            break;

        case ANSI_CMD_SELECT_HEAD:
            // This command shall condition the selected device to. accept the
            // Parameter Byte as the binary address of the head selected for read
            // or write operations. This command shall enable the moving heads
            // and shall disable the fixed heads.
            // A Select Moving Head Command issued during a read or write
            // operation is a violation of protocol.
            // The device shall set the Attention Condition and the Illegal
            // Parameter Bit in the General Status Byte upon receipt of a head
            // address out.side the head address range of the device.
            // Devices shall be initialized with moving head zero selected.
            SLOG1(("ANSI_CMD_SELECT_HEAD unimplemented"))
            break;

        case ANSI_CMD_LOAD_ATTRIBUTE_NUMBER:
            // This command shall condition the selected device to accept the
            // Parameter Byte as the number of a Device Attribute as defined in
            // Table 4-3. This command prepares the device for a subsequent Load
            // Device Attribute Command or Report Device Attribute Command (see
            // Sections 4.1.7 and 4.2.9). This command may be issued at any time.
			m_wdc_ansi_attribute_number = m_wdc_ansi_parm;
            break;

        case ANSI_CMD_LOAD_ATTRIBUTE:
            // This command shall condition the selected device to accept the
            // Parameter Byte as the new value of a Device Attribute. The number
            // of the Device Attribute must have been previously defined by the
            // Load Attribute Number Command (see Section 4.1.6).
            SLOG1(("ANSI_CMD_LOAD_ATTRIBUTE attribute=%02x, value=%02x", m_wdc_ansi_attribute_number, m_wdc_ansi_parm))
			if (m_wdc_ansi_attribute_number > 0x47) {
				SLOG1(("  + illegal attribute number"))
			}
			m_wdc_ansi_attributes[m_wdc_ansi_attribute_number] = m_wdc_ansi_parm;
			SLOG1(("  + done"));
            break;

        case ANSI_CMD_SPIN_CONTROL:
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
            // head(s) over the landing sone and stop the rotation of the
            // spindle. If the device detects that it cannot successfully seek to
            // the landing zone it shall set the Attention Condition and set bit 0
            // of Sense Byte 1.
            // See vendor specification for initial state of the Spin Control.

            m_wdc_general_status |= WDC_GS_BUSY_EXECUTING;

            // if this were an async emulator we'd return here:
            //
            // m_ansi_param = m_general_status;
            // return;
            //
            // but instead we jump immediately to steps performed after the
            // operation is done:

            m_wdc_general_status &= ~WDC_GS_BUSY_EXECUTING;
            if (m_wdc_attention_enabled) {
                m_wdc_general_status |= WDC_GS_NORMAL_COMPLETE;
                m_wdc_status_high |= CONTROLLER_STATUS_HIGH_DRIVE_ATTENTION;
            }

            m_wdc_ansi_parm = m_wdc_general_status;
            break;
        case ANSI_CMD_LOAD_SECT_PER_TRACK_HIGH:
            SLOG1(("ANSI_CMD_LOAD_SECT_PER_TRACK_HIGH unimplemented"))
            break;
        case ANSI_CMD_LOAD_SECT_PER_TRACK_MEDIUM:
		    SLOG1(("ANSI_CMD_LOAD_SECT_PER_TRACK_MEDIUM unimplemented"))
            break;
        case ANSI_CMD_LOAD_SECT_PER_TRACK_LOW:
		    SLOG1(("ANSI_CMD_LOAD_SECT_PER_TRACK_LOW unimplemented"))
            break;
        case ANSI_CMD_LOAD_BYTES_PER_SECT_HIGH:
		    SLOG1(("ANSI_CMD_LOAD_BYTES_PER_SECT_HIGH unimplemented"))
            break;
        case ANSI_CMD_LOAD_BYTES_PER_SECT_MEDIUM:
		    SLOG1(("ANSI_CMD_LOAD_BYTES_PER_SECT_MEDIUM unimplemented"))
            break;
        case ANSI_CMD_LOAD_BYTES_PER_SECT_LOW:
		    SLOG1(("ANSI_CMD_LOAD_BYTES_PER_SECT_LOW unimplemented"))
            break;
        case ANSI_CMD_LOAD_READ_PERMIT_HIGH:
		    SLOG1(("ANSI_CMD_LOAD_READ_PERMIT_HIGH unimplemented"))
            break;
        case ANSI_CMD_LOAD_READ_PERMIT_LOW:
		    SLOG1(("ANSI_CMD_LOAD_READ_PERMIT_LOW unimplemented"))
            break;
        case ANSI_CMD_LOAD_WRITE_PERMIT_HIGH:
		    SLOG1(("ANSI_CMD_LOAD_WRITE_PERMIT_HIGH unimplemented"))
            break;
        case ANSI_CMD_LOAD_WRITE_PERMIT_LOW:
		    SLOG1(("ANSI_CMD_LOAD_WRITE_PERMIT_LOW unimplemented"))
            break;
        case ANSI_CMD_LOAD_TEST_BYTE:
		    SLOG1(("ANSI_CMD_LOAD_TEST_BYTE %02x", m_wdc_ansi_parm));
			m_wdc_ansi_test_byte = m_wdc_ansi_parm;
            break;
        default:
            SLOG1(("unknown ANSI command %02x", m_wdc_ansi_cmd));
            break;
    }
}

uint8_t apollo_dn300_disk_device::read_byte(offs_t _unused_offset)
{
    // SLOG1(("reading disk DMA from offset %02x -> %02x", m_read_cursor, _unused_offset));
    // send back the byte pointed to by the read cursor
    return m_read_buffer[m_read_cursor++];
}

void apollo_dn300_disk_device::write_byte(offs_t offset, uint8_t data)
{
    SLOG1(("writing disk DMA at offset %02x = %02x", offset, data));
}

