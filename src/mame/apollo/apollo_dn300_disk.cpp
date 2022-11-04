
#include "emu.h"

#define VERBOSE 2
#include "apollo_dn300.h"

DEFINE_DEVICE_TYPE(APOLLO_DN300_DISK, apollo_dn300_disk_device, "apollo_dn300_disk", "Apollo DN300 Winchester/Floppy controller")

apollo_dn300_disk_device::apollo_dn300_disk_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock):
    device_t(mconfig, APOLLO_DN300_DISK, tag, owner, clock),
    m_cpu(*this, "cpu"),
    m_physical_space(*this, "physical_space"),
	m_ansi_cmd(0),
	m_ansi_parm(0),
	m_sector(0),
	m_cylinder_high(0),
	m_cylinder_low(0),
	m_head(0),
	m_interrupt_control(0),
	m_controller_command(0),
	m_status_high(0),
	m_status_low(0),
	m_attention_status(0),
	m_write_enabled(false),
	m_selected_head(0),
	m_selected_drive(0)
{
}

void apollo_dn300_disk_device::device_start()
{
}

void apollo_dn300_disk_device::device_reset()
{
}

#define REG_ANSI_CMD            0x00
#define REG_ANSI_PARM           0x02
#define REG_SECTOR              0x06
#define REG_CYLINDER_HIGH       0x08
#define REG_CYLINDER_LOW        0x09
#define REG_HEAD                0x0a
#define REG_INTERRUPT_CONTROL   0x0c
#define REG_CONTROLLER_COMMAND  0x0e

#define REG_ATTENTION_STATUS    0x00
#define REG_DRIVE_NUM_OF_STATUS 0x02
#define REG_STATUS_HIGH         0x06
#define REG_STATUS_LOW          0x07

// Controller commands
#define CMD_NOOP              0x00
#define CMD_READ_RECORD       0x01
#define CMD_WRITE_RECORD      0x02
#define CMD_FORMAT_TRACK      0x03
#define CMD_SEEK              0x04
#define CMD_EXEC_ANSI_CMD     0x05
#define CMD_EXEC_DRIVE_SELECT 0x06
#define CMD_EXEC_ATTENTION    0x07
#define CMD_SELECT_HEAD       0x08

// ANSI commands
#define ANSI_CMD_REPORT_ILLEGAL_COMMAND     0x00
#define ANSI_CMD_CLEAR_FAULT                0x01
#define ANSI_CMD_CLEAN_ATTENTION            0x02
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

void apollo_dn300_disk_device::write(offs_t offset, uint8_t data, uint8_t mem_mask)
{
	switch (offset) {
		case REG_ANSI_CMD:
			m_ansi_cmd = data;
			SLOG1(("DN300_DISK: ANSI_COMMAND = %02x", data));
			break;
		case REG_ANSI_PARM:
			m_ansi_parm = data;
			SLOG1(("DN300_DISK: ANSI_PARM = %02x", data));
			break;
		case REG_SECTOR:
			m_sector = data;
			SLOG1(("DN300_DISK: SECTOR = %02x", data));
			break;
		case REG_CYLINDER_HIGH:
			m_cylinder_high = data;
			SLOG1(("DN300_DISK: CYLINDER_HIGH = %02x", data));
			break;
		case REG_CYLINDER_LOW:
			m_cylinder_low = data;
			SLOG1(("DN300_DISK: CYLINDER_LOW = %02x", data));
			break;
		case REG_HEAD:
			m_head = data;
			SLOG1(("DN300_DISK: HEAD = %02x", data));
			break;
		case REG_INTERRUPT_CONTROL:
			m_interrupt_control = data;
			SLOG1(("DN300_DISK: INTERRUPT_CONTROL = %02x", data));
			break;
		case REG_CONTROLLER_COMMAND:
			m_controller_command = data;
			SLOG1(("DN300_DISK: CONTROLLER_COMMAND = %02x", data));
			execute_command();
			break;
		default:
			SLOG1(("DN300_DISK: unknown write to offset %02x = %02x & %08x", offset, data, mem_mask));
			break;
	}
}

uint8_t apollo_dn300_disk_device::read(offs_t offset, uint8_t mem_mask)
{
	switch (offset) {
		case REG_ATTENTION_STATUS:
			SLOG1(("DN300_DISK: ATTENTION_STATUS = %02x", m_attention_status));
			// reading this clears some status bits
			m_status_high &= ~0x10;
			return m_attention_status;
		case REG_ANSI_PARM:
			SLOG1(("DN300_DISK: ANSI_PARM = %02x", m_ansi_parm));
			return m_ansi_parm;
		case REG_STATUS_HIGH:
			SLOG1(("DN300_DISK: STATUS_HIGH = %02x & %02x", m_status_high, mem_mask));
			return m_status_high;
		case REG_STATUS_LOW:
			SLOG1(("DN300_DISK: STATUS_LOW = %02x", m_status_low));
			return m_status_low;
		default:
			SLOG1(("DN300_DISK: unknown read at offset %02x & %08x", offset, mem_mask));
			return 0;
		}
}


void apollo_dn300_disk_device::execute_command()
{
	// clear the status bits that clear on command
	m_status_high &= ~0x80;
	m_status_low &= ~0xfa;

	switch (m_controller_command) {
		case CMD_NOOP:
			break;

		case CMD_READ_RECORD:
			break;

		case CMD_WRITE_RECORD:
			break;

		case CMD_FORMAT_TRACK:
			break;

		case CMD_SEEK:
			break;

		case CMD_EXEC_ANSI_CMD:
			execute_ansi_command();
			break;

		case CMD_EXEC_DRIVE_SELECT:
			m_selected_drive = m_ansi_parm;
			break;

		case CMD_EXEC_ATTENTION:
			break;

		case CMD_SELECT_HEAD:
			m_selected_head = m_head;
			break;
		default:
			SLOG1(("unknown controller command %02x", m_controller_command));
			break;
	}
}

void apollo_dn300_disk_device::execute_ansi_command()
{
	switch (m_ansi_cmd) {
		case ANSI_CMD_REZERO:
			// no actual in parm here - we set attention "when it's finished."
			m_attention_status |= 0x80;
			// if (m_interrupt_control & 0x01) {
				m_status_high |= 0x10;
			// }
			break;
		case ANSI_CMD_WRITE_CONTROL:
			m_write_enabled = (m_ansi_parm & 0x80) != 0;
			break;
		default:
			SLOG1(("unknown ANSI command %02x", m_ansi_cmd));
			break;
	}
}
