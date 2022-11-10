
// see http://bitsavers.org/pdf/priam/Priam_Device_Level_Interface_Jun83.pdf
// and http://www.bitsavers.org/pdf/apollo/002398-04_Domain_Engineering_Handbook_Rev4_Jan87.pdf

#include "emu.h"

#define VERBOSE 2
#include "apollo_dn300.h"

DEFINE_DEVICE_TYPE(APOLLO_DN300_DISK, apollo_dn300_disk_device, "apollo_dn300_disk", "Apollo DN300 Winchester/Floppy controller")

apollo_dn300_disk_device::apollo_dn300_disk_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock):
    device_t(mconfig, APOLLO_DN300_DISK, tag, owner, clock),
	drq_cb(*this),
    m_cpu(*this, "cpu"),
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
	m_selected_head(0),
	m_selected_drive(0),
	m_general_status(0),
	m_sense_byte_1(0),
	m_sense_byte_2(0),
	m_write_enabled(false),
	m_attention_enabled(true)
{
}

void apollo_dn300_disk_device::device_start()
{
	drq_cb.resolve();

	// save_item(NAME(drq));
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
#define SB1_SEEK_ERROR     0x01
#define SB1_RW_FAULT       0x02
#define SB1_POWER_FAULT    0x04
// nothing on bit 3
#define SB1_SPEED_ERROR    0x10
#define SB1_COMMAND_REJECT 0x20

// sense byte 2 bits
#define SB2_INITIAL_STATE    0x01
#define SB2_READY_TRANSITION 0x02
// nothing on bit 2-4
#define SB2_DEVICE_ATTR_TABLE_MODIFIED 0x20
#define SB2_POSITIONED_WITHIN_WRITE_PROTECTED_AREA 0x40

// controller status word
// high byte:
#define STATUS_HIGH_CONTROLLER_BUSY             0x80
#define STATUS_HIGH_DRIVE_BUSY                  0x40
#define STATUS_HIGH_DRIVE_ATTENTION             0x20
#define STATUS_HIGH_STATUS_AVAILABLE_INTERRUPT  0x10
#define STATUS_HIGH_END_OF_OP_INTERRUPT         0x08
#define STATUS_HIGH_FLOPPY_INTERRUPTING         0x04
// low byte:
#define STATUS_LOW_TIMEOUT                      0x80
#define STATUS_LOW_OVERRUN                      0x40
#define STATUS_LOW_CRC_ERROR                    0x20
#define STATUS_LOW_BUS_PARITY_ERROR             0x10
#define STATUS_LOW_ILLEGAL_CONFIG               0x08
#define STATUS_LOW_STATUS_TIMEOUT               0x04
#define STATUS_LOW_DMA_PARITY_ERROR             0x02

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
			SLOG1(("DN300_DISK: ATTENTION_STATUS = %02x", m_general_status));
			// reading this clears some status bits
			m_status_high &= ~STATUS_HIGH_DRIVE_ATTENTION;
			return m_general_status;
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
			SLOG1(("CMD_READ_RECORD unimplemented"))
#define PULSE_DRQ() do { drq_cb(true); drq_cb(false); } while (0)
			// 16 for the first operation
			PULSE_DRQ();
			PULSE_DRQ();
			PULSE_DRQ();
			PULSE_DRQ();
			PULSE_DRQ();
			PULSE_DRQ();
			PULSE_DRQ();
			PULSE_DRQ();
			PULSE_DRQ();
			PULSE_DRQ();
			PULSE_DRQ();
			PULSE_DRQ();
			PULSE_DRQ();
			PULSE_DRQ();
			PULSE_DRQ();
			PULSE_DRQ();
			break;

		case CMD_WRITE_RECORD:
			SLOG1(("CMD_WRITE_RECORD unimplemented"))
			break;

		case CMD_FORMAT_TRACK:
			SLOG1(("CMD_FORMAT_TRACK unimplemented"))
			break;

		case CMD_SEEK:
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

			m_general_status |= GS_BUSY_EXECUTING;

			// if this were an async emulator we'd return here,
			// but instead we jump immediately to steps performed after the
			// operation is done:

			m_general_status &= ~GS_BUSY_EXECUTING;
			if (m_attention_enabled) {
				m_status_high |= STATUS_HIGH_DRIVE_ATTENTION;
			}

			break;

		case CMD_EXEC_ANSI_CMD:
			execute_ansi_command();
			break;

		case CMD_EXEC_DRIVE_SELECT:
			m_selected_drive = m_ansi_parm;
			break;

		case CMD_EXEC_ATTENTION:
			SLOG1(("CMD_EXEC_ATTENTION unimplemented"))
			break;

		case CMD_SELECT_HEAD:
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

			m_general_status |= GS_BUSY_EXECUTING;

			// if this were an async emulator we'd return here,
			// but instead we jump immediately to steps performed after the
			// operation is done:
			m_selected_head = m_head;

			m_general_status &= ~GS_BUSY_EXECUTING;
			if (m_attention_enabled) {
				m_status_high |= STATUS_HIGH_DRIVE_ATTENTION;
			}

			break;

		default:
			SLOG1(("unknown controller command %02x", m_controller_command));
			break;
	}
}

void apollo_dn300_disk_device::execute_ansi_command()
{
	switch (m_ansi_cmd) {
		case ANSI_CMD_REPORT_ILLEGAL_COMMAND:
			// This command shall force the Illegal Command Bit to be set in the
	        // General Status Byte (see Section 4.4). The General Status Byte,
	        // with the Illegal Command Bit equal to one, is returned to the host
	        // by the Parameter Byte of the command sequence.
			m_general_status |= GS_ILLEGAL_COMMAND;
			m_ansi_parm = m_general_status;
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

			m_general_status &= ~(
				GS_CONTROL_BUS_ERROR |
				GS_ILLEGAL_COMMAND |
				GS_ILLEGAL_PARAMETER
			);
			m_sense_byte_1 &= ~(
				SB1_SEEK_ERROR |
				SB1_RW_FAULT |
				SB1_POWER_FAULT |
				SB1_COMMAND_REJECT
			);

			m_ansi_parm = m_general_status;
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

			m_general_status &= ~GS_NORMAL_COMPLETE;
			m_sense_byte_2 &= ~(
				SB2_INITIAL_STATE |
				SB2_READY_TRANSITION |
				SB2_DEVICE_ATTR_TABLE_MODIFIED
			);

			m_ansi_parm = m_general_status;
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

			m_general_status |= GS_BUSY_EXECUTING;

			// if this were an async emulator we'd return here:
			//
			// m_ansi_param = m_general_status;
			// return;
			//
			// but instead we jump immediately to steps performed after the
			// operation is done:

			m_general_status &= ~GS_BUSY_EXECUTING;
			if (m_attention_enabled) {
				m_status_high |= STATUS_HIGH_DRIVE_ATTENTION;
			}

			m_ansi_parm = m_general_status;
			break;

		case ANSI_CMD_REPORT_SENSE_BYTE_2:
			// The command shall cause the selected device to return Sense Byte 2
			// by the Parameter Byte of the command sequence. No other action
			// shall be taken in the device.
			m_ansi_parm = m_sense_byte_2;
			break;
		case ANSI_CMD_REPORT_SENSE_BYTE_1:
			// This command shall cause the selected device to return Sense Byte 1
			// by the Parameter Byte of the command sequence. No other action
			// shall be taken in the device.
			m_ansi_parm = m_sense_byte_1;
			break;
		case ANSI_CMD_REPORT_GENERAL_STATUS:
			// This command shall cause the selected device to return the general
			// Status Byte by the Parameter Byte of the command sequence. This
			// command shall not perform any other function in the device and acts
			// as a "no-op" in order to allow the host to monitor the device's
			// General Status Byte without changing any device condition.
			m_ansi_parm = m_general_status;
			break;

		case ANSI_CMD_REPORT_ATTRIBUTE:
			// This command shall cause the selected device to return a byte of
			// information that is the Device Attribute whose number was defined
			// in the Load Attribute Number Command (see Section 4.1.6). The
			// contents of the byte is defined by Table 4-3 and Section 4.3.

			// TODO
			SLOG1(("ANSI_CMD_REPORT_ATTRIBUTE unimplemented"))
			break;

		case ANSI_CMD_SET_ATTENTION:
			// This command shall cause the selected device to set the Attention
			// Condition. No other action shall be caused.
			// The General Status Byte shall be transferred to the host by the
			// Parameter Byte of the command sequence.

			m_general_status |= GS_NORMAL_COMPLETE; // is this right?
			m_status_high |= STATUS_HIGH_DRIVE_ATTENTION;

			m_ansi_parm = m_general_status;
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

			// TODO
			SLOG1(("ANSI_CMD_REPORT_CYL_ADDR_HIGH unimplemented"))
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
			// TODO
			SLOG1(("ANSI_CMD_REPORT_CYL_ADDR_LOW unimplemented"))
			break;

		case ANSI_CMD_REPORT_TEST_BYTE:
			// This command shall cause the selected device to return a copy of
			// the Test Byte transferred to the device via the Load 'Test Byte
			// Command. (See Section 4.1.12.)
			// The Test Byte shall be transferred by the Parameter Byte of the
			// command sequence.
			//
			// TODO
			SLOG1(("ANSI_CMD_REPORT_TEST_BYTE unimplemented"))
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
			m_attention_enabled = (m_ansi_parm & 0x80) ? false : true;
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
			m_write_enabled = (m_ansi_parm & 0x80) != 0;
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
			// TODO
			SLOG1(("ANSI_CMD_LOAD_CYL_ADDR_HIGH unimplemented"))
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
			// TODO
			SLOG1(("ANSI_CMD_LOAD_CYL_ADDR_LOW unimplemented"))
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
			//
			// TODO
			SLOG1(("ANSI_CMD_LOAD_ATTRIBUTE_NUMBER unimplemented"))
			break;

		case ANSI_CMD_LOAD_ATTRIBUTE:
			// This command shall condition the selected device to accept the
			// Parameter Byte as the new value of a Device Attribute. The number
			// of the Device Attribute must have been previously defined by the
			// Load Attribute Number Command (see Section 4.1.6).
			//
			// TODO
			SLOG1(("ANSI_CMD_LOAD_ATTRIBUTE unimplemented"))
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

			m_general_status |= GS_BUSY_EXECUTING;

			// if this were an async emulator we'd return here:
			//
			// m_ansi_param = m_general_status;
			// return;
			//
			// but instead we jump immediately to steps performed after the
			// operation is done:

			m_general_status &= ~GS_BUSY_EXECUTING;
			if (m_attention_enabled) {
				m_general_status |= GS_NORMAL_COMPLETE;
				m_status_high |= STATUS_HIGH_DRIVE_ATTENTION;
			}

			m_ansi_parm = m_general_status;
			break;
		default:
			SLOG1(("unknown ANSI command %02x", m_ansi_cmd));
			break;
	}
}
