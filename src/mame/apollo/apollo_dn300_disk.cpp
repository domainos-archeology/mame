
// see http://bitsavers.org/pdf/priam/Priam_Device_Level_Interface_Jun83.pdf
// and http://www.bitsavers.org/pdf/apollo/002398-04_Domain_Engineering_Handbook_Rev4_Jan87.pdf

// the drive controller looks to be entirely custom for the hard drive (probably
// a thin layer over it, with the driver doing most of the work).
//
// for the floppy drive, however, it uses a standard chip - NEC765ac

#include "emu.h"

#define VERBOSE 1
#include "apollo_dn300.h"
#include "formats/apollo_dsk.h"
#include "ansi_disk_device.h"

#define HARD_DISK_SECTOR_SIZE 1056
#define FLOPPY_DISK_SECTOR_SIZE 1024

#define UNKNOWN_SECTOR -50

#define DN300_DISK0_TAG "dn300_disk0"
#define DN300_DISK1_TAG "dn300_disk1"

#define PULSE_DRQ() do { drq_cb(true); drq_cb(false); } while (0)
#define PULSE_FLOPPY_DRQ(fdc) do { drq_cb(true); drq_cb(false); } while (0)

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
#define WDC_CONTROLLER_CMD_EXEC_ATTENTION_IN 0x07
#define WDC_CONTROLLER_CMD_SELECT_HEAD       0x08

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

static void floppies(device_slot_interface &device)
{
	device.option_add("525hd", FLOPPY_525_HD);
}

DEFINE_DEVICE_TYPE(APOLLO_DN300_DISK_CTRLR, apollo_dn300_disk_ctrlr_device, APOLLO_DN300_DISK_TAG, "Apollo DN300 Winchester/Floppy controller")

apollo_dn300_disk_ctrlr_device::apollo_dn300_disk_ctrlr_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
    : device_t(mconfig, APOLLO_DN300_DISK_CTRLR, tag, owner, clock)
	, irq_cb(*this)
    , drq_cb(*this)
	// WDC-specific
    , m_wdc_selected_drive(0)
    , m_wdc_ansi_cmd(0)
    , m_wdc_ansi_parm_out(0)
    , m_wdc_sector(0)
	, m_wdc_cylinder_hi(0)
	, m_wdc_cylinder_lo(0)
    , m_wdc_head(0)
    , m_wdc_interrupt_control(0)
    , m_controller_command(0)
    , m_wdc_attention_status(0)
	, m_wdc_ansi_parm_in(0)
    , m_wdc_drive_num_of_status(0)
    , m_controller_status_high(0)
    , m_controller_status_low(0)
    , m_bytes_read(0)
    , m_word_transfer_count(0)
	, m_pulsed_sector(UNKNOWN_SECTOR)
    , m_start_read_sector(false)
    , m_start_write_sector(false)
	// FDC-specific
	, m_fdc(*this, APOLLO_DN300_FLOPPY_TAG)
    , m_floppy(*this, APOLLO_DN300_FLOPPY_TAG":0")
	, m_floppy_drq_state(false)
    , m_fdc_control(0)
	// Calendar-specific
    , m_rtc(*this, APOLLO_DN300_RTC_TAG)
    , m_calendar_ctrl(0)
{
}

void apollo_dn300_disk_ctrlr_device::device_add_mconfig(machine_config &config)
{
    ANSI_DISK_DEVICE(config, DN300_DISK0_TAG, 0);
	ANSI_DISK_DEVICE(config, DN300_DISK1_TAG, 0);

    our_disks[0] = subdevice<ansi_disk_device>(DN300_DISK0_TAG);
    our_disks[1] = subdevice<ansi_disk_device>(DN300_DISK1_TAG);

	UPD765A(config, m_fdc, 48_MHz_XTAL, false, false);
	m_fdc->intrq_wr_callback().set(FUNC(apollo_dn300_disk_ctrlr_device::fdc_irq));
	m_fdc->drq_wr_callback().set([this](int state) {
		SLOG1(("DN300_DISK_CTRLR: floppy drq %s", state ? "asserted" : "cleared"));
		m_floppy_drq_state = state;
		drq_cb(state);
	});
	FLOPPY_CONNECTOR(config, m_floppy, floppies, "525hd", apollo_dn300_disk_ctrlr_device::floppy_formats);

    MSM5832(config, m_rtc, 16_MHz_XTAL /* not sure about this clock.  there may be dividers.. */ );
}

void apollo_dn300_disk_ctrlr_device::floppy_formats(format_registration &fr)
{
	fr.add(FLOPPY_APOLLO_FORMAT);
}

void apollo_dn300_disk_ctrlr_device::device_start()
{
	irq_cb.resolve();
    drq_cb.resolve();

    // save_item(NAME(drq));
    our_disks[0] = subdevice<ansi_disk_device>(DN300_DISK0_TAG);
    our_disks[1] = subdevice<ansi_disk_device>(DN300_DISK1_TAG);

    SLOG1(("in apollo_dn300_disk_ctrlr_device::device_start: disks = [%p, %p]", our_disks[0], our_disks[1]));
	// floppy_image_device *floppy = m_floppy->get_device();
	// if (floppy != nullptr) {
	// 	m_fdc->set_floppy(floppy);
	// }
}

void apollo_dn300_disk_ctrlr_device::ansi_disk_attention(ansi_disk_device *disk, bool state) {
	static bool handling_attn = false;
	if (handling_attn) {
		SLOG1(("DN300_DISK_CTRLR: disk%d attn %s, but we're already handling an attn", disk == our_disks[0] ? 0 : 1, state ? "asserted" : "cleared"));
		return;
	}

    int idx = disk == our_disks[0] ? 0 : 1;
    if (idx != m_wdc_selected_drive-1) {
        SLOG1(("DN300_DISK_CTRLR: disk%d attn %s, but it's not the selected drive", idx, state ? "asserted" : "cleared"));
        return;
    }
    SLOG1(("DN300_DISK_CTRLR: disk%d attn %s", idx, state ? "asserted" : "cleared"));
    SLOG1(("DN300_DISK_CTRLR:    irq control = %d", m_wdc_interrupt_control));

	// if interrupt control includes WDC_IRQCTRL_ENABLE_STATUS_AVAIL,
	// grab the general status byte from the disk and set our status
	// register.  then trigger an interrupt.
	bool need_irq = ((m_wdc_interrupt_control & WDC_IRQCTRL_ENABLE_ATTENTION) != 0);
	if (state) {
		m_controller_status_high |= CONTROLLER_STATUS_HIGH_DRIVE_ATTENTION;
	} else {
		m_controller_status_high &= ~CONTROLLER_STATUS_HIGH_DRIVE_ATTENTION;
	}


	if (m_wdc_interrupt_control & WDC_IRQCTRL_ENABLE_STATUS_AVAIL) {
		SLOG1(("DN300_DISK_CTRLR:    irqctrl enable_status set"));
		// the controller resets the attention flag in this case
		m_controller_status_high &= ~CONTROLLER_STATUS_HIGH_DRIVE_ATTENTION;
		need_irq = true;
		m_wdc_attention_status = disk->execute_command(ANSI_CMD_REPORT_GENERAL_STATUS, 0);
		m_wdc_drive_num_of_status = idx + 1;
		handling_attn = true;
		disk->execute_command(ANSI_CMD_CLEAR_ATTENTION, 0);
		handling_attn = false;
        SLOG1(("DN300_DISK_CTRLR:    attention_status = %02x", m_wdc_attention_status));
		if (state) {
			m_controller_status_high |= CONTROLLER_STATUS_HIGH_STATUS_AVAILABLE_INTERRUPT;
		} else {
			m_controller_status_high &= ~CONTROLLER_STATUS_HIGH_STATUS_AVAILABLE_INTERRUPT;
		}
	}

	if (need_irq && (m_wdc_interrupt_control & WDC_IRQCTRL_ENABLE_OVERALL)) {
		SLOG1(("calling irq_cb(%s)", state ? "true" : "false"));
        SLOG1(("  DN300_DISK_CTRLR: STATUS_HIGH = %02x", m_controller_status_high));
        SLOG1(("  DN300_DISK_CTRLR: STATUS_LOW = %02x", m_controller_status_low));

		irq_cb(state);
	}
}

void apollo_dn300_disk_ctrlr_device::ansi_disk_busy(ansi_disk_device *disk, bool state) {
	int idx = disk == our_disks[0] ? 0 : 1;
	if (idx != m_wdc_selected_drive-1) {
		SLOG1(("DN300_DISK_CTRLR: disk%d busy %s, but it's not the selected drive", idx, state ? "asserted" : "cleared"));
		return;
	}
	SLOG2(("DN300_DISK_CTRLR: disk%d busy %s", idx, state ? "asserted" : "cleared"));
	if (state) {
		m_controller_status_high |= CONTROLLER_STATUS_HIGH_DRIVE_BUSY;
	} else {
		m_controller_status_high &= ~CONTROLLER_STATUS_HIGH_DRIVE_BUSY;
	}
}

void apollo_dn300_disk_ctrlr_device::ansi_disk_read_data(ansi_disk_device *disk, uint8_t data) {
    int idx = disk == our_disks[0] ? 0 : 1;
    if (idx != m_wdc_selected_drive-1) {
        SLOG1(("DN300_DISK_CTRLR: disk%d read data: m_buffer[%d] = %02x, but it's not the selected drive", idx, m_bytes_read, data));
        return;
    }
    SLOG2(("DN300_DISK_CTRLR: disk%d read data: m_buffer[%d] = %02x", idx, m_bytes_read, data));
    m_buffer[m_bytes_read++] = data;
    if (m_bytes_read == 2) {
        m_bytes_read = 0;
		drq_cb(true);
    }
}

void apollo_dn300_disk_ctrlr_device::ansi_disk_ref_clock_tick(ansi_disk_device *disk) {
    int idx = disk == our_disks[0] ? 0 : 1;
    if (idx != m_wdc_selected_drive-1) {
        SLOG1(("DN300_DISK_CTRLR: disk%d ref clock tick, but it's not the selected drive", idx));
        return;
    }

	PULSE_DRQ();

	m_word_transfer_count++;
	if (m_word_transfer_count == HARD_DISK_SECTOR_SIZE / 2) {
		// we're done with the write.  let the cpu know
		disk->deassert_write_gate();
		end_of_controller_op();
		return;
	}
}

void apollo_dn300_disk_ctrlr_device::end_of_controller_op() {
    m_controller_status_high &= ~CONTROLLER_STATUS_HIGH_CONTROLLER_BUSY;
    if (m_wdc_interrupt_control & WDC_IRQCTRL_ENABLE_END_OF_OP) {
        SLOG1(("DN300_DISK_CTRLR:    irqctrl end of op set"));
        m_controller_status_high |= CONTROLLER_STATUS_HIGH_END_OF_OP_INTERRUPT;
        irq_cb(ASSERT_LINE);
    }
}

void apollo_dn300_disk_ctrlr_device::device_reset()
{
	m_fdc->set_floppy(m_floppy->get_device());

    our_disks[0] = subdevice<ansi_disk_device>(DN300_DISK0_TAG);
    our_disks[1] = subdevice<ansi_disk_device>(DN300_DISK1_TAG);

    SLOG1(("in apollo_dn300_disk_ctrlr_device::device_reset: disks = [%p, %p]", our_disks[0], our_disks[1]));

    our_disks[0]->set_attention_cb(ansi_disk_device::bool_cb(&apollo_dn300_disk_ctrlr_device::ansi_disk_attention, this));
    our_disks[0]->set_busy_cb(ansi_disk_device::bool_cb(&apollo_dn300_disk_ctrlr_device::ansi_disk_busy, this));
    our_disks[0]->set_read_data_cb(ansi_disk_device::read_data_cb(&apollo_dn300_disk_ctrlr_device::ansi_disk_read_data, this));
    our_disks[0]->set_ref_clock_tick_cb(ansi_disk_device::pulse_cb(&apollo_dn300_disk_ctrlr_device::ansi_disk_ref_clock_tick, this));
	our_disks[0]->set_index_pulse_cb(ansi_disk_device::pulse_cb(&apollo_dn300_disk_ctrlr_device::ansi_index_pulse, this));
	our_disks[0]->set_sector_pulse_cb(ansi_disk_device::pulse_cb(&apollo_dn300_disk_ctrlr_device::ansi_sector_pulse, this));

    our_disks[1]->set_attention_cb(ansi_disk_device::bool_cb(&apollo_dn300_disk_ctrlr_device::ansi_disk_attention, this));
    our_disks[1]->set_busy_cb(ansi_disk_device::bool_cb(&apollo_dn300_disk_ctrlr_device::ansi_disk_busy, this));
    our_disks[1]->set_read_data_cb(ansi_disk_device::read_data_cb(&apollo_dn300_disk_ctrlr_device::ansi_disk_read_data, this));
    our_disks[1]->set_ref_clock_tick_cb(ansi_disk_device::pulse_cb(&apollo_dn300_disk_ctrlr_device::ansi_disk_ref_clock_tick, this));
	our_disks[1]->set_index_pulse_cb(ansi_disk_device::pulse_cb(&apollo_dn300_disk_ctrlr_device::ansi_index_pulse, this));
	our_disks[1]->set_sector_pulse_cb(ansi_disk_device::pulse_cb(&apollo_dn300_disk_ctrlr_device::ansi_sector_pulse, this));

	// floppy_image_device *floppy = m_floppy->get_device();
	// if (floppy != nullptr) {
	// 	m_fdc->set_floppy(floppy);
	// }
}

static const char* command_names[] = {
	"NOOP",
	"READ_RECORD",
	"WRITE_RECORD",
	"FORMAT_TRACK",
	"SEEK",
	"EXEC_ANSI_CMD",
	"EXEC_DRIVE_SELECT",
	"EXEC_ATTENTION",
	"SELECT_HEAD"
};

void
apollo_dn300_disk_ctrlr_device::map(address_map &map)
{
	// custom hard drive controller
	map(0x00, 0x0F).rw(FUNC(apollo_dn300_disk_ctrlr_device::wdc_read), FUNC(apollo_dn300_disk_ctrlr_device::wdc_write));

	// standard floppy controller
	map(0x10, 0x10).r(FUNC(apollo_dn300_disk_ctrlr_device::fdc_msr_r));
	map(0x12, 0x12).rw(m_fdc, FUNC(upd765a_device::fifo_r), FUNC(upd765a_device::fifo_w));
	map(0x14, 0x14).w(FUNC(apollo_dn300_disk_ctrlr_device::fdc_control_w));

    // our rtc
	map(0x20, 0x21).w(FUNC(apollo_dn300_disk_ctrlr_device::calendar_ctrl_w));
	map(0x22, 0x23).w(FUNC(apollo_dn300_disk_ctrlr_device::calendar_data_w));
	map(0x24, 0x25).r(FUNC(apollo_dn300_disk_ctrlr_device::calendar_data_r));
}

void
apollo_dn300_disk_ctrlr_device::fdc_irq(int state)
{
	SLOG1(("DN300_DISK_CTRLR: floppy irq %d (enabled? %s)", state, m_fdc_control & 0x02 ? "yes" : "no"));
	if (m_fdc_control & 0x02) {
        if (state) {
            m_controller_status_high |= CONTROLLER_STATUS_HIGH_FLOPPY_INTERRUPTING;
        }
		// irq_cb(state);
	}
}

uint8_t
apollo_dn300_disk_ctrlr_device::fdc_msr_r(offs_t, uint8_t mem_mask)
{
	uint8_t fdc_status = m_fdc->msr_r();
	SLOG1(("DN300_DISK_CTRLR: floppy msr read= %02x (mask %02x)", fdc_status, mem_mask));

	m_controller_status_high &= ~CONTROLLER_STATUS_HIGH_FLOPPY_INTERRUPTING;
	irq_cb(CLEAR_LINE);
	return fdc_status;
}

void
apollo_dn300_disk_ctrlr_device::fdc_control_w(offs_t offset, uint8_t data, uint8_t mem_mask)
{
	SLOG1(("DN300_DISK_CTRLR: floppy control write = %02x (mask %02x)", data, mem_mask));

	m_fdc_control = data;

	if (m_fdc_control & 0x02) {
		SLOG1(("DN300_DISK_CTRLR:   floppy interrupt enabled"));
	}
	if (m_fdc_control & 0x01) {
		SLOG1(("DN300_DISK_CTRLR:   floppy WRITE"));
	} else {
		SLOG1(("DN300_DISK_CTRLR:   floppy READ"));

		// we shouldn't need these pulses here.  the upd765a emulator will do it for us.

		// 0x200 * 2 for a 1k read.
		for (int i = 0; i < 0x200; i++) {
			PULSE_FLOPPY_DRQ(m_fdc);
		}
		// if irq is enabled we should trigger it here
		// if (m_fdc_control & 0x02) {
		// 	irq_cb(ASSERT_LINE);
		// }
	}
}

void
apollo_dn300_disk_ctrlr_device::calendar_ctrl_w(offs_t, uint8_t data, uint8_t mem_mask)
{
	SLOG1(("DN300_DISK_CTRLR: calendar_ctrl write = %02x", data));
	COMBINE_DATA(&m_calendar_ctrl);

	// not sure if this CS line handling is correct.  I expect if I trace the CS
	// line on the board I'll find that it routes directly to Vcc/GND depending
	// on its sense.  But this should be ~the same.
	if (m_calendar_ctrl) {
		m_rtc->cs_w(1);
	} else {
		m_rtc->cs_w(0);
	}

	// I think this is wrong, but it's as close as I can get without more local
	// handling of the read/write data registers.  The timing diagrams in the
	// MSM5832 datasheet give the order in which these lines should be asserted,
	// but in the write case the data lines need to be holding their value
	// before the write line is asserted.  This will only happen if the emulated
	// code writes the data before writing to this control address, and I'm not
	// sure we can guarantee that happens.
	m_rtc->hold_w(m_calendar_ctrl & 0x01);
	m_rtc->read_w((m_calendar_ctrl >> 2) & 0x01);
	m_rtc->address_w(m_calendar_ctrl >> 4);
	m_rtc->write_w((m_calendar_ctrl >> 1) & 0x01);
}

void
apollo_dn300_disk_ctrlr_device::calendar_data_w(offs_t offset, uint8_t data, uint8_t mem_mask)
{
	m_rtc->data_w(~data);
}

uint8_t
apollo_dn300_disk_ctrlr_device::calendar_data_r(offs_t offset, uint8_t mem_mask)
{
	return ~m_rtc->data_r();
}

void apollo_dn300_disk_ctrlr_device::wdc_write(offs_t offset, uint8_t data, uint8_t mem_mask)
{
    switch (offset) {
        // winchester register writes
        case WDC_REG_ANSI_CMD:
            SLOG1(("DN300_DISK_CTRLR: wdc ANSI_COMMAND write = %02x", data));
            m_wdc_ansi_cmd = data;
            break;
        case WDC_REG_ANSI_PARM:
            SLOG1(("DN300_DISK_CTRLR: wdc ANSI_PARM write = %02x", data));
            m_wdc_ansi_parm_out = data;
            break;
        case WDC_REG_SECTOR:
            SLOG1(("DN300_DISK_CTRLR: wdc SECTOR write = %02x", data));
            m_wdc_sector = data;
            break;
        case WDC_REG_CYLINDER_HIGH: {
            SLOG1(("DN300_DISK_CTRLR: wdc CYLINDER_HIGH write = %02x", data));
			m_wdc_cylinder_hi = data;
            break;
        }
        case WDC_REG_CYLINDER_LOW: {
            SLOG1(("DN300_DISK_CTRLR: wdc CYLINDER_LOW write = %02x", data));
			m_wdc_cylinder_lo = data;
            break;
        }
        case WDC_REG_HEAD: {
            SLOG1(("DN300_DISK_CTRLR: HEAD write = %02x", data));
            m_wdc_head = data;
            break;
        }
        case WDC_REG_INTERRUPT_CONTROL:
            SLOG1(("DN300_DISK_CTRLR: wdc INTERRUPT_CONTROL write = %02x", data));
            m_wdc_interrupt_control = data;
			// maybe we _always_ clear the interrupt line here?
			irq_cb(CLEAR_LINE);

			// if ((m_wdc_interrupt_control & WDC_IRQCTRL_ENABLE_OVERALL) == 0) {
			// 	irq_cb(CLEAR_LINE);
			// }
			if ((m_wdc_interrupt_control & WDC_IRQCTRL_ENABLE_STATUS_AVAIL) == 0) {
				m_controller_status_high &= ~CONTROLLER_STATUS_HIGH_STATUS_AVAILABLE_INTERRUPT;
			}
			if ((m_wdc_interrupt_control & WDC_IRQCTRL_ENABLE_ATTENTION) == 0) {
				m_controller_status_high &= ~CONTROLLER_STATUS_HIGH_DRIVE_ATTENTION;
			}
			if ((m_wdc_interrupt_control & WDC_IRQCTRL_ENABLE_END_OF_OP) == 0) {
				m_controller_status_high &= ~CONTROLLER_STATUS_HIGH_END_OF_OP_INTERRUPT;
			}
            break;
        case WDC_REG_CONTROLLER_COMMAND:
            SLOG1(("DN300_DISK_CTRLR: wdc CONTROLLER_COMMAND write = %02x", data));
            m_controller_command = data;
            execute_command();
            break;

        default:
            SLOG1(("DN300_DISK_CTRLR: unknown wdc write to offset %02x = %02x & %08x", offset, data, mem_mask));
            break;
    }
}

uint8_t apollo_dn300_disk_ctrlr_device::wdc_read(offs_t offset, uint8_t mem_mask)
{
    switch (offset) {
        // winchester register reads
        case WDC_REG_ATTENTION_STATUS:
            SLOG1(("DN300_DISK_CTRLR: wdc ATTENTION_STATUS read = %02x", m_wdc_attention_status));
            // reading this clears the status available interrupt bit as well as the status timeout bit
            m_controller_status_high &= ~(
				CONTROLLER_STATUS_HIGH_STATUS_AVAILABLE_INTERRUPT
			);
			m_controller_status_low &= ~(
				CONTROLLER_STATUS_LOW_STATUS_TIMEOUT
			);
            return m_wdc_attention_status;
        case WDC_REG_ANSI_PARM:
            SLOG1(("DN300_DISK_CTRLR: wdc ANSI_PARM read = %02x", m_wdc_ansi_parm_in));
            return m_wdc_ansi_parm_in;
        case WDC_REG_STATUS_HIGH:
            SLOG1(("DN300_DISK_CTRLR: wdc STATUS_HIGH read = %02x", m_controller_status_high, mem_mask));
            return m_controller_status_high;
        case WDC_REG_STATUS_LOW:
            SLOG1(("DN300_DISK_CTRLR: wdc STATUS_LOW read = %02x", m_controller_status_low));
            return m_controller_status_low;

        default:
            SLOG1(("DN300_DISK_CTRLR: unknown wdc read at offset %02x & %08x", offset, mem_mask));
            return 0;
    }
}


void apollo_dn300_disk_ctrlr_device::execute_command()
{
	if (m_controller_status_high == CONTROLLER_STATUS_HIGH_CONTROLLER_BUSY) {
		SLOG1(("controller was busy and we started another command?"));
		abort();
	}

	irq_cb(CLEAR_LINE);

    // clear the status bits that clear on command
    m_controller_status_high &= ~CONTROLLER_STATUS_HIGH_END_OF_OP_INTERRUPT;
    m_controller_status_low &= ~(
        CONTROLLER_STATUS_LOW_TIMEOUT |
        CONTROLLER_STATUS_LOW_OVERRUN |
        CONTROLLER_STATUS_LOW_CRC_ERROR |
        CONTROLLER_STATUS_LOW_BUS_PARITY_ERROR |
        CONTROLLER_STATUS_LOW_ILLEGAL_CONFIG |
        CONTROLLER_STATUS_LOW_DMA_PARITY_ERROR
    );

    SLOG1(("DN300_DISK_CTRLR: execute_command %02x (%s)", m_controller_command, command_names[m_controller_command]))
    switch (m_controller_command) {
        case WDC_CONTROLLER_CMD_NOOP:
			// clear pending interrupts?
            break;

        case WDC_CONTROLLER_CMD_READ_RECORD: {
            // we're busy reading
            m_controller_status_high |= CONTROLLER_STATUS_HIGH_CONTROLLER_BUSY;

            // we'll signal the end of the op in our ansi_disk_read_data callback
            m_word_transfer_count = 0;
			m_start_read_sector = true;
            break;
        }
        case WDC_CONTROLLER_CMD_WRITE_RECORD: {
            // we're busy writing
            m_controller_status_high |= CONTROLLER_STATUS_HIGH_CONTROLLER_BUSY;

            // we'll signal the end of the op in check_for_sector_write below.
            m_word_transfer_count = 0;
			m_start_write_sector = true;
            break;
        }

        case WDC_CONTROLLER_CMD_FORMAT_TRACK:
            SLOG1(("DN300_DISK_CTRLR:    CMD_FORMAT_TRACK unimplemented"))
            break;

        case WDC_CONTROLLER_CMD_SEEK: {
            ansi_disk_device *disk = our_disks[m_wdc_selected_drive-1];

            disk->execute_command(ANSI_CMD_LOAD_CYL_ADDR_HIGH, m_wdc_cylinder_hi);
            disk->execute_command(ANSI_CMD_LOAD_CYL_ADDR_LOW, m_wdc_cylinder_lo);
            disk->execute_command(ANSI_CMD_SEEK, m_wdc_ansi_parm_out);

            break;
        }

        case WDC_CONTROLLER_CMD_EXEC_ANSI_CMD: {
            ansi_disk_device *disk = our_disks[m_wdc_selected_drive-1];
            m_wdc_ansi_parm_in = disk->execute_command(m_wdc_ansi_cmd, m_wdc_ansi_parm_out);
            break;
        }

        case WDC_CONTROLLER_CMD_EXEC_DRIVE_SELECT: {
            m_wdc_selected_drive = m_wdc_ansi_parm_out;
			SLOG1(("DN300_DISK_CTRLR: selecting disk %d", m_wdc_selected_drive))
			m_pulsed_sector = UNKNOWN_SECTOR;
            ansi_disk_device *disk = our_disks[m_wdc_selected_drive-1];
			disk->select();
            break;
		}

        case WDC_CONTROLLER_CMD_EXEC_ATTENTION_IN:
            SLOG1(("DN300_DISK_CTRLR:    CMD_EXEC_ATTENTION_IN unimplemented"))
            break;

        case WDC_CONTROLLER_CMD_SELECT_HEAD: {
            ansi_disk_device *disk = our_disks[m_wdc_selected_drive-1];
            m_wdc_ansi_parm_in = disk->execute_command(ANSI_CMD_SELECT_HEAD, m_wdc_head);
            break;
        }
        default:
            SLOG1(("DN300_DISK_CTRLR:    unknown controller command %02x", m_controller_command));
            break;
    }
}

uint8_t apollo_dn300_disk_ctrlr_device::dma_read_byte(offs_t offset)
{
	if (m_floppy_drq_state) {
		uint8_t data =  m_fdc->dma_r();
		SLOG2(("DN300_DISK_CTRLR: reading disk DMA from FDC FIFO offset %d.  data = %02x", offset, data));
		return data;
	}
    // send back the byte pointed to by the cursor
    uint8_t rv = m_buffer[m_bytes_read++];
    SLOG2(("DN300_DISK_CTRLR: reading disk DMA: %02x", rv));
    if (m_bytes_read == 2) {
        // we've read the last byte of the buffer
        m_bytes_read = 0;

		drq_cb(false);

        m_word_transfer_count++;
        if (m_word_transfer_count == HARD_DISK_SECTOR_SIZE / 2) {
            // we're done with the read.  let the cpu know
		    ansi_disk_device *disk = our_disks[m_wdc_selected_drive-1];
            disk->deassert_read_gate();
            end_of_controller_op();
        }
    }
    return rv;
}

void apollo_dn300_disk_ctrlr_device::dma_write_byte(offs_t offset, uint8_t data)
{
	// XXX handle floppy case

    SLOG2(("writing disk DMA at offset %02x = %02x", offset, data));

    ansi_disk_device *disk = our_disks[m_wdc_selected_drive-1];

	disk->write_sector_next_byte(data);
}

void apollo_dn300_disk_ctrlr_device::ansi_index_pulse(ansi_disk_device *) {
	m_pulsed_sector = 0;
	check_for_sector_read();
	check_for_sector_write();
}

void apollo_dn300_disk_ctrlr_device::ansi_sector_pulse(ansi_disk_device *) {
	// we don't do anything until we've seen the index pulse
	if (m_pulsed_sector == UNKNOWN_SECTOR) {
		return;
	}

	m_pulsed_sector++;
	check_for_sector_read();
	check_for_sector_write();
}

void apollo_dn300_disk_ctrlr_device::check_for_sector_read() {
	if (!m_start_read_sector) {
		return;
	}

	if (m_pulsed_sector != m_wdc_sector) {
		SLOG2(("DN300_DISK_CTRLR: check_for_sector_read: waiting to read. head is at %d, but we want %d", m_pulsed_sector, m_wdc_sector));
		return;
	}

	SLOG1(("DN300_DISK_CTRLR: check_for_sector_read: start read sector at sector %d", m_pulsed_sector));
	m_start_read_sector = false;

	ansi_disk_device *disk = our_disks[m_wdc_selected_drive-1];
	disk->assert_read_gate();
}

void apollo_dn300_disk_ctrlr_device::check_for_sector_write() {
	if (!m_start_write_sector) {
		return;
	}

	if (m_pulsed_sector != m_wdc_sector) {
		SLOG2(("DN300_DISK_CTRLR: check_for_sector_write: waiting to write head is at %d, but we want %d", m_pulsed_sector, m_wdc_sector));
		return;
	}

	SLOG1(("DN300_DISK_CTRLR: check_for_sector_write: start write sector at sector %d", m_pulsed_sector));
	m_start_write_sector = false;

	ansi_disk_device *disk = our_disks[m_wdc_selected_drive-1];
	disk->assert_write_gate();
}
