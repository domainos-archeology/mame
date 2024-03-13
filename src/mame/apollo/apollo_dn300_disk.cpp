
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
    , m_rtc(*this, APOLLO_DN300_RTC_TAG)
	, m_fdc(*this, APOLLO_DN300_FLOPPY_TAG)
    , m_floppy(*this, APOLLO_DN300_FLOPPY_TAG":0")
	, m_floppy_drq_state(false)
    , m_wdc_ansi_cmd(0)
    , m_wdc_ansi_parm(0)
    , m_wdc_sector(0)
    , m_wdc_head(0)
    , m_wdc_interrupt_control(0)
    , m_controller_command(0)
    , m_controller_status_high(0)
    , m_controller_status_low(0)
    , m_wdc_selected_head(0)
    , m_wdc_selected_drive(0)
    , m_wdc_general_status(0)
    , m_wdc_sense_byte_1(0)
    , m_wdc_sense_byte_2(0)
    , m_wdc_write_enabled(false)
    , m_wdc_attention_enabled(true)
    , m_cursor(0)
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
		SLOG1(("DN300_DISK: floppy drq %s", state ? "asserted" : "cleared"));
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

	m_timer = timer_alloc(FUNC(apollo_dn300_disk_ctrlr_device::trigger_interrupt), this);

    // save_item(NAME(drq));
    our_disks[0] = subdevice<ansi_disk_device>(DN300_DISK0_TAG);
    our_disks[1] = subdevice<ansi_disk_device>(DN300_DISK1_TAG);

    SLOG1(("in apollo_dn300_disk_ctrlr_device::device_start: disks = [%p, %p]", our_disks[0], our_disks[1]));
	// floppy_image_device *floppy = m_floppy->get_device();
	// if (floppy != nullptr) {
	// 	m_fdc->set_floppy(floppy);
	// }
}

void apollo_dn300_disk_ctrlr_device::ansi_disk0_attention(ansi_disk_device *device, bool state) {
    SLOG1(("DN300_DISK: disk0 attn %s", state ? "asserted" : "cleared"));
    if (state) {
        m_controller_status_high |= CONTROLLER_STATUS_HIGH_DRIVE_ATTENTION;
    } else {
        m_controller_status_high &= ~CONTROLLER_STATUS_HIGH_DRIVE_ATTENTION;
    }
}

void apollo_dn300_disk_ctrlr_device::ansi_disk0_read_data(ansi_disk_device *disk, uint8_t data) {
    SLOG1(("DN300_DISK: disk0 read data: m_buffer[%d] = %02x", m_cursor, data));
    m_buffer[m_cursor++] = data;
    if (m_cursor == 2) {
        m_cursor = 0;
        PULSE_DRQ();

        m_read_record_word_count++;
        if (m_read_record_word_count == HARD_DISK_SECTOR_SIZE / 2) {
            // we're done with the read.  let the cpu know
            disk->finish_read_sector(0);
            end_of_controller_op();
            return;
        }
    }
}

void apollo_dn300_disk_ctrlr_device::ansi_disk1_attention(ansi_disk_device *device, bool state) {
    SLOG1(("DN300_DISK: disk1 attn %s", state ? "asserted" : "cleared"));
    if (state) {
        m_controller_status_high |= CONTROLLER_STATUS_HIGH_DRIVE_ATTENTION;
    } else {
        m_controller_status_high &= ~CONTROLLER_STATUS_HIGH_DRIVE_ATTENTION;
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

    SLOG1(("DN300_DISK: registering attn callbacks"));
    our_disks[0]->set_attention_cb(ansi_disk_device::attention_cb(&apollo_dn300_disk_ctrlr_device::ansi_disk0_attention, this));
    our_disks[0]->set_read_data_cb(ansi_disk_device::read_data_cb(&apollo_dn300_disk_ctrlr_device::ansi_disk0_read_data, this));
    our_disks[1]->set_attention_cb(ansi_disk_device::attention_cb(&apollo_dn300_disk_ctrlr_device::ansi_disk1_attention, this));

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

TIMER_CALLBACK_MEMBER(apollo_dn300_disk_ctrlr_device::trigger_interrupt)
{
	m_controller_status_high &= ~CONTROLLER_STATUS_HIGH_CONTROLLER_BUSY;
	irq_cb(ASSERT_LINE);
}

void
apollo_dn300_disk_ctrlr_device::fdc_irq(int state)
{
	SLOG1(("DN300_DISK: floppy irq %d (enabled? %s)", state, m_fdc_control & 0x02 ? "yes" : "no"));
    if (state) {
        m_controller_status_high |= CONTROLLER_STATUS_HIGH_FLOPPY_INTERRUPTING;
    }
	if (m_fdc_control & 0x02) {
        m_timer->adjust(attotime::from_msec(10), 0);
		// irq_cb(state);
	}
}

uint8_t
apollo_dn300_disk_ctrlr_device::fdc_msr_r(offs_t, uint8_t mem_mask)
{
	uint8_t fdc_status = m_fdc->msr_r();
	SLOG1(("DN300_DISK: floppy msr read= %02x (mask %02x)", fdc_status, mem_mask));

	// EH87 says reading the floppy status reg clears this bit, but I wonder if
	// it shouldn't be a side effect of the controller clearing the irq line?
	m_controller_status_high &= ~CONTROLLER_STATUS_HIGH_FLOPPY_INTERRUPTING;
	irq_cb(CLEAR_LINE);
	return fdc_status;
}

void
apollo_dn300_disk_ctrlr_device::fdc_control_w(offs_t offset, uint8_t data, uint8_t mem_mask)
{
	SLOG1(("DN300_DISK: floppy control write = %02x (mask %02x)", data, mem_mask));

	m_fdc_control = data;

	if (m_fdc_control & 0x02) {
		SLOG1(("DN300_DISK:   floppy interrupt enabled"));
	}
	if (m_fdc_control & 0x01) {
		SLOG1(("DN300_DISK:   floppy WRITE"));
	} else {
		SLOG1(("DN300_DISK:   floppy READ"));

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
	SLOG1(("DN300_DISK: calendar_ctrl write = %02x", data));
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
        case WDC_REG_CYLINDER_HIGH: {
            SLOG1(("DN300_DISK: wdc CYLINDER_HIGH write = %02x", data));
            ansi_disk_device *disk = our_disks[m_wdc_selected_drive-1];
            disk->execute_command(ANSI_CMD_LOAD_CYL_ADDR_HIGH, data);
            break;
        }
        case WDC_REG_CYLINDER_LOW: {
            SLOG1(("DN300_DISK: wdc CYLINDER_LOW write = %02x", data));
            ansi_disk_device *disk = our_disks[m_wdc_selected_drive-1];
            disk->execute_command(ANSI_CMD_LOAD_CYL_ADDR_LOW, data);
            break;
        }
        case WDC_REG_HEAD: {
            SLOG1(("DN300_DISK: HEAD write = %02x", data));
            ansi_disk_device *disk = our_disks[m_wdc_selected_drive-1];
            disk->execute_command(ANSI_CMD_SELECT_HEAD, data);
            break;
        }
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

uint8_t apollo_dn300_disk_ctrlr_device::wdc_read(offs_t offset, uint8_t mem_mask)
{
    switch (offset) {
        // winchester register reads
        case WDC_REG_ATTENTION_STATUS:
            SLOG1(("DN300_DISK: wdc ATTENTION_STATUS read = %02x", m_wdc_general_status));
            // reading this clears some status bits
            m_controller_status_high &= ~(
				CONTROLLER_STATUS_HIGH_STATUS_AVAILABLE_INTERRUPT |
				CONTROLLER_STATUS_HIGH_DRIVE_ATTENTION
			);
			irq_cb(CLEAR_LINE);
            return m_wdc_general_status;
        case WDC_REG_ANSI_PARM:
            SLOG1(("DN300_DISK: wdc ANSI_PARM read = %02x", m_wdc_ansi_parm));
            return m_wdc_ansi_parm;
        case WDC_REG_STATUS_HIGH:
            // SLOG1(("DN300_DISK: wdc STATUS_HIGH read = %02x & %02x", m_controller_status_high, mem_mask));
            return m_controller_status_high;
        case WDC_REG_STATUS_LOW:
            SLOG1(("DN300_DISK: wdc STATUS_LOW read = %02x", m_controller_status_low));
            return m_controller_status_low;

        default:
            SLOG1(("DN300_DISK: unknown wdc read at offset %02x & %08x", offset, mem_mask));
            return 0;
    }
}


void apollo_dn300_disk_ctrlr_device::execute_command()
{
	if (m_controller_status_high == CONTROLLER_STATUS_HIGH_CONTROLLER_BUSY) {
		SLOG1(("controller was busy and we started another command?"));
		abort();
	}

	bool need_interrupt = false;
	int command_duration = 0;

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

	m_controller_status_high |= CONTROLLER_STATUS_HIGH_CONTROLLER_BUSY;

    SLOG1(("DN300_DISK: execute_command %02x (%s)", m_controller_command, command_names[m_controller_command]))
    switch (m_controller_command) {
        case WDC_CONTROLLER_CMD_NOOP:
			// clear pending interrupts?
			m_timer->adjust(attotime::never);
            break;

        case WDC_CONTROLLER_CMD_READ_RECORD: {
            ansi_disk_device *disk = our_disks[m_wdc_selected_drive-1];

            // we're busy reading
            m_controller_status_high |= CONTROLLER_STATUS_HIGH_CONTROLLER_BUSY;

            // we'll signal the end of the op in our ansi_disk#_read_data callbacks
            m_read_record_word_count = 0;
            disk->start_read_sector(m_wdc_sector);
            break;
        }
#if 0
            m_wdc_general_status |= GS_BUSY_EXECUTING;

            ansi_disk_device *disk = our_disks[m_wdc_selected_drive-1];

            int cylinder = (disk->m_current_cylinder_high << 8) | disk->m_current_cylinder_low;
            int track = cylinder * disk->m_heads + m_wdc_head;
            int sector_offset = track * disk->m_sectors;
            int sector = sector_offset + m_wdc_sector;

            SLOG1(("DN300_DISK:    CMD_READ_RECORD for drive %d sector %d on cylinder %d and head %d", m_wdc_selected_drive, m_wdc_sector, cylinder, m_wdc_head));
            SLOG1(("DN300_DISK:    linearized as logical sector address %d", sector));

			if (!disk) {
				SLOG1(("%p: disk is null?", this));
			}
			if (!disk->m_image) {
				SLOG1(("%p: disk image is null?", this));
			}
            disk->m_image->fseek(sector * HARD_DISK_SECTOR_SIZE, SEEK_SET);
            disk->m_image->fread(m_buffer, HARD_DISK_SECTOR_SIZE);

            m_cursor = 0;

            // 0x10 for the first operation
            for (int i = 0; i < 0x10; i++) {
                PULSE_DRQ();
            }

            // 0x200 for the second operation
            for (int i = 0; i < 0x200; i++) {
                PULSE_DRQ();
            }

		    SLOG1(("DN300_DISK:    done with synchronous read"));
            m_wdc_general_status &= ~GS_BUSY_EXECUTING;
            m_wdc_general_status |= GS_NORMAL_COMPLETE;

			if (m_wdc_interrupt_control & WDC_IRQCTRL_ENABLE_END_OF_OP) {
	            m_controller_status_high |= CONTROLLER_STATUS_HIGH_END_OF_OP_INTERRUPT;
				SLOG1(("DN300_DISK:    irqctrl end of op set, interrupting in 10ms"));
                need_interrupt = true;
				command_duration = 10;
            }
            if (m_wdc_interrupt_control & WDC_IRQCTRL_ENABLE_STATUS_AVAIL) {
                m_controller_status_high |= CONTROLLER_STATUS_HIGH_STATUS_AVAILABLE_INTERRUPT;
				SLOG1(("DN300_DISK:    irqctrl status avail set, interrupting in 10ms"));
                need_interrupt = true;
				command_duration = 10;
			}

			if (!need_interrupt) {
				SLOG1(("DN300_DISK:    irqctrl not set, not interrupting"));
                m_controller_status_high |= CONTROLLER_STATUS_HIGH_DRIVE_ATTENTION;
            }

            break;
        }
#endif
        case WDC_CONTROLLER_CMD_WRITE_RECORD: {
            ansi_disk_device *disk = our_disks[m_wdc_selected_drive-1];

            // we're busy writing
            m_controller_status_high |= CONTROLLER_STATUS_HIGH_CONTROLLER_BUSY;

            // we'll signal the end of the op in our ansi_disk#_read_data callbacks
            m_read_record_word_count = 0;
            disk->start_write_sector();

            // XXX add a timer here to use as a write clock

            // 0x10 for the first operation
            for (int i = 0; i < 0x10; i++) {
                PULSE_DRQ();
            }

            // 0x200 for the second operation
            for (int i = 0; i < 0x200; i++) {
                PULSE_DRQ();
            }

            disk->finish_write_sector(m_wdc_sector, 0);
            end_of_controller_op();

            break;
        }
#if 0
            m_wdc_general_status |= GS_BUSY_EXECUTING;

            ansi_disk_device *disk = our_disks[m_wdc_selected_drive-1];

            int cylinder = (disk->m_current_cylinder_high << 8) | disk->m_current_cylinder_low;
            int track = cylinder * disk->m_heads + m_wdc_head;
            int sector_offset = track * disk->m_sectors;
            int sector = sector_offset + m_wdc_sector;

            SLOG1(("DN300_DISK:    CMD_WRITE_RECORD for drive %d sector %d on cylinder %d and head %d", m_wdc_selected_drive, m_wdc_sector, cylinder, m_wdc_head));
            SLOG1(("DN300_DISK:    linearized as logical sector address %d", sector));

            m_cursor = 0;

            // 0x10 for the first operation
            for (int i = 0; i < 0x10; i++) {
                PULSE_DRQ();
            }

            // 0x200 for the second operation
            for (int i = 0; i < 0x200; i++) {
                PULSE_DRQ();
            }

			SLOG1(("writing buffer to disk:"))
			int idx = 0;
			for (int i = 0; i < 1056/16; i++) {
				std::ostringstream line;
				// util::string_format(line, "%08x: ", idx + 0x420);
				for (int j = 0; j < 16; j ++) {
					util::stream_format(line, "%02x ", m_buffer[idx++]);
					if (j == 7) {
						line << " ";
					}
				}
				SLOG1((line.str().c_str()));
			}

            disk->m_image->fseek(sector * HARD_DISK_SECTOR_SIZE, SEEK_SET);
            disk->m_image->fwrite(m_buffer, HARD_DISK_SECTOR_SIZE);

		    SLOG1(("DN300_DISK:    done with synchronous write"));
            m_wdc_general_status &= ~GS_BUSY_EXECUTING;
            m_wdc_general_status |= GS_NORMAL_COMPLETE;

			if (m_wdc_interrupt_control & WDC_IRQCTRL_ENABLE_END_OF_OP) {
	            m_controller_status_high |= CONTROLLER_STATUS_HIGH_END_OF_OP_INTERRUPT;
				SLOG1(("DN300_DISK:    irqctrl end of op set, interrupting in 10ms"));
                need_interrupt = true;
				command_duration = 10;
            }
            if (m_wdc_interrupt_control & WDC_IRQCTRL_ENABLE_STATUS_AVAIL) {
                m_controller_status_high |= CONTROLLER_STATUS_HIGH_STATUS_AVAILABLE_INTERRUPT;
				SLOG1(("DN300_DISK:    irqctrl status avail set, interrupting in 10ms"));
                need_interrupt = true;
				command_duration = 10;
			}

			if (!need_interrupt) {
				SLOG1(("DN300_DISK:    irqctrl not set, not interrupting"));
                m_controller_status_high |= CONTROLLER_STATUS_HIGH_DRIVE_ATTENTION;
            }

            break;
		}
#endif

        case WDC_CONTROLLER_CMD_FORMAT_TRACK:
            SLOG1(("DN300_DISK:    CMD_FORMAT_TRACK unimplemented"))
            break;

        case WDC_CONTROLLER_CMD_SEEK:
#if 0
            SLOG1(("DN300_DISK:    to cylinder %02x%02x", m_wdc_load_cylinder_high, m_wdc_load_cylinder_low))
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

            m_wdc_general_status |= GS_BUSY_EXECUTING;

            // if this were an async emulator we'd return here,
            // but instead we jump immediately to steps performed after the
            // operation is done:
			m_wdc_current_cylinder_high = m_wdc_load_cylinder_high;
			m_wdc_current_cylinder_low = m_wdc_load_cylinder_low;

            m_wdc_general_status &= ~GS_BUSY_EXECUTING;
            if (m_wdc_attention_enabled) {
                m_controller_status_high |= CONTROLLER_STATUS_HIGH_DRIVE_ATTENTION;
				SLOG1(("DN300_DISK:    HELLO2 %02x", m_wdc_interrupt_control));
				// if (m_wdc_interrupt_control & WDC_IRQCTRL_ENABLE_OVERALL) {
					m_controller_status_high |= CONTROLLER_STATUS_HIGH_DRIVE_ATTENTION;
					// m_wdc_general_status |= 0xb;
					// status available?
					if (m_wdc_interrupt_control & WDC_IRQCTRL_ENABLE_STATUS_AVAIL) {
						m_controller_status_high |= CONTROLLER_STATUS_HIGH_STATUS_AVAILABLE_INTERRUPT;
						m_controller_status_high &= ~CONTROLLER_STATUS_HIGH_DRIVE_ATTENTION;
			            m_wdc_general_status |= GS_NORMAL_COMPLETE;
						need_interrupt = true;

						command_duration = 3;
					}

                    // if (m_wdc_interrupt_control & WDC_IRQCTRL_ENABLE_END_OF_OP) {
					// 	m_controller_status_high |= CONTROLLER_STATUS_HIGH_END_OF_OP_INTERRUPT;
					// 	need_interrupt = true;
					// 	command_duration = 1;
					// }
            }
#else
            {
                ansi_disk_device *disk = our_disks[m_wdc_selected_drive-1];
                disk->execute_command(ANSI_CMD_SEEK, m_wdc_ansi_parm);
            }
#endif
            break;

        case WDC_CONTROLLER_CMD_EXEC_ANSI_CMD: {
            ansi_disk_device *disk = our_disks[m_wdc_selected_drive-1];
            m_wdc_ansi_parm = disk->execute_command(m_wdc_ansi_cmd, m_wdc_ansi_parm);
            break;
        }

        case WDC_CONTROLLER_CMD_EXEC_DRIVE_SELECT:
            m_wdc_selected_drive = m_wdc_ansi_parm;
			SLOG1(("DN300_DISK: selecting disk %d", m_wdc_selected_drive))
            break;

        case WDC_CONTROLLER_CMD_EXEC_ATTENTION_IN:
            SLOG1(("DN300_DISK:    CMD_EXEC_ATTENTION_IN unimplemented"))
            break;

        case WDC_CONTROLLER_CMD_SELECT_HEAD: {
#if 0
            // Guessing this is equivalent to the ansi select head command?

            // This command shall condition the selected device to accept the
            // Parameter Byte as the binary address of the head selected for read
            // or write operations. This command shall enable the moving heads
            // and shall disable the fixed heads.
            // A Select Moving Head Command issued during a read or write
            // operation is a violation of protocol.
            // The device shall set the Attention Condition and the Illegal
            // Parameter Bit in the General Status Byte upon receipt of a head
            // address outside the head address range of the device.
            // Devices shall be initialized with moving head zero selected.

            m_wdc_general_status |= GS_BUSY_EXECUTING;

            // if this were an async emulator we'd return here,
            // but instead we jump immediately to steps performed after the
            // operation is done:
            m_wdc_selected_head = m_wdc_head;

            m_wdc_general_status &= ~GS_BUSY_EXECUTING;
            if (m_wdc_attention_enabled) {
                // m_wdc_status_high |= CONTROLLER_STATUS_HIGH_DRIVE_ATTENTION;
				SLOG1(("DN300_DISK:    HELLO1 %02x", m_wdc_interrupt_control));
				if ((m_wdc_interrupt_control & WDC_IRQCTRL_ENABLE_OVERALL) &&
				    (m_wdc_interrupt_control & WDC_IRQCTRL_ENABLE_ATTENTION)) {
						// need_interrupt = true;
				}
            }

            break;
        }
#endif
            ansi_disk_device *disk = our_disks[m_wdc_selected_drive-1];
            m_wdc_ansi_parm = disk->execute_command(ANSI_CMD_SELECT_HEAD, m_wdc_head);
            break;
        }
        default:
            SLOG1(("DN300_DISK:    unknown controller command %02x", m_controller_command));
            break;
    }

	if (need_interrupt) {
		if (command_duration == 0) {
			SLOG1(("DN300_DISK:    irq_cb now"));
			m_controller_status_high &= ~CONTROLLER_STATUS_HIGH_CONTROLLER_BUSY;
			irq_cb(ASSERT_LINE);
		} else {
			SLOG1(("DN300_DISK:    irq_cb in %d ms", command_duration));
			m_timer->adjust(attotime::from_msec(command_duration), 0);
		}
	} else {
		m_controller_status_high &= ~CONTROLLER_STATUS_HIGH_CONTROLLER_BUSY;
	}
}

uint8_t apollo_dn300_disk_ctrlr_device::dma_read_byte(offs_t offset)
{
	if (m_floppy_drq_state) {
		uint8_t data =  m_fdc->dma_r();
		SLOG1(("DN300_DISK: reading disk DMA from FDC FIFO offset %d.  data = %02x", offset, data));
		return data;
	}
    // send back the byte pointed to by the cursor
    uint8_t rv = m_buffer[m_cursor++];
    SLOG1(("DN300_DISK: reading disk DMA: %02x", rv));
    if (m_cursor == 2) {
        // we've read the last byte of the buffer
        m_cursor = 0;
    }
    return rv;
}

void apollo_dn300_disk_ctrlr_device::dma_write_byte(offs_t offset, uint8_t data)
{
    SLOG1(("writing disk DMA at offset %02x = %02x", offset, data));

    ansi_disk_device *disk = our_disks[m_wdc_selected_drive-1];
   
	disk->write_sector_next_byte(data);
}
