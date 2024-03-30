// license:BSD-3-Clause
// copyright-holders:Chris Toshok
/*
 * apollo_dn300_disk.h
 *
 *  Created on: Nov 1, 2022
 *      Author: Chris Toshok
 *
 */

#pragma once

#ifndef MAME_MACHINE_APOLLO_DN300_DISK_H
#define MAME_MACHINE_APOLLO_DN300_DISK_H

#include "machine/upd765.h"
#include "machine/msm5832.h"
#include "imagedev/floppy.h"

class ansi_disk_device;

class apollo_dn300_disk_ctrlr_device :
    public device_t
{
public:
	//construction/destruction
	apollo_dn300_disk_ctrlr_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	auto irq_callback() { return irq_cb.bind(); }
	auto drq_wr_callback() { return drq_cb.bind(); }

	uint8_t dma_read_byte(offs_t offset);
	void dma_write_byte(offs_t offset, uint8_t data);

    void map(address_map &map);

protected:
	// device-level overrides
	virtual void device_add_mconfig(machine_config &config) override;
	virtual void device_start() override;
	virtual void device_reset() override;

	static constexpr unsigned DN300_MAX_DISK = 2;
	ansi_disk_device *our_disks[DN300_MAX_DISK];

private:
	void end_of_controller_op();

	// controller-level irq/drq signals
	devcb_write_line irq_cb;
	devcb_write_line drq_cb;

	// WDC-specific
	void wdc_write(offs_t offset, uint8_t data, uint8_t mem_mask = ~0);
    uint8_t wdc_read(offs_t offset, uint8_t mem_mask = ~0);

	uint8_t m_wdc_selected_drive;

	// write registers
	uint8_t m_wdc_ansi_cmd;
	uint8_t m_wdc_ansi_parm; // should be _out
	uint8_t m_wdc_sector;
	uint8_t m_wdc_cylinder_hi;
	uint8_t m_wdc_cylinder_lo;
	uint8_t m_wdc_head;
	uint8_t m_wdc_interrupt_control;
	uint8_t m_controller_command;

	// read registers
	uint8_t m_wdc_attention_status;
	// XXX 	uint8_t m_wdc_ansi_parm_in;
	uint8_t m_wdc_drive_num_of_status;
	uint8_t m_controller_status_high;
	uint8_t m_controller_status_low;

	void execute_command();

	// internal buffer to help with our simulated reading (we read a byte
	// at a time, but the dma is done a word at a time.)
	uint8_t m_bytes_read;
	char m_buffer[2];

	int m_word_transfer_count;

	// our ansi disks generate sector/index pulses, which we need to
	// listen for in order to tell when the head is over the correct spot
	// (to assert/deassert the gate lines and start a read/write operation.)
	void ansi_index_pulse(ansi_disk_device *);
	void ansi_sector_pulse(ansi_disk_device *);

	int m_pulsed_sector;

	bool m_start_read_sector;
	bool m_start_write_sector;
	void check_for_sector_read();
	void check_for_sector_write();

	// callbacks representing other ansi disk signal lines - these should be line members,
	// but I couldn't get that to work
	void ansi_disk_attention(ansi_disk_device *disk, bool state);
	void ansi_disk_busy(ansi_disk_device *disk, bool state);
	void ansi_disk_ref_clock_tick(ansi_disk_device *disk);

	// for our simulated reading, we consume a byte at a time, instead of doing the NRZ
	// clocked reading that the real hardware does.  This function is called by the ansi_disk_device
	// for each byte of data in a sector via a timer tick, to make everything async.  The timer
	// is ticked to roughly how fast the real hardware would be reading the data, so... maybe that's
	// good enough?
	void ansi_disk_read_data(ansi_disk_device *disk, uint8_t data);


	// FDC-specific
	required_device<upd765a_device> m_fdc;
	required_device<floppy_connector> m_floppy;
	DECLARE_WRITE_LINE_MEMBER(fdc_irq);
	static void floppy_formats(format_registration &fr);

	// these might not really be necessary, as the controller card
	// uses a NEC 765A.
	void fdc_control_w(offs_t offset, uint8_t data, uint8_t mem_mask);
	uint8_t fdc_msr_r(offs_t, uint8_t mem_mask);

	bool m_floppy_drq_state;

	// write registers
	// XXX 	uint8_t m_fdc_write_data;
	uint8_t m_fdc_control;

	// read registers
	// XXX uint8_t m_fdc_status;
	// XXX uint8_t m_fdc_read_data;


	// Calendar-specific
	required_device<msm5832_device> m_rtc;

	// these might not really be necessary, as the controller card
	// uses an OKI MSM5832. TODO(toshok) verify this.
	void calendar_ctrl_w(offs_t offset, uint8_t data, uint8_t mem_mask = ~0);
	void calendar_data_w(offs_t offset, uint8_t data, uint8_t mem_mask = ~0);
	uint8_t calendar_data_r(offs_t offset, uint8_t mem_mask = ~0);

	// write registers
	uint8 m_calendar_ctrl;
	// XXX uint8 m_calendar_write_data;

	// read registers
	// XXX uint8 m_calendar_read_data;
};

// device type definition
DECLARE_DEVICE_TYPE(APOLLO_DN300_DISK_CTRLR, apollo_dn300_disk_ctrlr_device)

#endif // MAME_MACHINE_APOLLO_DN300_DISK_H
