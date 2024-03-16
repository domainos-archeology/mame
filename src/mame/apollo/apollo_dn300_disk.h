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

	void wdc_write(offs_t offset, uint8_t data, uint8_t mem_mask = ~0);
    uint8_t wdc_read(offs_t offset, uint8_t mem_mask = ~0);

	void fdc_control_w(offs_t offset, uint8_t data, uint8_t mem_mask);

	void calendar_ctrl_w(offs_t offset, uint8_t data, uint8_t mem_mask = ~0);
	void calendar_data_w(offs_t offset, uint8_t data, uint8_t mem_mask = ~0);
	uint8_t calendar_data_r(offs_t offset, uint8_t mem_mask = ~0);

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
	static void floppy_formats(format_registration &fr);

	void ansi_disk_attention(ansi_disk_device *disk, bool state);
	void ansi_disk_read_data(ansi_disk_device *disk, uint8_t data);

	void end_of_controller_op();

	DECLARE_WRITE_LINE_MEMBER(fdc_irq);
	uint8_t fdc_msr_r(offs_t, uint8_t mem_mask);

	emu_timer *m_timer;

	devcb_write_line irq_cb;
	devcb_write_line drq_cb;

	TIMER_CALLBACK_MEMBER(trigger_interrupt);

	required_device<msm5832_device> m_rtc;
	required_device<upd765a_device> m_fdc;
	required_device<floppy_connector> m_floppy;
	bool m_floppy_drq_state;

	uint8_t m_wdc_ansi_cmd;
	uint8_t m_wdc_ansi_parm;
	uint8_t m_wdc_sector;
	uint8_t m_wdc_head;
	uint8_t m_wdc_interrupt_control;
	uint8_t m_controller_command;

	uint8_t m_fdc_control;

	void execute_command();
	void execute_ansi_command();

	// our current state
	uint8_t m_controller_status_high;
	uint8_t m_controller_status_low;
	uint8_t m_wdc_selected_head;
	uint8_t m_wdc_selected_drive;
	uint8_t m_wdc_general_status; // this seems wrong - this is the attention status on the controller, not general status
	uint8_t m_wdc_sense_byte_1;
	uint8_t m_wdc_sense_byte_2;
	bool m_wdc_write_enabled;
	bool m_wdc_attention_enabled;

	uint32_t m_cursor;
	char m_buffer[2000]; // really only need 1056 here.

	uint8_t m_wdc_read_data_byte;
	int m_read_record_word_count;

	uint8 m_calendar_ctrl;

	void ansi_index_pulse(ansi_disk_device *);
	void ansi_sector_pulse(ansi_disk_device *);
	int m_pulsed_sector;

	bool m_start_read_sector;
	bool m_start_write_sector;
	void check_for_sector_read();
	void check_for_sector_write();
};

// device type definition
DECLARE_DEVICE_TYPE(APOLLO_DN300_DISK_CTRLR, apollo_dn300_disk_ctrlr_device)

#endif // MAME_MACHINE_APOLLO_DN300_DISK_H
