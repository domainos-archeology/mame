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

#include "machine/ram.h"
#include "machine/bankdev.h"
#include "machine/upd765.h"

class apollo_dn300_disk_device :
    public device_t
{
public:
	//construction/destruction
	apollo_dn300_disk_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

    // configuration
	template <typename T> void set_cpu(T &&cputag)
	{
		m_cpu.set_tag(std::forward<T>(cputag));
	}

	template <typename T> void set_fdc(T &&fdctag)
	{
		m_fdc.set_tag(std::forward<T>(fdctag));
	}

	void wdc_write(offs_t offset, uint8_t data, uint8_t mem_mask = ~0);
    uint8_t wdc_read(offs_t offset, uint8_t mem_mask = ~0);

	void fdc_dsr_w(offs_t offset, uint8_t data, uint8_t mem_mask = ~0);
	uint8_t fdc_msr_r(offs_t offset, uint8_t mem_mask = ~0);

	void fdc_fifo_w(offs_t offset, uint8_t data, uint8_t mem_mask = ~0);
	uint8_t fdc_fifo_r(offs_t offset, uint8_t mem_mask = ~0);

	void calendar_ctrl_w(offs_t offset, uint8_t data, uint8_t mem_mask = ~0);
	void calendar_data_w(offs_t offset, uint8_t data, uint8_t mem_mask = ~0);
	uint8_t calendar_data_r(offs_t offset, uint8_t mem_mask = ~0);

	auto drq_wr_callback() { return drq_cb.bind(); }

	uint8_t read_byte(offs_t offset);
	void write_byte(offs_t offset, uint8_t data);

    void map(address_map &map);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	devcb_write_line drq_cb;

    required_device<cpu_device> m_cpu;
	required_device<upd765a_device> m_fdc;

	uint8_t m_ansi_cmd;
	uint8_t m_ansi_parm;
	uint8_t m_sector;
	uint8_t m_cylinder_high;
	uint8_t m_cylinder_low;
	uint8_t m_head;
	uint8_t m_interrupt_control;
	uint8_t m_controller_command;

	void execute_command();
	void execute_ansi_command();

	// our current state
	uint8_t m_status_high;
	uint8_t m_status_low;
	uint8_t m_selected_head;
	uint8_t m_selected_drive;
	uint8_t m_general_status;
	uint8_t m_sense_byte_1;
	uint8_t m_sense_byte_2;
	bool m_write_enabled;
	bool m_attention_enabled;

	uint32_t m_read_cursor;
	char m_read_buffer[2000];

	uint8 m_calendar_ctrl;
	uint8 m_calendar_data;

	FILE *m_disk_fp;
	FILE *m_sysboot_fp;
};

// device type definition
DECLARE_DEVICE_TYPE(APOLLO_DN300_DISK, apollo_dn300_disk_device)

#endif // MAME_MACHINE_APOLLO_DN300_DISK_H
