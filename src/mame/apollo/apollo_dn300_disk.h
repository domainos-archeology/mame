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

	template <typename T> void set_physical_space(T &&tag)
	{
		m_physical_space.set_tag(std::forward<T>(tag));
	}

	void write(offs_t offset, uint8_t data, uint8_t mem_mask = ~0);
    uint8_t read(offs_t offset, uint8_t mem_mask = ~0);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

private:
    required_device<cpu_device> m_cpu;
	required_device<address_map_bank_device> m_physical_space;

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
	uint8_t m_attention_status;
	bool m_write_enabled;
	uint8_t m_selected_head;
	uint8_t m_selected_drive;
};

// device type definition
DECLARE_DEVICE_TYPE(APOLLO_DN300_DISK, apollo_dn300_disk_device)

#endif // MAME_MACHINE_APOLLO_DN300_DISK_H
