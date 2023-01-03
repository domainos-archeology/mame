// license:BSD-3-Clause
// copyright-holders:Chris Toshok
/*
 * apollo_dn300_ring.h
 *
 *  Created on: Jan 3, 2023
 *      Author: Chris Toshok
 *
 */

#pragma once

#ifndef MAME_MACHINE_APOLLO_DN300_RING_H
#define MAME_MACHINE_APOLLO_DN300_RING_H

#include "machine/ram.h"
#include "machine/bankdev.h"

class apollo_dn300_ring_device :
    public device_t
{
public:
	//construction/destruction
	apollo_dn300_ring_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

    // configuration
	template <typename T> void set_cpu(T &&cputag)
	{
		m_cpu.set_tag(std::forward<T>(cputag));
	}

	void write(offs_t offset, uint8_t data, uint8_t mem_mask = ~0);
    uint8_t read(offs_t offset, uint8_t mem_mask = ~0);

	uint8_t read_byte(offs_t offset);
	void write_byte(offs_t offset, uint8_t data);
protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

private:
    required_device<cpu_device> m_cpu;
};

// device type definition
DECLARE_DEVICE_TYPE(APOLLO_DN300_RING, apollo_dn300_ring_device)

#endif // MAME_MACHINE_APOLLO_DN300_RING_H
