// license:BSD-3-Clause
// copyright-holders:Chris Toshok
/*
 * apollo_dn300_mmu.h
 *
 *  Created on: Sep 7, 2022
 *      Author: Chris Toshok
 *
 */

#pragma once

#ifndef MAME_MACHINE_APOLLO_DN300_MMU_H
#define MAME_MACHINE_APOLLO_DN300_MMU_H

#include "machine/ram.h"
#include "machine/bankdev.h"

class apollo_dn300_mmu_device :
    public device_t
{
public:
	//construction/destruction
	apollo_dn300_mmu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
    
    // configuration
	template <typename T> void set_cpu(T &&cputag)
	{
		m_cpu.set_tag(std::forward<T>(cputag));
	}

	template <typename T> void set_physical_space(T &&tag)
	{
		m_physical_space.set_tag(std::forward<T>(tag));
	}

	offs_t translate(offs_t offset);

	void write16(offs_t offset, uint16_t data, uint16_t mem_mask = ~0);
    uint16_t read16(offs_t offset, uint16_t mem_mask = ~0);

	void pft_w(offs_t offset, uint16_t data, uint16_t mem_mask = ~0);
	uint16_t pft_r(offs_t offset, uint16_t mem_mask = ~0);

	void ptt_w(offs_t offset, uint16_t data, uint16_t mem_mask = ~0);
	uint16_t ptt_r(offs_t offset, uint16_t mem_mask = ~0);

    void unk_w(offs_t offset, uint16_t data, uint16_t mem_mask = ~0);
	uint16_t unk_r(offs_t offset, uint16_t mem_mask = ~0);

    void pid_priv_power_w(offs_t offset, uint16_t data, uint16_t mem_mask = ~0);
	uint16_t pid_priv_power_r(offs_t offset, uint16_t mem_mask = ~0);

    void status_w(offs_t offset, uint8_t data, uint8_t mem_mask = ~0);
	uint8_t status_r(offs_t offset, uint8_t mem_mask = ~0);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

private:
    required_device<cpu_device> m_cpu;
	required_device<address_map_bank_device> m_physical_space;

    std::unique_ptr<uint16_t[]> m_pft;
	std::unique_ptr<uint16_t[]> m_ptt;
    uint16_t m_pid_priv_power;
    uint16_t m_status;

    uint8_t m_enabled;
	uint8_t m_ptt_access_enabled;
	uint8_t m_asid;
};

// device type definition
DECLARE_DEVICE_TYPE(APOLLO_DN300_MMU, apollo_dn300_mmu_device)

#endif // MAME_MACHINE_APOLLO_DN300_MMU_H