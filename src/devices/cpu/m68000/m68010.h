// license:BSD-3-Clause
// copyright-holders:Karl Stenerud
#ifndef MAME_CPU_M68000_M68010_H
#define MAME_CPU_M68000_M68010_H

#pragma once

#include "m68kmusashi.h"

class m68010_device : public m68000_musashi_device
{
public:
	// construction/destruction
	m68010_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	virtual std::unique_ptr<util::disasm_interface> create_disassembler() override;

	virtual u32 execute_min_cycles() const noexcept override { return 4; }
	virtual u32 execute_max_cycles() const noexcept override { return 158; }

	// device-level overrides
	virtual void device_start() override ATTR_COLD;
};

class m68010emmu_device : public m68000_musashi_device
{
public:
	// construction/destruction
	m68010emmu_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	virtual std::unique_ptr<util::disasm_interface> create_disassembler() override;

	virtual u32 execute_min_cycles() const noexcept override { return 4; }
	virtual u32 execute_max_cycles() const noexcept override { return 158; }

	virtual bool memory_translate(int space, int intention, offs_t &address, address_space *&target_space) override;

	// device-level overrides
	virtual void device_start() override;
};


DECLARE_DEVICE_TYPE(M68010, m68010_device)
DECLARE_DEVICE_TYPE(M68010EMMU, m68010emmu_device)

#endif
