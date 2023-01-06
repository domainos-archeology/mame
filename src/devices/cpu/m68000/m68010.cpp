// license:BSD-3-Clause
// copyright-holders:Karl Stenerud

#include "emu.h"
#include "m68010.h"
#include "m68kdasm.h"

DEFINE_DEVICE_TYPE(M68010,      m68010_device,      "m68010",       "Motorola MC68010")
DEFINE_DEVICE_TYPE(M68010EMMU,  m68010emmu_device,  "m68010emmu",   "Motorola MC68010EMMU")

std::unique_ptr<util::disasm_interface> m68010_device::create_disassembler()
{
	return std::make_unique<m68k_disassembler>(m68k_disassembler::TYPE_68010);
}

std::unique_ptr<util::disasm_interface> m68010emmu_device::create_disassembler()
{
	return std::make_unique<m68k_disassembler>(m68k_disassembler::TYPE_68010);
}


m68010_device::m68010_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: m68000_musashi_device(mconfig, tag, owner, clock, M68010, 16,24)
{
}

void m68010_device::device_start()
{
	m68000_musashi_device::device_start();
	init_cpu_m68010();
}


bool m68010emmu_device::memory_translate(int space, int intention, offs_t &address)
{
	/* only applies to the program address space and only does something if the MMU's enabled */
	{
		if ((space == AS_PROGRAM) && (m_emmu_enabled))
		{
			address = emmu_translate_addr(address);
		}
	}
	return true;
}


m68010emmu_device::m68010emmu_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: m68000_musashi_device(mconfig, tag, owner, clock, M68010EMMU, 16,24)
{
}

void m68010emmu_device::device_start()
{
	m68000_musashi_device::device_start();
	init_cpu_m68010emmu();
}
