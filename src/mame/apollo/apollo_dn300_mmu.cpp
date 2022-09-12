
#include "emu.h"

#define VERBOSE 2
#include "apollo_dn300.h"

DEFINE_DEVICE_TYPE(APOLLO_DN300_MMU, apollo_dn300_mmu_device, "apollo_dn300_mmu", "Apollo DN300 Custom MMU")

apollo_dn300_mmu_device::apollo_dn300_mmu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock): 
    device_t(mconfig, APOLLO_DN300_MMU, tag, owner, clock),
    m_cpu(*this, "cpu"),
    m_physical_space(*this, "physical_space"),
    m_enabled(0)
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void apollo_dn300_mmu_device::device_start()
{
    m_pft = std::make_unique<uint16_t[]>(8192);
    save_pointer(NAME(m_pft), 8192);

    m_ptt = std::make_unique<uint16_t[]>(524288);
    save_pointer(NAME(m_ptt), 524288);
}

//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void apollo_dn300_mmu_device::device_reset()
{
}

//-------------------------------------------------
//  device_add_mconfig - add device configuration
//-------------------------------------------------

offs_t apollo_dn300_mmu_device::translate(offs_t offset) {
    // from Domain Engineering Handbook:
    // PAGE TRANSLATION TABLE ENTRY (PTTE)
    // (type "ppn_t" in base.ins.pas)
    // 15                 0
    // +------------------+
    // | XXXXPPPPPPPPPPPP |
    // +------------------+
    // P..P - Physical page number (PPN)
    // XXX  - Junk - ignore
    // Page Translation Table at [ n/a | 700000]
    // through [ n/a | 800000 ]
    // One PPTE every 1024 bytes in table. 
    int ptt_index = offset / 1024;
    int offset_within_page = offset % 1024;
    int ppn = m_ptt[ptt_index] & 0xfff;

    // page size = 1024, so guessing that physical page number starts
    // at PPN = 0, address = 0x000000 and goes up by 1024 per PPN?
    return ppn * 1024 + offset_within_page;
}

void apollo_dn300_mmu_device::write16(offs_t offset, uint16_t data, uint16_t mem_mask)
{
    if (!m_enabled) {
        m_physical_space->write16(offset, data, mem_mask);
        return;
    }

    m_physical_space->write16(translate(offset), data, mem_mask);
}

uint16_t apollo_dn300_mmu_device::read16(offs_t offset, uint16_t mem_mask)
{
    if (!m_enabled) {
        return m_physical_space->read16(offset, mem_mask);
    }

    if (offset >= 0 && offset < 0x4000) {
        // this is the ROM
        return m_physical_space->read16(offset, mem_mask);
    }

    return m_physical_space->read16(translate(offset), mem_mask);
}

void apollo_dn300_mmu_device::ptt_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
    // SLOG1(("writing PTT at offset %02x = %02x & %08x", offset, data, mem_mask));
    m_ptt[offset] = data & mem_mask;
}
uint16_t apollo_dn300_mmu_device::ptt_r(offs_t offset, uint16_t mem_mask)
{
    // SLOG1(("reading PTT at offset %02x & %08x", offset, mem_mask));
    return m_ptt[offset] & mem_mask;
}

void apollo_dn300_mmu_device::pft_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
    // SLOG1(("writing PFT at offset %02x = %02x & %08x", offset, data, mem_mask));
    m_pft[offset] = data & mem_mask;
}
uint16_t apollo_dn300_mmu_device::pft_r(offs_t offset, uint16_t mem_mask)
{
    // SLOG1(("reading PFT at offset %02x & %08x", offset, mem_mask));
    return m_pft[offset] & mem_mask;
}

void apollo_dn300_mmu_device::unk_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
    SLOG1(("UNKNOWN MMU WRITE at offset %02x = %02x & %08x", offset, data, mem_mask));
}

uint16_t apollo_dn300_mmu_device::unk_r(offs_t offset, uint16_t mem_mask)
{
    SLOG1(("UNKNOWN MMU READ at offset %02x & %08x", offset, mem_mask));

    return 0;
}

void apollo_dn300_mmu_device::pid_priv_power_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
    SLOG1(("writing PID/PRIV/POWER at offset %02x = %02x & %08x", offset, data, mem_mask));
    COMBINE_DATA(&m_pid_priv_power);

    m_enabled = m_pid_priv_power & 0x01;
    SLOG1(("mmu enabled?  %s", m_enabled ? "yes" : "no"));
}
uint16_t apollo_dn300_mmu_device::pid_priv_power_r(offs_t offset, uint16_t mem_mask)
{
    return m_pid_priv_power;
}

void apollo_dn300_mmu_device::status_w(offs_t offset, uint8_t data, uint8_t mem_mask)
{
    SLOG1(("MMU STATUS WRITE at offset %02x = %02x & %08x", offset, data, mem_mask));
}

uint8_t apollo_dn300_mmu_device::status_r(offs_t offset, uint8_t mem_mask)
{
    return m_status;  
}
