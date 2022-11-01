
#include "emu.h"

#define VERBOSE 2
#include "apollo_dn300.h"

DEFINE_DEVICE_TYPE(APOLLO_DN300_MMU, apollo_dn300_mmu_device, "apollo_dn300_mmu", "Apollo DN300 Custom MMU")

apollo_dn300_mmu_device::apollo_dn300_mmu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock):
    device_t(mconfig, APOLLO_DN300_MMU, tag, owner, clock),
    m_cpu(*this, "cpu"),
    m_physical_space(*this, "physical_space"),
    m_enabled(0),
    m_ptt_access_enabled(0),
    m_asid(0)
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void apollo_dn300_mmu_device::device_start()
{
    m_pft = std::make_unique<uint16_t[]>(8192);
    save_pointer(NAME(m_pft), 8192);

    m_ptt = std::make_unique<uint16_t[]>(1024);
    save_pointer(NAME(m_ptt), 1024);
}

//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void apollo_dn300_mmu_device::device_reset()
{
    SLOG1(("RESET of the mmu"));
    m_enabled = 0;
    m_ptt_access_enabled = 0;
    m_asid = 0;

    m_pft = std::make_unique<uint16_t[]>(8192);
    save_pointer(NAME(m_pft), 8192);

    m_ptt = std::make_unique<uint16_t[]>(1024);
    save_pointer(NAME(m_ptt), 1024);
}

typedef struct {

    uint link: 12;    // PFT hash thread
    uint global: 1;   // 1 = Page is global
    uint used: 1;     // 1 = Page is referenced
    uint bbmod: 1;    // 1 = Page is modified
    uint eoc: 1;      // 1 = this PFTE is the end of the hash thread chain
    uint xsvpn: 4;    // 1 = Excess virtual page number
    uint x: 1;        // Execute access
    uint r: 1;        // Read access
    uint w: 1;        // Write access
    uint domain: 1;   // DOMAIN (0 or 1).. whatever that means
    uint elaccess: 1; // Supervisor domain
    uint elsid: 7;    // Address space ID (0-127)

} pfte_s;

typedef union {
    struct {
        uint16_t high;
        uint16_t low;
    } v;
    pfte_s pfte;
} pfte;

//-------------------------------------------------
//  device_add_mconfig - add device configuration
//-------------------------------------------------

#define NUM_PTTE 1024
#define PAGE_SIZE 1024

offs_t apollo_dn300_mmu_device::translate(offs_t byte_offset) {
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
    int ptt_index = (byte_offset >> 10) % NUM_PTTE;
    int byte_offset_within_page = byte_offset % PAGE_SIZE;
    int ppn = m_ptt[ptt_index] & 0xfff;

	// MLOG1((
	//  "apollo_dn300_mmu_device::translate(%x) -> PTT index %d, PTTE %x, PPN %x, translated %x",
	//  byte_offset,
	//  ptt_index,
	//  m_ptt[ptt_index],
	//  ppn,
	//  (ppn << 10) | byte_offset_within_page));

	// we need to do more here with the PFT entry for this page, but for now, just return the physical address
	return (ppn << 10) | byte_offset_within_page;
}

void apollo_dn300_mmu_device::write16(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	offs_t write_offs = m_enabled ? translate(offset*2)/2 : offset;
    m_physical_space->write16(write_offs, data, mem_mask);
}

uint16_t apollo_dn300_mmu_device::read16(offs_t offset, uint16_t mem_mask)
{
	offs_t read_offs = m_enabled ? translate(offset*2)/2 : offset;
    return m_physical_space->read16(read_offs, mem_mask);
}

void apollo_dn300_mmu_device::ptt_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
    if (!m_ptt_access_enabled) {
        return;
    }

	// values exist only on even 1024 byte multiples, so 512 word multiples
	offs_t ptt_offset = offset;
	if (ptt_offset % 512 != 0) {
		return;
	}

    // SLOG1(("writing PTT at offset %02x = %02x & %08x", offset, data, mem_mask));
    m_ptt[offset/512] = data & mem_mask;
}

uint16_t apollo_dn300_mmu_device::ptt_r(offs_t offset, uint16_t mem_mask)
{
	// values exist only on even 1024 byte multiples, so 512 word multiples
	offs_t ptt_offset = offset;
	if (ptt_offset % 512 != 0) {
		return 0;
	}

    // SLOG1(("reading PTT at offset %02x & %08x", offset, mem_mask));
    return m_ptt[offset/512] & mem_mask;
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
    m_ptt_access_enabled = m_pid_priv_power & 0x02;
    m_asid = m_pid_priv_power >> 8;

    SLOG1(("mmu enabled?  %s  ptt access?  %s",
            m_enabled ? "yes" : "no",
            m_ptt_access_enabled ? "yes" : "no"));
    SLOG1(("asid %d", m_asid));
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
