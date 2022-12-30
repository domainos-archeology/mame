
#include "emu.h"

#define VERBOSE 2
#include "apollo_dn300.h"

DEFINE_DEVICE_TYPE(APOLLO_DN300_MMU, apollo_dn300_mmu_device, "apollo_dn300_mmu", "Apollo DN300 Custom MMU")

apollo_dn300_mmu_device::apollo_dn300_mmu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock):
    device_t(mconfig, APOLLO_DN300_MMU, tag, owner, clock),
    m_cpu(*this, "cpu"),
    m_status(0),
    m_enabled(0),
    m_asid(0),
    m_dump_translations(false)
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
    m_asid = 0;

    m_pft = std::make_unique<uint16_t[]>(8192);
    save_pointer(NAME(m_pft), 8192);

    m_ptt = std::make_unique<uint16_t[]>(1024);
    save_pointer(NAME(m_ptt), 1024);
}

typedef struct {

    uint elsid: 7;    // Address space ID (0-127)
    uint elaccess: 1; // Supervisor domain
    uint domain: 1;   // DOMAIN (0 or 1).. whatever that means
    uint w: 1;        // Write access
    uint r: 1;        // Read access
    uint x: 1;        // Execute access
    uint xsvpn: 4;    // 1 = Excess virtual page number
    uint eoc: 1;      // 1 = this PFTE is the end of the hash thread chain
    uint bbmod: 1;    // 1 = Page is modified
    uint used: 1;     // 1 = Page is referenced
    uint global: 1;   // 1 = Page is global
    uint link: 12;    // PFT hash thread

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
    int vpn = byte_offset >> 10;
    int ptt_index = vpn % NUM_PTTE;
    int xsvpn = vpn / NUM_PTTE;

    int byte_offset_within_page = byte_offset % PAGE_SIZE;
    int ppn = m_ptt[ptt_index] & 0xfff;

    if (m_dump_translations) {
        MLOG1((
            "apollo_dn300_mmu_device::translate(%x) -> PTT index %d, PTTE %x, PPN %d, XSVPN %d",
            byte_offset, ptt_index, m_ptt[ptt_index], ppn, xsvpn
        ));
    }


    int cur_ppn = ppn;
    bool seen_eoc = false;
    while (true) {
        uint32_t pfte = m_pft[cur_ppn * 2] << 16 | m_pft[cur_ppn * 2 + 1];

        int pfte_link =   (pfte >> 0)  & 0xfff;
        // int pfte_global = (pfte >> 12) & 0x1;
        int pfte_eoc =    (pfte >> 15) & 0x1;
        int pfte_xsvpn =  (pfte >> 16) & 0xf;

        if (pfte_eoc) {
            if (seen_eoc) {
                // if we're at the eoc twice, we've looped and didn't find a matching xsvpn.
                // this should be a bus error.
                break;
            }
            seen_eoc = true;
        }

        if (m_dump_translations) {
            MLOG1(("    pft_entry for ppn %d, xsvpn %d\n",  cur_ppn, pfte_xsvpn));
        }

        if (pfte_xsvpn == xsvpn) {
            // we found a match.
            if (m_dump_translations) {
                MLOG1(("    found a match\n"));
            }
            // update the PTT to point to this PFT entry first so we'll hit it faster next time
            m_ptt[ptt_index] = cur_ppn;

            // return the translated address
            return (cur_ppn << 10) | byte_offset_within_page;
        }

        cur_ppn = pfte_link;
    }

	m_cpu->set_input_line(M68K_LINE_BUSERROR, ASSERT_LINE);
	m_cpu->set_input_line(M68K_LINE_BUSERROR, CLEAR_LINE);
	return 0;
}

void apollo_dn300_mmu_device::ptt_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
    if (!(m_status & 0x02)) {
        // ptt access disabled
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
    m_asid = m_pid_priv_power >> 8;

    bool ptt_access_enabled = m_pid_priv_power & 0x02;
    SLOG1(("mmu enabled? %s.  ptt access? %s.",
            m_enabled ? "yes" : "no",
            ptt_access_enabled ? "yes" : "no"));
    SLOG1(("asid %d", m_asid));

    if (ptt_access_enabled) {
        m_status |= 0x02;
    } else {
        m_status &= ~0x02;
    }
    m_cpu->set_emmu_enable(m_enabled);
}
uint16_t apollo_dn300_mmu_device::pid_priv_power_r(offs_t offset, uint16_t mem_mask)
{
    return m_pid_priv_power;
}

void apollo_dn300_mmu_device::status_w(offs_t offset, uint8_t data, uint8_t mem_mask)
{
    SLOG1(("MMU STATUS WRITE at offset %02x = %02x & %08x", offset, data, mem_mask));
    // EH87 says any write to register clears bits 5-7.
    m_status &= ~(0xe0);
}

uint8_t apollo_dn300_mmu_device::status_r(offs_t offset, uint8_t mem_mask)
{
    return m_status;
}

void apollo_dn300_mmu_device::fpu_owner_w(offs_t offset, uint8_t data, uint8_t mem_mask)
{
    SLOG1(("MMU FPU OWNER WRITE at offset %02x = %02x & %08x", offset, data, mem_mask));
}

uint8_t apollo_dn300_mmu_device::fpu_owner_r(offs_t offset, uint8_t mem_mask)
{
    return m_fpu_owner;
}


void apollo_dn300_mmu_device::dump_tables()
{
    SLOG1(("PTT:"));
    for (int i = 0; i < 1024; i++) {
        uint32_t addr = 0x700000 + i * 1024;
        SLOG1(("PTTE %d @ %08x: %04x", i, addr, m_ptt[i]));
    }

    SLOG1(("PFT:"));
    for (int i = 0; i < 4096; i++) {
        SLOG1(("PFTE %d: %04x%04x", i, m_pft[i*2], m_pft[i*2+1]));
    }
}
