// license:BSD-3-Clause
// copyright-holders:Hans Ostermeyer, R. Belmont
/*
 * apollo.c - APOLLO DN3500/DN3000 driver
 *
 *  Created on: May 12, 2010
 *      Author: Hans Ostermeyer
 *
 *  Adapted February 19, 2012 for general MAME/MESS standards by R. Belmont
 *
 *  TODO: Remove need for instruction hook.
 *        Convert to modern address map.
 *
 *  see also:
 *  - Domain Series 3000/Series 4000 Hardware Architecture Handbook (Order No. 007861 Rev. 02)
 *  - Domain Personal Workstations and Servers Technical Reference (Apollo Order No. 008778-A01)
 *  - Servicing the Domain Personal Workstations and Servers (Apollo Order No. 007859-A01)
 *  - http://www.bitsavers.org/pdf/apollo/002398-04_Domain_Engineering_Handbook_Rev4_Jan87.pdf
 *  - http://www.bitsavers.org/pdf/apollo/008778-03_DOMAIN_Series_3000_4000_Technical_Reference_Aug87.pdf
 *  - http://www.bitsavers.org/pdf/apollo/AEGIS_Internals_and_Data_Structures_Jan86.pdf
 *  - http://www.bitsavers.org/pdf/apollo/019411-A00_Addendum_to_Domain_Personal_Workstations_and_Servers_Hardware_Architecture_Handbook_1991.pdf
 *  - data sheets from Intel and Motorola
 */

#include "emu.h"

#define VERBOSE 2

#include "apollo_dn300.h"

#include "cpu/m68000/m68000.h"
#include "machine/6850acia.h"

#include "debugger.h"

#define TERMINAL_TAG "terminal"

// we use this to prevent excessive logging (if emulation runs amok)
// error.log will be 10 MB for 100000 lines
#define APOLLO_DN300_MAX_NO_OF_LOG_LINES 1000000

#define DN300_RAM_SIZE     1536

#if DN300_RAM_SIZE == 512
#define DN300_RAM_BASE 0x100000
#define DN300_RAM_END  0x17FFFF
#elif DN300_RAM_SIZE == 1024
#define DN300_RAM_BASE 0x100000
#define DN300_RAM_END  0x1FFFFF
#elif DN300_RAM_SIZE == 1536
#define DN300_RAM_BASE 0x100000
#define DN300_RAM_END  0x27FFFF
#endif

#define NODE_TYPE_DN300 300
#define NODE_TYPE_DN320 320

#define DEFAULT_NODE_ID 0x12345

static uint8_t cache_control_register = 0x00;
static uint8_t cache_status_register = 0xff;
static uint8_t task_alias_register = 0x00;

static offs_t parity_error_offset = 0;
static uint16_t parity_error_byte_mask = 0;
// static int parity_error_handler_is_installed = 0;
// static int parity_error_handler_install_counter = 0;

static uint16_t latch_page_on_parity_error_register = 0x0000;

static uint32_t ram_base_address;
static uint32_t ram_end_address;

static int node_type;

// FIXME: value of ram_config_byte must match with default/selected RAM size
static uint8_t ram_config_byte;

static uint32_t log_line_counter = 0;

/***************************************************************************
 cpu_context - return a string describing which CPU is currently executing and their PC
 ***************************************************************************/

std::string apollo_dn300_cpu_context(running_machine &machine) {
	osd_ticks_t t = osd_ticks();
	int s = (t / osd_ticks_per_second()) % 3600;
	int ms = (t / (osd_ticks_per_second() / 1000)) % 1000;

	return util::string_format("%s %d.%03d", machine.describe_context().c_str(), s, ms);
}

/*-------------------------------------------------
 apollo_dn300_set_cpu_has_fpu - enable/disable the FPU
 -------------------------------------------------*/

void apollo_dn300_set_cpu_has_fpu(m68000_base_device *device, int onoff)
{
	if (device == nullptr)
	{
		DLOG1(("set_cpu_has_fpu: unexpected CPU device"));
	}
	else
	{
		device->set_fpu_enable(onoff);
		DLOG1(("apollo_dn300_set_cpu_has_fpu: FPU has been %s", onoff ? "enabled" : "disabled"));
	}
}

/***************************************************************************
 apollo_dn300_check_log - check for excessive logging
 ***************************************************************************/

void apollo_dn300_check_log() {
	if (++log_line_counter >= APOLLO_DN300_MAX_NO_OF_LOG_LINES) {
		// fatalerror("apollo_dn300_check_log: maximum number of log lines exceeded.\n");
	}
}

/***************************************************************************
 apollo_is_dn300 - return 1 if node is DN300, 0 otherwise
 ***************************************************************************/

int apollo_is_dn300(void) {
	return node_type == NODE_TYPE_DN300;
}

/***************************************************************************
 apollo_is_dn320 - return 1 if node is DN320, 0 otherwise
 ***************************************************************************/

int apollo_is_dn320(void) {
	return node_type == NODE_TYPE_DN320;
}

/***************************************************************************
 apollo_get_ram_config_byte - get the ram configuration byte
 ***************************************************************************/

uint8_t apollo_dn300_get_ram_config_byte(void) {
	return ram_config_byte;
}


/***************************************************************************
  instruction_hook
  must be called by the CPU core before executing each instruction
***************************************************************************/
static int instruction_hook(device_t &device, offs_t curpc)
{
	running_machine &machine = device.machine();
	// apollo_dn300_state *state = machine.driver_data<apollo_dn300_state>();
	// address_space      &space = device.memory().space(AS_PROGRAM);
	// uint8_t            *addr_ptr;

	// this is the pointer in the host machine's address space corresponding to
	// curpc.  we don't need it.
	// addr_ptr = (uint8_t*)space.get_read_ptr(curpc);

	// machine.logerror("hello from instruction_hook: %p %x\n", addr_ptr, curpc);

	if (curpc == 0x085a) {
		machine.logerror("HOOK: getc called\n");
	} else if (curpc == 0x0872) {
		machine.logerror("HOOK: pollc called\n");
	} else if (curpc == 0x0844) {
		// machine.logerror("HOOK: _putc_internal called, character = '%c'\n", state->getD1());
	} else if (curpc == 0x2580) {
		machine.logerror("HOOK: diagnostics called\n");
	}

	return 0;
}

/***************************************************************************
 apollo bus error
 ***************************************************************************/

void apollo_dn300_state::apollo_bus_error()
{
	m_maincpu->set_input_line(M68K_LINE_BUSERROR, ASSERT_LINE);
	m_maincpu->set_input_line(M68K_LINE_BUSERROR, CLEAR_LINE);
}

void apollo_dn300_state::cpu_space_map(address_map &map)
{
	//map(0xfffffff2, 0xffffffff).r(FUNC(apollo_dn300_state::apollo_irq_acknowledge));
}

u16 apollo_dn300_state::apollo_irq_acknowledge(offs_t offset)
{
	m_maincpu->set_input_line(offset+1, CLEAR_LINE);

	MLOG2(("apollo_irq_acknowledge: interrupt level=%d", offset+1));

#ifdef notyet
	if (offset+1 == 6)
		return apollo_pic_get_vector();
	else
#endif
		return m68000_base_device::autovector(offset+1);
}

/***************************************************************************
 DN390 Cache Control/Status Register at 0x10200 // FIXME(toshok)
 ***************************************************************************/

void apollo_dn300_state::cache_control_register_w(offs_t offset, uint8_t data){
		cache_control_register = data;
		cache_status_register = (cache_status_register & 0x7f) | (cache_control_register & 0x80);
		SLOG2(("writing Cache Control Register at offset %02x = %02x", offset, data));
}

uint8_t apollo_dn300_state::cache_status_register_r(offs_t offset){
	uint8_t data = cache_status_register;

	SLOG2(("reading Cache Status Register at offset %02x = %02x", offset, data));
	return data;
}

void apollo_dn300_set_cache_status_register(device_t *device,uint8_t mask, uint8_t data) {
	uint16_t new_value = (cache_status_register & ~mask) | (data & mask);
	if (new_value != cache_status_register) {
		cache_status_register = new_value;
		DLOG2(("setting Cache Status Register with data=%02x and mask=%02x to %02x",
				data, mask, cache_status_register));
	}
}

/***************************************************************************
 DN3500 Task Alias Register at 0x10300
 ***************************************************************************/

void apollo_dn300_state::task_alias_register_w(offs_t offset, uint8_t data){
	task_alias_register = data;
	apollo_dn300_set_cache_status_register(this,0x07,  data);
	SLOG(("writing Task Alias Register at offset %02x = %02x",offset, data));
}

uint8_t apollo_dn300_state::task_alias_register_r(offs_t offset){
	uint8_t data = 0xff;
	SLOG(("reading Task Alias Register at offset %02x = %02x", offset, data));
	return data;
}

/***************************************************************************
 DN3000/DN3500 Latch Page on Parity Error Register at 0x9300/0x11300
 ***************************************************************************/

void apollo_dn300_state::latch_page_on_parity_error_register_w(offs_t offset, uint16_t data){
	latch_page_on_parity_error_register = data;
	SLOG1(("writing Latch Page on Error Parity Register at offset %02x = %04x", offset*2, data));
}

uint16_t apollo_dn300_state::latch_page_on_parity_error_register_r(offs_t offset){
	uint16_t data = latch_page_on_parity_error_register;
	SLOG2(("reading Latch Page on Error Parity Register at offset %02x = %04x", offset*2, data));
	return data;
}

/***************************************************************************
 DN3000/DN3500 RAM with parity (and null proc loop delay for DomainOS)
 ***************************************************************************/

uint16_t apollo_dn300_state::ram_with_parity_r(offs_t offset, uint16_t mem_mask){
	uint16_t data = m_messram_ptr[parity_error_offset+offset];

	SLOG2(("memory dword read with parity error at %08x = %08x & %08x parity_byte=%04x",
			ram_base_address + parity_error_offset*4 + offset*4,data, mem_mask, parity_error_byte_mask));

#ifdef notyet
	if (parity_error_byte_mask != 0) {
		latch_page_on_parity_error_register = (ram_base_address + parity_error_offset * 4) >> 10;

		apollo_dn300_csr_set_status_register(APOLLO_DN300_CSR_CR_PARITY_BYTE_MASK,  apollo_dn300_csr_get_status_register() |parity_error_byte_mask);

		if (apollo_dn300_csr_get_control_register() & APOLLO_DN300_CSR_CR_INTERRUPT_ENABLE) {
			// force parity error (if NMI is enabled)
			m_maincpu->set_input_line(7, ASSERT_LINE);

		}
	}
#endif
	return data;
}

void apollo_dn300_state::ram_with_parity_w(offs_t offset, uint16_t data, uint16_t mem_mask){
#ifdef notyet
	COMBINE_DATA(m_messram_ptr+offset);

	if (apollo_dn300_csr_get_control_register() & APOLLO_DN300_CSR_CR_FORCE_BAD_PARITY) {
		parity_error_byte_mask = (apollo_dn300_csr_get_control_register()
				& APOLLO_DN300_CSR_CR_PARITY_BYTE_MASK);

		if (!apollo_is_dn3000()) {
			parity_error_byte_mask ^= APOLLO_DN300_CSR_CR_PARITY_BYTE_MASK;
		}

		parity_error_offset = offset;

//      SLOG1(("memory dword write with parity to %08x = %08x & %08x parity_byte=%04x",
//              ram_base_address +offset * 4, data, mem_mask, parity_error_byte_mask));

		if (parity_error_handler_is_installed == 0) {
			// no more than 192 read/write handlers may be used
			// see table_assign_handler in memory.c
			if (parity_error_handler_install_counter < 40) {
				m_maincpu->space(AS_PROGRAM).install_read_handler(ram_base_address+offset*4, ram_base_address+offset*4+3, read32s_delegate(*this, FUNC(apollo_dn300_state::ram_with_parity_r)));
				parity_error_handler_is_installed = 1;
				parity_error_handler_install_counter++;
			}
		}
	} else if (parity_error_handler_is_installed && offset == parity_error_offset) {
		SLOG1(("memory dword write with parity to %08x = %08x & %08x reset %d",
				ram_base_address +parity_error_offset*4, data, mem_mask, parity_error_handler_install_counter));

		// uninstall not supported, reinstall previous read handler instead

		// memory_install_rom(space, ram_base_address, ram_end_address, messram_ptr.v);
		m_maincpu->space(AS_PROGRAM).install_rom(ram_base_address,ram_end_address,&m_messram_ptr[0]);

		parity_error_handler_is_installed = 0;
		parity_error_byte_mask = 0;
	}
#endif
}

/***************************************************************************
 DN3000/DN3500 unmapped memory
 ***************************************************************************/

uint16_t apollo_dn300_state::apollo_unmapped_r(offs_t offset, uint16_t mem_mask)
{
	offs_t address = offset * 4;

	m68000_base_device *m68k = m_maincpu;

	if ((address & 0xfff00000) == 0xfa800000 && VERBOSE < 2) {
		// ?
	} else if ((address & 0xfff00ff7) == 0xfd800000 && VERBOSE < 2) {
		// omit logging for memory sizing in FPA address space
		// strange: MD seems to search for the 3C505 Boot ROM
		// note (6.10.2010): might be color7 address space (!?!)
	} else if ((address & 0xfc03ffff) == 0x00000000 && VERBOSE < 2) {
		// omit logging for memory sizing in standalone utilities
	} else if (address == 0xfff90000 && VERBOSE < 2) {
		// omit logging for FPA trial access
	} else if (address == 0x00030000 && VERBOSE < 2) {
		// omit logging for Bus error test address in DN3500 boot prom and self_test
	} else if (address == 0x0000ac00 && VERBOSE < 2) {
		// omit logging for Bus error test address in DN3000 boot prom
	} else {
		SLOG1(("unmapped memory dword read from %08x with mask %08x (ir=%04x)", address , mem_mask, m68k->state_int(M68K_IR)));
	}

	/* unmapped; access causes a bus error */
	apollo_bus_error();
	return 0xffff;
}

void apollo_dn300_state::apollo_unmapped_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	SLOG(("unmapped memory dword write to %08x = %04x & %04x", offset * 4, data, mem_mask));

	/* unmapped; access causes a bus error */
	apollo_bus_error();
}

void apollo_dn300_state::mem_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	m_mmu->write16(offset, data, mem_mask);
}
uint16_t apollo_dn300_state::mem_r(offs_t offset, uint16_t mem_mask)
{
	return m_mmu->read16(offset, mem_mask);
}

void apollo_dn300_state::apollo_timers_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	SLOG1(("writing timers at offset %02x = %02x & %08x", offset, data, mem_mask));
	m_ptm->write(offset, data);
}
uint16_t apollo_dn300_state::apollo_timers_r(offs_t offset, uint16_t mem_mask)
{
	SLOG1(("reading timers at offset %02x & %08x", offset, mem_mask));
	return m_ptm->read(offset);
}

void apollo_dn300_state::apollo_dma_ctl_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	SLOG1(("writing DMA CTL at offset %02x = %02x & %08x", offset, data, mem_mask));
	m_dma63450->write(offset, data);
}
uint16_t apollo_dn300_state::apollo_dma_ctl_r(offs_t offset, uint16_t mem_mask)
{
	uint8_t data = m_dma63450->read(offset);
	SLOG1(("reading DMA CTL at offset %02x - %02x & %08x", offset, data, mem_mask));
	return data;
}

void apollo_dn300_state::apollo_display_w(offs_t offset, uint8_t data, uint8_t mem_mask)
{
	SLOG2(("writing display at offset %02x = %02x", offset, data));
}
uint8_t apollo_dn300_state::apollo_display_r(offs_t offset, uint8_t mem_mask)
{
	SLOG1(("reading display at offset %02x & %08x", offset, mem_mask));
	return 0;
}

void apollo_dn300_state::apollo_ring_w(offs_t offset, uint8_t data, uint8_t mem_mask)
{
	SLOG1(("writing ring at offset %02x = %02x & %08x", offset, data, mem_mask));
}
uint8_t apollo_dn300_state::apollo_ring_r(offs_t offset, uint8_t mem_mask)
{
	SLOG1(("reading ring at offset %02x & %08x", offset, mem_mask));
	return 0;
}

void apollo_dn300_state::apollo_disk_w(offs_t offset, uint8_t data, uint8_t mem_mask)
{
	SLOG1(("writing disk at offset %02x = %02x & %08x", offset, data, mem_mask));
}
uint8_t apollo_dn300_state::apollo_disk_r(offs_t offset, uint8_t mem_mask)
{
	SLOG1(("reading disk at offset %02x & %08x", offset, mem_mask));
	return 0;
}

void apollo_dn300_state::apollo_fpu_ctl_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	SLOG1(("writing FPU CTL at offset %02x = %02x & %08x", offset, data, mem_mask));
}
uint16_t apollo_dn300_state::apollo_fpu_ctl_r(offs_t offset, uint16_t mem_mask)
{
	SLOG1(("reading FPU CTL at offset %02x & %08x", offset, mem_mask));
	return 0;
}

void apollo_dn300_state::apollo_fpu_cmd_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	SLOG1(("writing FPU CMD at offset %02x = %02x & %08x", offset, data, mem_mask));
}
uint16_t apollo_dn300_state::apollo_fpu_cmd_r(offs_t offset, uint16_t mem_mask)
{
	SLOG1(("reading FPU CMD at offset %02x & %08x", offset, mem_mask));
	return 0;
}

void apollo_dn300_state::apollo_fpu_cs_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	SLOG1(("writing FPU CS at offset %02x = %02x & %08x", offset, data, mem_mask));
}
uint16_t apollo_dn300_state::apollo_fpu_cs_r(offs_t offset, uint16_t mem_mask)
{
	SLOG1(("reading FPU CS at offset %02x & %08x", offset, mem_mask));
	return 0;
}

/***************************************************************************
 ADDRESS MAPS
 ***************************************************************************/

void apollo_dn300_state::dn300_physical_map(address_map &map)
{
		map(0x000000, 0xffffff).rw(FUNC(apollo_dn300_state::apollo_unmapped_r), FUNC(apollo_dn300_state::apollo_unmapped_w));
		map(0x000000, 0x003fff).rom(); /* boot ROM  */

		map(0x008000, 0x0083ff).rw(m_mmu, FUNC(apollo_dn300_mmu_device::unk_r), FUNC(apollo_dn300_mmu_device::unk_w));
		map(0x008000, 0x008001).rw(m_mmu, FUNC(apollo_dn300_mmu_device::pid_priv_power_r), FUNC(apollo_dn300_mmu_device::pid_priv_power_w));
		map(0x008002, 0x008002).rw(m_mmu, FUNC(apollo_dn300_mmu_device::status_r), FUNC(apollo_dn300_mmu_device::status_w));
		map(0x004000, 0x007fff).rw(m_mmu, FUNC(apollo_dn300_mmu_device::pft_r), FUNC(apollo_dn300_mmu_device::pft_w));
		map(0x700000, 0x7fffff).rw(m_mmu, FUNC(apollo_dn300_mmu_device::ptt_r), FUNC(apollo_dn300_mmu_device::ptt_w));

		// these live in the mmu space but we should have them outside as well...
		map(0x008005, 0x008005).rw(FUNC(apollo_dn300_state::apollo_dn300_mcsr_control_register_r), FUNC(apollo_dn300_state::apollo_dn300_mcsr_control_register_w));
		map(0x008006, 0x008007).rw(FUNC(apollo_dn300_state::apollo_dn300_mcsr_status_register_r), FUNC(apollo_dn300_state::apollo_dn300_mcsr_status_register_w));

		map(0x008400, 0x00841f).rw(m_sio, FUNC(apollo_dn300_sio::read), FUNC(apollo_dn300_sio::write));
		map(0x008420, 0x008421).rw(m_acia, FUNC(acia6850_device::status_r), FUNC(acia6850_device::control_w));
		map(0x008422, 0x008423).rw(m_acia, FUNC(acia6850_device::data_r), FUNC(acia6850_device::data_w));

		map(0x008800, 0x008bff).rw(m_ptm, FUNC(ptm6840_device::read), FUNC(ptm6840_device::write));

		map(0x009000, 0x0093ff).rw(FUNC(apollo_dn300_state::apollo_dma_ctl_r), FUNC(apollo_dn300_state::apollo_dma_ctl_w)); // docs make it seem like this is just 0x9000 - 0x90ff
		map(0x009400, 0x0097ff).rw(FUNC(apollo_dn300_state::apollo_display_r), FUNC(apollo_dn300_state::apollo_display_w)); // docs call this "display 1"

		map(0x009400, 0x00940f).rw(m_graphics, FUNC(apollo_dn300_graphics::reg_r), FUNC(apollo_dn300_graphics::reg_w));

		map(0x009800, 0x009bff).rw(FUNC(apollo_dn300_state::apollo_ring_r), FUNC(apollo_dn300_state::apollo_ring_w)); // docs call this "ring 2"
		map(0x009c00, 0x009fff).rw(FUNC(apollo_dn300_state::apollo_disk_r), FUNC(apollo_dn300_state::apollo_disk_w)); // docs call this "FLP,WIN,CAL"

		map(0x00b000, 0x00b3ff).rw(FUNC(apollo_dn300_state::apollo_fpu_ctl_r), FUNC(apollo_dn300_state::apollo_fpu_ctl_w)); // docs call this "fpu ctl"
		map(0x00b400, 0x00b7ff).rw(FUNC(apollo_dn300_state::apollo_fpu_cmd_r), FUNC(apollo_dn300_state::apollo_fpu_cmd_w)); // docs call this "fpu cmd"
		map(0x00b800, 0x00bbff).rw(FUNC(apollo_dn300_state::apollo_fpu_cs_r), FUNC(apollo_dn300_state::apollo_fpu_cs_w)); // docs call this "fpu cs"

		map(0x020000, 0x03ffff).rw(m_graphics, FUNC(apollo_dn300_graphics::mem_r), FUNC(apollo_dn300_graphics::mem_w)); // docs call this "disp1 mem"

		// map(0x100000, 0x17ffff).rw(/* MD stack / data */),


		// map(DN300_RAM_BASE, DN300_RAM_END).ram().w(FUNC(apollo_dn300_state::ram_with_parity_w)).share(RAM_TAG);
		map(DN300_RAM_BASE, DN300_RAM_END).ram().share(RAM_TAG);
}

void apollo_dn300_state::dn300_mem(address_map &map)
{
	// 24-bit virtual addresses
	map(0x000000, 0xffffff).rw(FUNC(apollo_dn300_state::mem_r), FUNC(apollo_dn300_state::mem_w));
	map(0x000000, 0x003fff).rom(); /* boot ROM  */
}

/***************************************************************************
 Machine Reset
 ***************************************************************************/

void apollo_dn300_state::machine_reset()
{
	MLOG1(("machine_reset"));

	MACHINE_RESET_CALL_MEMBER(apollo_dn300);

	if (apollo_dn300_config(APOLLO_DN300_CONF_NODE_ID))
	{
		// set node ID from UID of logical volume 1 of logical unit 0
		m_node_id->set_node_id_from_disk();
	}

	if (machine().debug_flags & DEBUG_FLAG_ENABLED) {
		m_maincpu->debug()->set_instruction_hook(instruction_hook);
	}
}

WRITE_LINE_MEMBER(apollo_dn300_state::apollo_reset_instr_callback)
{
	MLOG1(("apollo_reset_instr_callback"));

	// reset the CPU board devices
	MACHINE_RESET_CALL_MEMBER(apollo_dn300);

	m_graphics->reset();
	m_keyboard->reset();
}

/***************************************************************************
 Machine Start
 ***************************************************************************/

void apollo_dn300_state::machine_start(){
	memory_share *messram = memshare(RAM_TAG);
	MLOG1(("machine_start_dn300: ram size is %g MB", (float)messram->bytes()/(1024*1024)));

	// clear ram
	memset(messram->ptr(), 0x00, messram->bytes());

	MACHINE_START_CALL_MEMBER(apollo_dn300);
}

/***************************************************************************
 Driver Init
 ***************************************************************************/

void apollo_dn300_state::init_dn300()
{
//  MLOG1(("driver_init_dn3500"));

	/* hook the RESET line, which resets a slew of other components */
	m_maincpu->set_reset_callback(*this, FUNC(apollo_dn300_state::apollo_reset_instr_callback));

	ram_base_address = DN300_RAM_BASE;
	ram_end_address = DN300_RAM_END;

	node_type=  NODE_TYPE_DN300;
	// ram_config_byte= DN300_RAM_CONFIG_BYTE;

	init_apollo();
}

void apollo_dn300_state::init_dn320()
{
	init_dn300();
    MLOG1(("driver_init_dn320"));

	ram_base_address = DN300_RAM_BASE;
	ram_end_address = DN300_RAM_END;

	node_type = NODE_TYPE_DN300;
	// ram_config_byte= DN3000_RAM_CONFIG_8MB;
}

/***************************************************************************
 Input Ports
 ***************************************************************************/

static INPUT_PORTS_START( dn300 )
	PORT_INCLUDE(apollo_dn300_config)
INPUT_PORTS_END

/***************************************************************************
 MACHINE DRIVERS
 ***************************************************************************/

void apollo_dn300_state::dn300(machine_config &config)
{
	/* basic machine hardware */
	M68010(config, m_maincpu, 16000000); /* 16 MHz 68010 */
	m_maincpu->set_addrmap(AS_PROGRAM, &apollo_dn300_state::dn300_mem);
	// m_maincpu->set_addrmap(m68000_base_device::AS_CPU_SPACE, &apollo_dn300_state::cpu_space_map);

	config.set_maximum_quantum(attotime::from_hz(60));

	apollo_dn300(config);
	APOLLO_DN300_GRAPHICS(config, m_graphics, 0);

	APOLLO_DN300_MMU(config, m_mmu, 0);
	m_mmu->set_cpu(m_maincpu);
	m_mmu->set_physical_space(m_physical_space);

	/* internal ram */
	RAM(config, m_ram).set_default_size("1536K").set_extra_options("512K,1M,1536K");

	ADDRESS_MAP_BANK(config, "physical_space").set_map(&apollo_dn300_state::dn300_physical_map).set_options(ENDIANNESS_BIG, 16, 24);
}

void apollo_dn300_state::dn320(machine_config &config)
{
	/* basic machine hardware */
	M68010(config, m_maincpu, 16000000); /* 16 MHz 68010 */
	m_maincpu->set_addrmap(AS_PROGRAM, &apollo_dn300_state::dn300_mem);
	// m_maincpu->set_addrmap(m68000_base_device::AS_CPU_SPACE, &apollo_dn300_state::cpu_space_map);

	config.set_maximum_quantum(attotime::from_hz(60));

	apollo_dn300(config);
	APOLLO_DN300_GRAPHICS(config, m_graphics, 0);

	APOLLO_DN300_MMU(config, m_mmu, 0);
	m_mmu->set_cpu(m_maincpu);
	m_mmu->set_physical_space(m_physical_space);

	/* internal ram */
	RAM(config, m_ram).set_default_size("1536K").set_extra_options("512K,1M,1536K");

	ADDRESS_MAP_BANK(config, "physical_space").set_map(&apollo_dn300_state::dn300_physical_map).set_options(ENDIANNESS_BIG, 16, 24);
}

/***************************************************************************
 ROM Definitions
 ***************************************************************************/

ROM_START( dn300 )
	// dn300 boot ROM (Note: use sha1sum -b <rom file>)
	ROM_REGION( 0x04000, MAINCPU, 0 ) /* 68000 code */

	ROM_SYSTEM_BIOS( 0, "md-rev-5", "MD REV 5, 1983/08/31" )
	ROMX_LOAD( "300_boot.bin", 0x00000, 0x04000, CRC(0b8d94e7) SHA1(38085f10590f285f8378dfca4fd13d18fa476a74) , ROM_BIOS(0) )
ROM_END

ROM_START( dn320 )
	// dn300 boot ROM (Note: use sha1sum -b <rom file>)
	ROM_REGION( 0x04000, MAINCPU, 0 ) /* 68000 code */

	ROM_SYSTEM_BIOS( 0, "md-rev-5", "MD REV 5, 1983/08/31" )
	ROMX_LOAD( "300_boot.bin", 0x00000, 0x04000, CRC(0b8d94e7) SHA1(38085f10590f285f8378dfca4fd13d18fa476a74) , ROM_BIOS(0) )
ROM_END

/***************************************************************************
    GAME DRIVERS
 ***************************************************************************/

#define DN_FLAGS MACHINE_NO_SOUND

/*    YEAR  NAME       PARENT  COMPAT  MACHINE  INPUT   CLASS               INIT         COMPANY   FULLNAME         FLAGS */
COMP( 1983, dn300,     0,      0,      dn300,   dn300,  apollo_dn300_state, init_dn300,  "Apollo", "Apollo DN300",  DN_FLAGS )
COMP( 1984, dn320,     dn300,  0,      dn320,   dn300,  apollo_dn300_state, init_dn300,  "Apollo", "Apollo DN320",  DN_FLAGS )
