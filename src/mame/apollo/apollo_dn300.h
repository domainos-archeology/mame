// license:BSD-3-Clause
// copyright-holders:Hans Ostermeyer, R. Belmont
/*
 * apollo_dn300.h - APOLLO DN3500/DN3000 driver includes
 *
 *  Created on: May 12, 2010
 *      Author: Hans Ostermeyer
 *
 */

#ifndef MAME_INCLUDES_APOLLO_DN300_H
#define MAME_INCLUDES_APOLLO_DN300_H

#pragma once


#include "apollo_dn300_kbd.h"

#include "cpu/m68000/m68000.h"

#include "machine/6840ptm.h"
#include "machine/hd63450.h"
#include "machine/clock.h"
#include "machine/mc146818.h"
#include "machine/mc68681.h"
#include "machine/6850acia.h"
#include "machine/ram.h"
#include "machine/terminal.h"

#include "bus/rs232/rs232.h"

#include "diserial.h"
#include "screen.h"

#undef VERBOSE
#define VERBOSE 2

#define LOG(x)  { logerror x; logerror ("\n"); apollo_dn300_check_log(); }
#define LOG1(x) { if (VERBOSE > 0) LOG(x) }
#define LOG2(x) { if (VERBOSE > 1) LOG(x) }
#define CLOG(x) { machine().logerror ("%s - %s: ", apollo_dn300_cpu_context(machine()), tag()); machine().logerror x; machine().logerror ("\n"); apollo_dn300_check_log(); }
#define CLOG1(x) { if (VERBOSE > 0) CLOG(x) }
#define CLOG2(x) { if (VERBOSE > 1) CLOG(x) }
#define DLOG(x) { device->logerror ("%s - %s: ", apollo_dn300_cpu_context(device->machine()), device->tag()); device->logerror x; device->logerror ("\n"); apollo_dn300_check_log(); }
#define DLOG1(x) { if (VERBOSE > 0) DLOG(x) }
#define DLOG2(x) { if (VERBOSE > 1) DLOG(x) }
#define MLOG(x)  { machine().logerror ("%s: ", apollo_dn300_cpu_context(machine())); machine().logerror x; machine().logerror ("\n"); apollo_dn300_check_log(); }
#define MLOG1(x) { if (VERBOSE > 0) MLOG(x) }
#define MLOG2(x) { if (VERBOSE > 1) MLOG(x) }
#define SLOG(x)  { machine().logerror ("%s: ", apollo_dn300_cpu_context(machine()));machine().logerror x; machine().logerror ("\n"); apollo_dn300_check_log(); }
#define SLOG1(x) { if (VERBOSE > 0) SLOG(x) }
#define SLOG2(x) { if (VERBOSE > 1) SLOG(x) }

#define  MAINCPU "maincpu"

/*----------- drivers/apollo.cpp -----------*/

// return the current CPU context for log file entries
std::string apollo_dn300_cpu_context(running_machine &machine);

// enable/disable the FPU
void apollo_dn300_set_cpu_has_fpu(m68000_base_device *device, int onoff);

// check for excessive logging
void apollo_dn300_check_log();

// return 1 if node is DN300, 0 otherwise
int apollo_is_dn300(void);

// return 1 if node is DN320, 0 otherwise
int apollo_is_dn320(void);

// return 1 if node is DN330, 0 otherwise
int apollo_is_dn330(void);

// get the ram configuration byte
uint8_t apollo_dn300_get_ram_config_byte(void);

//apollo_dn300_get_node_id - get the node id
uint32_t apollo_dn300_get_node_id(void);

void apollo_dn300_set_cache_status_register(device_t *device,uint8_t mask, uint8_t data);

/*----------- machine/apollo.cpp -----------*/

#define APOLLO_DN300_CONF_TAG "conf"
#define APOLLO_DN300_DMA_TAG "dma63450_1"
#define APOLLO_DN300_STDIO_TAG "stdio"
#define APOLLO_DN300_PTM_TAG  "ptm"
#define APOLLO_DN300_RTC_TAG  "rtc"
#define APOLLO_DN300_SIO_TAG  "sio"
#define APOLLO_DN300_ACIA_TAG  "acia"
#define APOLLO_DN300_NI_TAG  "node_id"
#define APOLLO_DN300_SCREEN_TAG "apollo_dn300_screen"
#define APOLLO_DN300_KBD_TAG  "kbd"


// forward declaration
class apollo_dn300_sio;
class apollo_dn300_ni;
class apollo_dn300_graphics;
class apollo_dn300_kbd_device;

class apollo_dn300_state : public driver_device
{
public:
	apollo_dn300_state(const machine_config &mconfig, device_type type, const char *tag) :
		driver_device(mconfig, type, tag),
		m_maincpu(*this, MAINCPU),
		m_ram(*this, RAM_TAG),
		m_messram_ptr(*this, RAM_TAG),
		m_dma63450(*this, APOLLO_DN300_DMA_TAG),
		// m_pic8259_master(*this, APOLLO_DN300_PIC1_TAG),
		// m_pic8259_slave(*this, APOLLO_DN300_PIC2_TAG),
		m_ptm(*this, APOLLO_DN300_PTM_TAG),
		m_sio(*this, APOLLO_DN300_SIO_TAG),
		m_acia(*this, APOLLO_DN300_ACIA_TAG),
		m_rtc(*this, APOLLO_DN300_RTC_TAG),
		m_node_id(*this, APOLLO_DN300_NI_TAG),
		m_graphics(*this, APOLLO_DN300_SCREEN_TAG),
		m_keyboard(*this, APOLLO_DN300_KBD_TAG),
		m_internal_leds(*this, "internal_led_%u", 1U),
		m_external_leds(*this, "external_led_%c", unsigned('a'))
	{ }

	void dn300(machine_config &config);
	void dn320(machine_config &config);
	void dn330(machine_config &config);

	void init_dn300();
	void init_dn320();
	void init_dn330();
	void init_apollo();

	inline uint16_t getD1() { return m_maincpu->REG_D()[1]; }
protected:
	virtual void machine_start() override;
	virtual void machine_reset() override;

// private:
public:
	required_device<m68000_base_device> m_maincpu;
	required_device<ram_device> m_ram;
	required_shared_ptr<uint32_t> m_messram_ptr;

	required_device<hd63450_device> m_dma63450;
	required_device<ptm6840_device> m_ptm;
	required_device<apollo_dn300_sio> m_sio;
	required_device<acia6850_device> m_acia;
	optional_device<mc146818_device> m_rtc;
	required_device<apollo_dn300_ni> m_node_id;
	required_device<apollo_dn300_graphics> m_graphics;
	optional_device<apollo_dn300_kbd_device> m_keyboard;
	output_finder<4> m_internal_leds;
	output_finder<4> m_external_leds;

	void apollo_pft_w(offs_t offset, uint16_t data, uint16_t mem_mask = ~0);
	uint16_t apollo_pft_r(offs_t offset, uint16_t mem_mask = ~0);

	void apollo_mmu_w(offs_t offset, uint8_t data, uint8_t mem_mask = ~0);
	uint8_t apollo_mmu_r(offs_t offset, uint8_t mem_mask = ~0);

	void apollo_sio_w(offs_t offset, uint16_t data, uint16_t mem_mask = ~0);
	uint16_t apollo_sio_r(offs_t offset, uint16_t mem_mask = ~0);

	void apollo_timers_w(offs_t offset, uint16_t data, uint16_t mem_mask = ~0);
	uint16_t apollo_timers_r(offs_t offset, uint16_t mem_mask = ~0);

	void apollo_dma_ctl_w(offs_t offset, uint16_t data, uint16_t mem_mask = ~0);
	uint16_t apollo_dma_ctl_r(offs_t offset, uint16_t mem_mask = ~0);

	void apollo_display_w(offs_t offset, uint8_t data, uint8_t mem_mask = ~0);
	uint8_t apollo_display_r(offs_t offset, uint8_t mem_mask = ~0);

	void apollo_ring_w(offs_t offset, uint8_t data, uint8_t mem_mask = ~0);
	uint8_t apollo_ring_r(offs_t offset, uint8_t mem_mask = ~0);

	void apollo_disk_w(offs_t offset, uint8_t data, uint8_t mem_mask = ~0);
	uint8_t apollo_disk_r(offs_t offset, uint8_t mem_mask = ~0);

	void apollo_fpu_ctl_w(offs_t offset, uint16_t data, uint16_t mem_mask = ~0);
	uint16_t apollo_fpu_ctl_r(offs_t offset, uint16_t mem_mask = ~0);

	void apollo_fpu_cmd_w(offs_t offset, uint16_t data, uint16_t mem_mask = ~0);
	uint16_t apollo_fpu_cmd_r(offs_t offset, uint16_t mem_mask = ~0);

	void apollo_fpu_cs_w(offs_t offset, uint16_t data, uint16_t mem_mask = ~0);
	uint16_t apollo_fpu_cs_r(offs_t offset, uint16_t mem_mask = ~0);

	void apollo_ptt_w(offs_t offset, uint16_t data, uint16_t mem_mask = ~0);
	uint16_t apollo_ptt_r(offs_t offset, uint16_t mem_mask = ~0);

	void apollo_kbd_control_w(offs_t offset, uint8_t data, uint8_t mem_mask = ~0);
	uint8_t apollo_kbd_status_r(offs_t offset, uint8_t mem_mask = ~0);
	void apollo_kbd_data_w(offs_t offset, uint8_t data, uint8_t mem_mask = ~0);
	uint8_t apollo_kbd_data_r(offs_t offset, uint8_t mem_mask = ~0);

	void apollo_csr_status_register_w(offs_t offset, uint16_t data, uint16_t mem_mask = ~0);
	uint16_t apollo_csr_status_register_r(offs_t offset, uint16_t mem_mask = ~0);
	void apollo_csr_control_register_w(offs_t offset, uint16_t data, uint16_t mem_mask = ~0);
	uint16_t apollo_csr_control_register_r(offs_t offset, uint16_t mem_mask = ~0);
	void apollo_dma_1_w(offs_t offset, uint8_t data);
	uint8_t apollo_dma_1_r(offs_t offset);
	void apollo_dma_2_w(offs_t offset, uint8_t data);
	uint8_t apollo_dma_2_r(offs_t offset);
	void apollo_dma_page_register_w(offs_t offset, uint8_t data);
	uint8_t apollo_dma_page_register_r(offs_t offset);
	void apollo_address_translation_map_w(offs_t offset, uint16_t data);
	uint16_t apollo_address_translation_map_r(offs_t offset);
	uint8_t apollo_dma_read_byte(offs_t offset);
	void apollo_dma_write_byte(offs_t offset, uint8_t data);
	uint8_t apollo_dma_read_word(offs_t offset);
	void apollo_dma_write_word(offs_t offset, uint8_t data);
	void apollo_rtc_w(offs_t offset, uint8_t data);
	uint8_t apollo_rtc_r(offs_t offset);
	void cache_control_register_w(offs_t offset, uint8_t data);
	uint8_t cache_status_register_r(offs_t offset);
	void task_alias_register_w(offs_t offset, uint8_t data);
	uint8_t task_alias_register_r(offs_t offset);
	void latch_page_on_parity_error_register_w(offs_t offset, uint16_t data);
	uint16_t latch_page_on_parity_error_register_r(offs_t offset);
	void master_req_register_w(offs_t offset, uint8_t data);
	uint8_t master_req_register_r(offs_t offset);
	void selective_clear_locations_w(offs_t offset, uint16_t data);
	uint16_t selective_clear_locations_r(offs_t offset);
	uint32_t ram_with_parity_r(offs_t offset, uint32_t mem_mask = ~0);
	void ram_with_parity_w(offs_t offset, uint32_t data, uint32_t mem_mask = ~0);
	uint32_t apollo_unmapped_r(offs_t offset, uint32_t mem_mask = ~0);
	void apollo_unmapped_w(offs_t offset, uint32_t data, uint32_t mem_mask = ~0);
	void apollo_rom_w(offs_t offset, uint32_t data, uint32_t mem_mask = ~0);

	DECLARE_MACHINE_RESET(apollo_dn300);
	DECLARE_MACHINE_START(apollo_dn300);

	void cpu_space_map(address_map &map);
	u16 apollo_irq_acknowledge(offs_t offset);
	u16 apollo_pic_get_vector();
	void apollo_bus_error();
	DECLARE_READ_LINE_MEMBER( apollo_kbd_is_german );
	DECLARE_WRITE_LINE_MEMBER( apollo_dma_1_hrq_changed );
	DECLARE_WRITE_LINE_MEMBER( apollo_dma_2_hrq_changed );
	DECLARE_WRITE_LINE_MEMBER( apollo_pic8259_master_set_int_line );
	DECLARE_WRITE_LINE_MEMBER( apollo_pic8259_slave_set_int_line );
	DECLARE_WRITE_LINE_MEMBER( sio_irq_handler );
	void sio_output(uint8_t data);
	DECLARE_WRITE_LINE_MEMBER( apollo_ptm_irq_function );
	DECLARE_WRITE_LINE_MEMBER( apollo_ptm_timer_tick );
	uint8_t apollo_pic8259_get_slave_ack(offs_t offset);
	DECLARE_WRITE_LINE_MEMBER( apollo_rtc_irq_function );

	uint8_t pc_dma8237_0_dack_r();
	uint8_t pc_dma8237_1_dack_r();
	uint8_t pc_dma8237_2_dack_r();
	uint8_t pc_dma8237_3_dack_r();
	uint8_t pc_dma8237_5_dack_r();
	uint8_t pc_dma8237_6_dack_r();
	uint8_t pc_dma8237_7_dack_r();
	void pc_dma8237_0_dack_w(uint8_t data);
	void pc_dma8237_1_dack_w(uint8_t data);
	void pc_dma8237_2_dack_w(uint8_t data);
	void pc_dma8237_3_dack_w(uint8_t data);
	void pc_dma8237_5_dack_w(uint8_t data);
	void pc_dma8237_6_dack_w(uint8_t data);
	void pc_dma8237_7_dack_w(uint8_t data);
	DECLARE_WRITE_LINE_MEMBER(pc_dack0_w);
	DECLARE_WRITE_LINE_MEMBER(pc_dack1_w);
	DECLARE_WRITE_LINE_MEMBER(pc_dack2_w);
	DECLARE_WRITE_LINE_MEMBER(pc_dack3_w);
	DECLARE_WRITE_LINE_MEMBER(pc_dack4_w);
	DECLARE_WRITE_LINE_MEMBER(pc_dack5_w);
	DECLARE_WRITE_LINE_MEMBER(pc_dack6_w);
	DECLARE_WRITE_LINE_MEMBER(pc_dack7_w);

	void apollo_pic_set_irq_line(int irq, int state);
	void select_dma_channel(int channel, bool state);

	DECLARE_WRITE_LINE_MEMBER(apollo_reset_instr_callback);

	void common(machine_config &config);
	void apollo_dn300(machine_config &config);
	void apollo_dn300_terminal(machine_config &config);

	void dn300_map(address_map &map);
	void dn320_map(address_map &map);
	void dn330_map(address_map &map);

	uint32_t ptm_counter;
	uint8_t sio_output_data;
	int m_dma_channel;
	bool m_cur_eop;
};

/*----------- machine/apollo_dn300_config.cpp -----------*/

// configuration bit definitions

#define APOLLO_DN300_CONF_SERVICE_MODE 0x0001
#define APOLLO_DN300_CONF_DISPLAY      0x001e
#define APOLLO_DN300_CONF_MONO_15I     0x0008
#define APOLLO_DN300_CONF_GERMAN_KBD   0x0020
#define APOLLO_DN300_CONF_30_YEARS_AGO 0x0040
#define APOLLO_DN300_CONF_25_YEARS_AGO 0x0080
#define APOLLO_DN300_CONF_NODE_ID      0x0100
#define APOLLO_DN300_CONF_IDLE_SLEEP   0x0200
#define APOLLO_DN300_CONF_TRAP_TRACE   0x0400
#define APOLLO_DN300_CONF_FPU_TRACE    0x0800
#define APOLLO_DN300_CONF_DISK_TRACE   0x1000
#define APOLLO_DN300_CONF_NET_TRACE    0x2000

// check configuration setting
int apollo_dn300_config(int mask);

INPUT_PORTS_EXTERN(apollo_dn300_config);

/*----------- machine/apollo_dn300_csr.cpp -----------*/

#define APOLLO_DN300_CSR_SR_SERVICE            0x0001
#define APOLLO_DN300_CSR_SR_ATBUS_IO_TIMEOUT   0x0002
#define APOLLO_DN300_CSR_SR_FP_TRAP            0x0004
#define APOLLO_DN300_CSR_SR_INTERRUPT_PENDING  0x0008 // DN3000 only
#define APOLLO_DN300_CSR_SR_PARITY_BYTE_MASK   0x00f0
#define APOLLO_DN300_CSR_SR_CPU_TIMEOUT        0x0100
#define APOLLO_DN300_CSR_SR_ATBUS_MEM_TIMEOUT  0x2000
#define APOLLO_DN300_CSR_SR_BIT15              0x8000
#define APOLLO_DN300_CSR_SR_CLEAR_ALL          0x3ffe

#define APOLLO_DN300_CSR_CR_INTERRUPT_ENABLE   0x0001
#define APOLLO_DN300_CSR_CR_RESET_DEVICES       0x0002
#define APOLLO_DN300_CSR_CR_FPU_TRAP_ENABLE    0x0004
#define APOLLO_DN300_CSR_CR_FORCE_BAD_PARITY   0x0008
#define APOLLO_DN300_CSR_CR_PARITY_BYTE_MASK   0x00f0

uint16_t apollo_dn300_csr_get_control_register(void);
uint16_t apollo_dn300_csr_get_status_register(void);
void apollo_dn300_csr_set_status_register(uint16_t mask, uint16_t data);

/*----------- machine/apollo_dn300_sio.cpp -----------*/

class apollo_dn300_sio: public duart_base_device
{
public:
	apollo_dn300_sio(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual uint8_t read(offs_t offset) override;
	virtual void write(offs_t offset, uint8_t data) override;

protected:
	virtual void device_reset() override;

private:
	uint8_t m_csrb;
};

DECLARE_DEVICE_TYPE(APOLLO_DN300_SIO, apollo_dn300_sio)

/*----------- machine/apollo_dn300_ni.cpp -----------*/


/*** Apollo_dn300 Node ID device ***/

class apollo_dn300_ni: public device_t, public device_image_interface
{
public:
	// construction/destruction
	apollo_dn300_ni(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	virtual ~apollo_dn300_ni();

	// image-level overrides
	virtual bool is_readable()  const noexcept override { return true; }
	virtual bool is_writeable() const noexcept override { return true; }
	virtual bool is_creatable() const noexcept override { return true; }
	virtual bool is_reset_on_load() const noexcept override { return false; }
	virtual bool support_command_line_image_creation() const noexcept override { return true; }
	virtual const char *file_extensions() const noexcept override { return "ani,bin"; }

	virtual image_init_result call_load() override;
	virtual image_init_result call_create(int format_type, util::option_resolution *format_options) override;
	virtual void call_unload() override;
	virtual const char *image_type_name() const noexcept override { return "node_id"; }
	virtual const char *image_brief_type_name() const noexcept override { return "ni"; }

	void write(offs_t offset, uint16_t data, uint16_t mem_mask = ~0);
	uint16_t read(offs_t offset, uint16_t mem_mask = ~0);

	void set_node_id_from_disk();

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	void set_node_id(uint32_t node_id);
	uint32_t m_node_id;
};

// device type definition
DECLARE_DEVICE_TYPE(APOLLO_DN300_NI, apollo_dn300_ni)

/*----------- video/apollo_dn300.cpp -----------*/

class apollo_dn300_graphics : public device_t
{
public:
	apollo_dn300_graphics(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	~apollo_dn300_graphics();


	uint16_t reg_r(offs_t offset);
	void reg_w(offs_t offset, uint16_t data);

	uint16_t mem_r(offs_t offset, uint16_t mem_mask);
	void mem_w(offs_t offset, uint16_t data, uint16_t mem_mask);

    uint32_t screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);

	int is_mono() { return 1; }

protected:
	required_device<screen_device> m_screen;

	apollo_dn300_graphics(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_add_mconfig(machine_config &config) override;

protected:
	void blt();

	void screen_update1(bitmap_rgb32 &bitmap, const rectangle &cliprect);
	void register_vblank_callback();
    void vblank_state_changed(screen_device &screen, bool vblank_state);

protected:

	uint16_t m_n_planes = 0U;
	uint16_t m_width = 0U;
	uint16_t m_height = 0U;
	uint16_t m_buffer_width = 0U;
	uint16_t m_buffer_height = 0U;

	uint16_t m_cr = 0U;
	uint16_t m_sr = 0U;

	uint16_t m_deb = 0U;
	uint16_t m_wssy = 0U;
	uint16_t m_wssx = 0U;
	uint16_t m_dcy = 0U;
	uint16_t m_dcx = 0U;
	uint16_t m_wsdy = 0U;
	uint16_t m_wsdx = 0U;

	uint8_t m_update_flag = 0U;
	uint8_t m_update_pending = 0U;

	std::unique_ptr<uint16_t[]> m_image_memory{};
	int m_image_plane_size = 0;
	int m_image_memory_size = 0;
};

DECLARE_DEVICE_TYPE(APOLLO_DN300_GRAPHICS, apollo_dn300_graphics)

#endif // MAME_INCLUDES_APOLLO_DN300_H
