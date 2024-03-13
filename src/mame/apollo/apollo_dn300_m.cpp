// license:BSD-3-Clause
// copyright-holders:Hans Ostermeyer, R. Belmont, Chris Toshok
/*
* apollo_dn300.cpp - Apollo DN300/DN320/DN360 support
*
*/


#include "emu.h"
#include "apollo_dn300.h"

#include "softlist.h"

//##########################################################################
// machine/apollo_dn300_config.c - APOLLO_DN300 DS3500 configuration
//##########################################################################

#undef VERBOSE
#define VERBOSE 2

static uint16_t config = 0;

/***************************************************************************
 apollo_dn300_config - check configuration setting
 ***************************************************************************/

int apollo_dn300_config(int mask)
{
	return config & mask ? 1 : 0;
}

/***************************************************************************
 Input Ports
 ***************************************************************************/

INPUT_PORTS_START( apollo_dn300_config )
	PORT_START( "apollo_dn300_config" )
		PORT_CONFNAME(APOLLO_DN300_CONF_SERVICE_MODE, 0x00, "Normal/Service" )
		PORT_CONFSETTING(0x01, "Service" )
		PORT_CONFSETTING(0x00, "Normal" )

		PORT_CONFNAME(APOLLO_DN300_CONF_DISPLAY, APOLLO_DN300_CONF_MONO_15I, "Graphics Controller")
		PORT_CONFSETTING(APOLLO_DN300_CONF_MONO_15I, "15\" Monochrome")

		PORT_CONFNAME(APOLLO_DN300_CONF_30_YEARS_AGO, APOLLO_DN300_CONF_30_YEARS_AGO, "30 Years Ago ...")
		PORT_CONFSETTING(0x00, DEF_STR ( Off ) )
		PORT_CONFSETTING(APOLLO_DN300_CONF_30_YEARS_AGO, DEF_STR ( On ) )

		PORT_CONFNAME(APOLLO_DN300_CONF_25_YEARS_AGO, APOLLO_DN300_CONF_25_YEARS_AGO, "25 Years Ago ...")
		PORT_CONFSETTING(0x00, DEF_STR ( Off ) )
		PORT_CONFSETTING(APOLLO_DN300_CONF_25_YEARS_AGO, DEF_STR ( On ) )

		PORT_CONFNAME(APOLLO_DN300_CONF_NODE_ID, APOLLO_DN300_CONF_NODE_ID, "Node ID from Disk")
		PORT_CONFSETTING(0x00, DEF_STR ( Off ) )
		PORT_CONFSETTING(APOLLO_DN300_CONF_NODE_ID, DEF_STR ( On ) )

//      PORT_CONFNAME(APOLLO_DN300_CONF_IDLE_SLEEP, 0x00, "Idle Sleep")
//      PORT_CONFSETTING(0x00, DEF_STR ( Off ) )
//      PORT_CONFSETTING(APOLLO_DN300_CONF_IDLE_SLEEP, DEF_STR ( On ) )
INPUT_PORTS_END

class apollo_dn300_config_device : public device_t
{
public:
	apollo_dn300_config_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	~apollo_dn300_config_device();

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
private:
	// internal state
};

DEFINE_DEVICE_TYPE(APOLLO_DN300_CONF, apollo_dn300_config_device, "apollo_dn300_config", "Apollo_dn300 Configuration")

apollo_dn300_config_device::apollo_dn300_config_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, APOLLO_DN300_CONF, tag, owner, clock)
{
}

apollo_dn300_config_device::~apollo_dn300_config_device()
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void apollo_dn300_config_device::device_start()
{
	MLOG1(("start apollo_dn300_config"));
}

//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void apollo_dn300_config_device::device_reset()
{
	MLOG1(("reset apollo_dn300_config"));
	// load configuration
	config = machine().root_device().ioport("apollo_dn300_config")->read();
}

//##########################################################################
// machine/apollo_dn300_csr.c - APOLLO_DN300 DS3500 CPU Control and Status registers
//##########################################################################

#undef VERBOSE
#define VERBOSE 0

#define CPU_CONTROL_REGISTER_ADDRESS 0x010100

static uint16_t mem_status_register = 0x0000;
static uint8_t mem_control_register = 0x00;

/*-------------------------------------------------
  apollo_dn300_csr_get/set_servicemode
 -------------------------------------------------*/

/*
static int apollo_dn300_csr_get_servicemode()
{
    return cpu_status_register & APOLLO_DN300_CSR_SR_SERVICE ? 0 : 1;
}
static void apollo_dn300_csr_set_servicemode(int mode)
{
	apollo_dn300_csr_set_status_register(1, mode ? APOLLO_DN300_CSR_SR_SERVICE : 0);
}
*/

uint8_t apollo_dn300_mcsr_get_control_register(void)
{
	return mem_control_register;
}

uint16_t apollo_dn300_mcsr_get_status_register(void)
{
	return mem_status_register;
}

void apollo_dn300_csr_set_status_register(uint16_t mask, uint16_t data)
{
	uint16_t new_value = (mem_status_register & ~mask) | (data & mask);

	if (new_value != mem_status_register)
	{
		mem_status_register = new_value;
		//LOG1(("#### setting CPU Status Register with data=%04x & %04x to %04x", data, mask, cpu_status_register));
	}
}

/*-------------------------------------------------
 DN300/DN320 Memory Control Register at 0x8006
 -------------------------------------------------*/

void apollo_dn300_state::apollo_dn300_mcsr_control_register_w(offs_t offset, uint8_t data, uint8_t mem_mask)
{
	int leds;

	COMBINE_DATA(&mem_control_register);

	for (int i = 0; i < 4; i++)
		m_internal_leds[i] = BIT(mem_control_register, 7 - i);

	leds = ((mem_control_register >> 4) & 0xf) ^ 0xf;

	SLOG1(("writing Memory Control Register at offset %X = %04x & %04x (%04x - %d%d%d%d)",
					offset, data, mem_mask, mem_control_register,
					(leds >> 3) & 1,(leds >> 2) & 1, (leds >> 1) & 1, (leds >> 0) & 1));
}

uint8_t apollo_dn300_state::apollo_dn300_mcsr_control_register_r(offs_t offset, uint8_t mem_mask)
{
	SLOG1(("reading Memory Control Register at offset %X = %04x & %04x", offset, mem_control_register, mem_mask));
	return mem_control_register & mem_mask;
}

/*-------------------------------------------------
 DN300/DN320 Memory Status Register at 0x8006
 -------------------------------------------------*/

void apollo_dn300_state::apollo_dn300_mcsr_status_register_w(offs_t offset, uint16_t data, uint16_t mem_mask){
	// To clear parity error condition, write to the status register.
	// This register is readonly aside from this
	mem_status_register &= ~(APOLLO_DN300_MCSR_SR_RIGHT_PARITY_ERROR | APOLLO_DN300_MCSR_SR_LEFT_PARITYU_ERROR);
	SLOG1(("writing Memory Status Register at offset %X = %04x & %04x (%04x)", offset, data, mem_mask, mem_status_register));
}

uint16_t apollo_dn300_state::apollo_dn300_mcsr_status_register_r(offs_t offset, uint16_t mem_mask){
	SLOG1(("reading Memory Status Register at offset %X = %04x & %04x", offset, mem_status_register, mem_mask));
	return mem_status_register & mem_mask;
}

//##########################################################################
// machine/apollo_dn300_rtc.c - APOLLO_DN300 DS3500 RTC MC146818
//##########################################################################

#undef VERBOSE
#define VERBOSE 0

#ifdef notyet
/***************************************************************************
 DN3000/DN3500 Realtime Calendar MC146818 at 0x8900/0x10900
 ***************************************************************************/

void apollo_dn300_state::apollo_rtc_w(offs_t offset, uint8_t data)
{
	m_rtc->write_direct(offset, data);
	if (offset >= 0x0b && offset <= 0x0c)
	{
		SLOG2(("writing MC146818 at offset %02x = %02x", offset, data));
	}
}

uint8_t apollo_dn300_state::apollo_rtc_r(offs_t offset)
{
	uint8_t data;
	data = m_rtc->read_direct(offset);
	if (offset >= 0x0b && offset <= 0x0c)
	{
		SLOG2(("reading MC146818 at offset %02x = %02x", offset, data));
	}
	return data;
}

WRITE_LINE_MEMBER(apollo_dn300_state::apollo_rtc_irq_function)
{
	apollo_dn300_pic_set_irq_line(APOLLO_DN300_IRQ_RTC, state);
}
#endif

//##########################################################################
// machine/apollo_dn300.c - APOLLO_DN300 CPU Board
//##########################################################################

#undef VERBOSE
#define VERBOSE 0

#define KBD_TAG "kbd_ser"
#define KEYBOARD_TAG "keyboard"

void apollo_dn300_state::common(machine_config &config)
{
	// configuration MUST be reset first !
	APOLLO_DN300_CONF(config, APOLLO_DN300_CONF_TAG, 0);

	PTM6840(config, m_ptm, 0);
	m_ptm->set_external_clocks(250000, 125000, 62500);
	m_ptm->irq_callback().set_inputline(MAINCPU, APOLLO_DN300_IRQ_PTM);

	// the cpu board has a 4.9152 MHz oscillator can which passes through 2 /16
	// dividers, resulting in 19200.
	ACIA6850(config, m_acia, 0);
	clock_device &acia_clock(CLOCK(config, "acia_clock", 19200));
	acia_clock.signal_handler().set(m_acia, FUNC(acia6850_device::write_txc));
	acia_clock.signal_handler().append(m_acia, FUNC(acia6850_device::write_rxc));

	// 9000-903F - ring receive header
	// 9040-907F - ring receive data
	// 9080-90BF - ring transmit
	// 90CO-90FF - winchester/floppy
	HD63450(config, m_dmac, 8'000'000, m_maincpu);
	m_dmac->set_clocks(attotime::from_usec(2), attotime::from_nsec(450), attotime::from_usec(4), attotime::from_hz(15625/2));
	m_dmac->set_burst_clocks(attotime::from_usec(2), attotime::from_nsec(450), attotime::from_nsec(450), attotime::from_nsec(50));
	m_dmac->irq_callback().set(FUNC(apollo_dn300_state::dma_irq));
	m_dmac->dma_end().set(FUNC(apollo_dn300_state::dma_end));
	m_dmac->dma_read<APOLLO_DN300_DMA_RING_RCVHEADER>().set(m_ring, FUNC(apollo_dn300_ring_ctrlr_device::rcv_header_read_byte));
	m_dmac->dma_write<APOLLO_DN300_DMA_RING_RCVHEADER>().set(m_ring, FUNC(apollo_dn300_ring_ctrlr_device::rcv_header_write_byte));
	m_dmac->dma_read<APOLLO_DN300_DMA_RING_RCVDATA>().set(m_ring, FUNC(apollo_dn300_ring_ctrlr_device::rcv_data_read_byte));
	m_dmac->dma_write<APOLLO_DN300_DMA_RING_RCVDATA>().set(m_ring, FUNC(apollo_dn300_ring_ctrlr_device::rcv_data_write_byte));
	m_dmac->dma_read<APOLLO_DN300_DMA_RING_XMIT>().set(m_ring, FUNC(apollo_dn300_ring_ctrlr_device::transmit_read_byte));
	m_dmac->dma_write<APOLLO_DN300_DMA_RING_XMIT>().set(m_ring, FUNC(apollo_dn300_ring_ctrlr_device::transmit_write_byte));
	m_dmac->dma_read<APOLLO_DN300_DMA_DISK>().set(m_disk, FUNC(apollo_dn300_disk_ctrlr_device::dma_read_byte));
	m_dmac->dma_write<APOLLO_DN300_DMA_DISK>().set(m_disk, FUNC(apollo_dn300_disk_ctrlr_device::dma_write_byte));
}

void apollo_dn300_state::apollo_dn300(machine_config &config)
{
	common(config);
	SCN2681(config, m_sio, 3.6864_MHz_XTAL);
	// disable the serial interrupt for the time being.  something about the SCN2681 implementation
	// is triggering an interrupt when it shouldn't be.  This shows up as unexpected interrupts
	// in MOUSE and DISK tests.
	//m_sio->irq_cb().set_inputline(MAINCPU, APOLLO_DN300_IRQ_SIO1);

	APOLLO_DN300_KBD(config, m_keyboard, 0);
	m_keyboard->tx_cb().set(m_acia, FUNC(acia6850_device::write_rxd));
	m_acia->txd_handler().set(m_keyboard, FUNC(apollo_dn300_kbd_device::write_txd));
	m_acia->irq_handler().set_inputline(MAINCPU, APOLLO_DN300_IRQ_KBD);
}

void apollo_dn300_state::init_apollo()
{
	MLOG1(("driver_init_apollo_dn300"));
}

MACHINE_START_MEMBER(apollo_dn300_state,apollo_dn300)
{
	MLOG1(("machine_start_apollo_dn300"));

	m_dma_channel = -1;
	m_cur_eop = false;

	m_internal_leds.resolve();
}

MACHINE_RESET_MEMBER(apollo_dn300_state,apollo_dn300)
{
#ifdef notyet
	uint8_t year = apollo_dn300_rtc_r(9);
#endif

	MLOG1(("machine_reset_apollo_dn300"));

#ifdef notyet
	// set configuration
	apollo_dn300_csr_set_servicemode(apollo_dn300_config(APOLLO_DN300_CONF_SERVICE_MODE));

	// change year according to configuration settings
	if (year < 25 && apollo_dn300_config(APOLLO_DN300_CONF_25_YEARS_AGO))
	{
		year += 75;
		apollo_dn300_rtc_w(9, year);
	}
	else if (year < 30 && apollo_dn300_config(APOLLO_DN300_CONF_30_YEARS_AGO))
	{
		year += 70;
		apollo_dn300_rtc_w(9, year);
	}
	else if (year >= 70 && !apollo_dn300_config(APOLLO_DN300_CONF_30_YEARS_AGO)
			&& !apollo_dn300_config(APOLLO_DN300_CONF_25_YEARS_AGO))
	{
		year -= 70;
		apollo_dn300_rtc_w(9, year);
	}
#endif
}
