// license:BSD-3-Clause
// copyright-holders:Hans Ostermeyer, R. Belmont
/*
 *
 *  Created on: April 25, 2013
 *      Author: Hans Ostermeyer
 *              Chris Toshok
 *
 *  see also:
 *  - http://www.bitsavers.org/pdf/apollo/002398-04_Domain_Engineering_Handbook_Rev4_Jan87.pdf (page 12-16 ...)
 *
 */

#include "emu.h"

#include "apollo_dn300.h"

#include "emupal.h"
#include "screen.h"

#include "apollo.lh"
#include "apollo_15i.lh"


#define VIDEO_SCREEN_TAG "screen"

// status register
#define SR_BLT_IN_PROGRESS  0x8000
#define SR_END_OF_FRAME_IRQ 0x0080

// control register
#define CR_GO                  0x8000 // start BLT operation
#define CR_IRQ_AFTER_FRAME     0x0020
#define CR_IRQ_AFTER_BLT       0x0010
#define CR_INC_Y               0x0008
#define CR_INC_X               0x0004
#define CR_FILL_MODE           0x0002
#define CR_ENABLE_DISPLAY      0x0001

void apollo_dn300_graphics::reg_w(offs_t offset, uint16_t data)
{
	static const char * const graphics_reg_write_names[0x08] = {
		"DISPLAY CONTROL",
		"DEB",
		"WSSY",
		"WSSX",
		"DCY",
		"DCX",
		"WSDY",
		"WSDX"
	};

	int reg = offset;

	MLOG2(("writing display reg %02x (%s) with %02x",
			offset, graphics_reg_write_names[reg], data));

	// XXX actually do the write

    switch(reg) {
		case 0: // DISPLAY CONTROL
			m_cr = data;
			if ((m_cr & CR_ENABLE_DISPLAY) != 0) {
				// update screen
				m_update_flag = 1;
			} else {
				m_update_flag = 0;
			}
			if ((m_cr & CR_GO) != 0) {
				blt();
			}
			break;
		case 1: // DEB
			m_deb = data;
			break;
		case 2: // WSSY
			m_wssy = data;
			break;
		case 3: // WSSX
			m_wssx = data;
			break;
		case 4: // DCY
			m_dcy = data;
			break;
		case 5: // DCX
			m_dcx = data;
			break;
		case 6: // WSDY
			m_wsdy = data;
			break;
		case 7: // WSDX
			m_wsdx = data;
			break;
	}
}

uint16_t apollo_dn300_graphics::reg_r(offs_t offset)
{
	static const char * const graphics_reg_read_names[0x08] = {
		"DISPLAY STATUS",
		"DEB",
		"WSSY",
		"WSSX",
		"DCY",
		"DCX",
		"WSDY",
		"WSDX"
	};

	int reg = offset;
    uint16_t value;

    switch(reg) {
		case 0: // DISPLAY STATUS
			value = m_sr;
			break;
		case 1: // DEB
			value = m_deb;
			break;
		case 2: // WSSY
			value = m_wssy;
			break;
		case 3: // WSSX
			value = m_wssx;
			break;
		case 4: // DCY
			value = m_dcy;
			break;
		case 5: // DCX
			value = m_dcx;
			break;
		case 6: // WSDY
			value = m_wsdy;
			break;
		case 7: // WSDX
			value = m_wsdx;
			break;
	}

	MLOG2(("reading display reg %02x (%s) = %02x",
			offset, graphics_reg_read_names[(offset/2) & 0x7], value));

	return value;
}

uint16_t apollo_dn300_graphics::mem_r(offs_t offset, uint16_t mem_mask)
{
	return m_image_memory[offset];
}

void apollo_dn300_graphics::mem_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	MLOG2(("writing display memory %04x = %04x", offset, data&mem_mask));
	m_image_memory[offset] = data & mem_mask;
	m_update_pending = 1;
}

void apollo_dn300_graphics::blt()
{
	m_sr |= SR_BLT_IN_PROGRESS;

	int lines = 1+(int16_t)(~(m_dcy|0xe000));
	int words = 1+(int16_t)(~(m_dcx|0xffc0));

	MLOG2(("should perform blt operation "
	        "CR=%04x (inc_y=%d, inc_x=%d, fill_mode=%d, enable_display=%d) "
			"DEB=%04x "
			"source_y=%04x "
			"source_x=%04x "
			"count_y=%04x (%d lines.  WEDY=%d) "
			"count_x=%04x (%d words.)"
			"dest_y=%04x "
			"dest_x=%04x",
		m_cr,
		(m_cr & CR_INC_Y) ? 1 : 0,
		(m_cr & CR_INC_X) ? 1 : 0,
		(m_cr & CR_FILL_MODE) ? 1 : 0,
		(m_cr & CR_ENABLE_DISPLAY) ? 1 : 0,
		m_deb,
		m_wssy,
		m_wssx,
		m_dcy,
		lines,
		m_wsdy+lines,
		m_dcx,
		words,
		m_wsdy,
		m_wsdx));
	m_sr &= ~SR_BLT_IN_PROGRESS;

	m_update_pending = 1;
}

uint32_t apollo_dn300_graphics::screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
       int has_changed = 0;

       if (m_update_flag && !m_update_pending)
       {
               has_changed = 1;
               m_update_flag = 0;
               m_update_pending = 1;
               screen_update1(bitmap, cliprect);
               m_update_pending = 0;
       }
       return has_changed ? 0 : UPDATE_HAS_NOT_CHANGED;
}

void apollo_dn300_graphics::screen_update1(bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	MLOG1(("in screen_update1"));
	uint16_t const *source_ptr = m_image_memory.get();

	if ((m_cr & CR_ENABLE_DISPLAY) == 0)
	{
		// display is disabled
		for (int y = 0; y < m_height; y++)
		{
			int dest = 0;
			for (int x = 0; x < m_width; x += 16)
			{
				for (uint16_t mask = 0x8000; mask; mask >>= 1)
				{
					bitmap.pix(y, dest++) = 0;
				}
			}
			source_ptr += (m_buffer_width - m_width) / 16;
		}
	}
	else
	{
		for (int y = 0; y < m_height; y++)
		{
			int dest = 0;
			for (int x = 0; x < m_width; x += 16)
			{
				uint16_t const data = *source_ptr++;
				for (uint16_t mask = 0x8000; mask; mask >>= 1)
				{
					bitmap.pix(y, dest++) = data & mask ? 0 : 0x00ffffff;
				}
			}
			source_ptr += (m_buffer_width - m_width) / 16;
		}
	}
}

/*-------------------------------------------------
 vblank_state_changed -
 called on each state change of the VBLANK signal
 -------------------------------------------------*/

void apollo_dn300_graphics::vblank_state_changed(screen_device &screen, bool vblank_state)
{
}

void apollo_dn300_graphics::register_vblank_callback()
{
       MLOG1(("register_vblank_callback"));

       /* register for VBLANK callbacks */
       m_screen->register_vblank_callback(vblank_state_delegate(&apollo_dn300_graphics::vblank_state_changed,this));
}

/***************************************************************************
 MACHINE DRIVERS
 ***************************************************************************/

void apollo_dn300_graphics::device_add_mconfig(machine_config &config)
{
       config.set_default_layout(layout_apollo_15i);
       SCREEN(config, m_screen, SCREEN_TYPE_RASTER);
       m_screen->set_video_attributes(VIDEO_UPDATE_AFTER_VBLANK);
       m_screen->set_raw(68000000, 1346, 0, 1024, 1346, 0, 1024);
       m_screen->set_screen_update(FUNC(apollo_dn300_graphics::screen_update));
}

apollo_dn300_graphics::apollo_dn300_graphics(const machine_config &mconfig,const char *tag, device_t *owner, uint32_t clock) :
       apollo_dn300_graphics(mconfig, APOLLO_DN300_GRAPHICS, tag, owner, clock)
{
}

apollo_dn300_graphics::apollo_dn300_graphics(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock) :
       device_t(mconfig, type, tag, owner, clock),
       m_screen(*this, VIDEO_SCREEN_TAG)
{
}

apollo_dn300_graphics::~apollo_dn300_graphics()
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void apollo_dn300_graphics::device_start()
{
       MLOG1(("apollo_dn300_graphics::device_start"))

	   m_cr = 0;
	   m_sr = 0;
}

//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void apollo_dn300_graphics::device_reset()
{
	MLOG1(("apollo_dn300_graphics::device_reset"));

		m_n_planes = 1;
		m_width = 1024;
		m_height = 1024;
		m_buffer_width = 1024;
		m_buffer_height = 1024;

	if (m_image_memory == nullptr)
	{
		/* allocate the memory image */
		m_image_plane_size = m_buffer_height * m_buffer_width / 16;
		m_image_memory_size = m_image_plane_size * m_n_planes;
		m_image_memory
				= std::make_unique<uint16_t[]>(m_image_memory_size);
		assert(m_image_memory != nullptr);

		MLOG1(("device reset apollo graphics: buffer=%p size=%0x", (void *) m_image_memory.get(), m_image_memory_size));
	}

	memset(m_image_memory.get(), 0, m_image_memory_size * 2);

       //  register_vblank_callback(this);

       /* FIXME: register for VBLANK callbacks */
       register_vblank_callback();
}

DEFINE_DEVICE_TYPE(APOLLO_DN300_GRAPHICS, apollo_dn300_graphics, "apollo_dn300_graphics", "Apollo Screen")

