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

#undef VERBOSE
#define VERBOSE 0

#define VIDEO_SCREEN_TAG "screen"

// status register
#define SR_BLT_IN_PROGRESS 0x8000
#define SR_END_OF_FRAME_IRQ 0x0080

// control register
#define CR_GO 0x8000 // start BLT operation
#define CR_IRQ_AFTER_FRAME 0x0020
#define CR_IRQ_AFTER_BLT 0x0010
#define CR_INC_Y 0x0008
#define CR_INC_X 0x0004
#define CR_FILL_MODE 0x0002
#define CR_ENABLE_DISPLAY 0x0001

void apollo_dn300_graphics::reg_w(offs_t offset, uint16_t data)
{
	static const char *const graphics_reg_write_names[0x08] = {
		"DISPLAY CONTROL",
		"DEB",
		"WSSY",
		"WSSX",
		"DCY",
		"DCX",
		"WSDY",
		"WSDX"};

	int reg = offset;

	MLOG2(("writing display reg %02x (%s) with %02x",
		   offset, graphics_reg_write_names[reg], data));

	// XXX actually do the write

	switch (reg)
	{
	case 0: // DISPLAY CONTROL
		m_cr = data;
		if ((m_cr & CR_ENABLE_DISPLAY) != 0)
		{
			// update screen
			m_update_flag = 1;
		}
		else
		{
			m_update_flag = 0;
		}
		if ((m_cr & CR_GO) != 0)
		{
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
	static const char *const graphics_reg_read_names[0x08] = {
		"DISPLAY STATUS",
		"DEB",
		"WSSY",
		"WSSX",
		"DCY",
		"DCX",
		"WSDY",
		"WSDX"};

	int reg = offset;
	uint16_t value = 0;

	switch (reg)
	{
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
	default:
		abort();
	}

	MLOG2(("reading display reg %02x (%s) = %02x",
		   offset, graphics_reg_read_names[(offset / 2) & 0x7], value));

	return value;
}

uint16_t apollo_dn300_graphics::mem_r(offs_t offset, uint16_t mem_mask)
{
	return m_image_memory[offset];
}

void apollo_dn300_graphics::mem_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	m_image_memory[offset] &= ~mem_mask;
	m_image_memory[offset] |= data & mem_mask;
	MLOG2(("writing display memory %04x = %04x. new value = %04x", offset, data&mem_mask, m_image_memory[offset]));
	// m_update_pending = 1;
}

// 64 words per scan line
#define WORD_INDEX(y, x) ((y)*64 + x / 16)

#define LOAD_16BITS_AT(y, cursor)                                                                                                       \
	do                                                                                                                                  \
	{                                                                                                                                   \
		if ((cursor) % 16 == 0)                                                                                                         \
		{                                                                                                                               \
			if (log)                                                                                                                    \
			{                                                                                                                           \
				MLOG2((" source is aligned!"))                                                                                          \
			}                                                                                                                           \
			/* we're aligned.  lucky us.  load the whole word. */                                                                       \
			source_word = m_image_memory[WORD_INDEX(y, cursor)];                                                                        \
		}                                                                                                                               \
		else                                                                                                                            \
		{                                                                                                                               \
			if (log)                                                                                                                    \
			{                                                                                                                           \
				MLOG2((" source is not aligned!"))                                                                                      \
			}                                                                                                                           \
			/* we'll need to load two words here. */                                                                                    \
			uint16_t first_word = m_image_memory[WORD_INDEX(y, cursor)];                                                                \
			uint16_t second_word = m_image_memory[WORD_INDEX(y, cursor) + 1];                                                           \
			uint16_t bits_in_first_word = 16 - (cursor) % 16;                                                                           \
			source_word = (first_word << (16 - bits_in_first_word)) | (second_word >> bits_in_first_word);                              \
			if (log)                                                                                                                    \
			{                                                                                                                           \
				MLOG2((" first word = %04x, second word = %04x, bits_in_first_word = %d", first_word, second_word, bits_in_first_word)) \
			}                                                                                                                           \
		}                                                                                                                               \
	} while (0)

void apollo_dn300_graphics::blt()
{
	m_sr |= SR_BLT_IN_PROGRESS;

	int lines = 1 + (int16_t)(~(m_dcy | 0xe000));
	int words = 1 + (int16_t)(~(m_dcx | 0xffc0));

	MLOG2(("should perform blt operation "
		   "CR=%04x (inc_y=%d, inc_x=%d, fill_mode=%d, enable_display=%d) "
		   "DEB=%04x "
		   "source_y=%04x "
		   "source_x=%04x "
		   "count_y=%04x (%d lines.  WEDY=%d) "
		   "count_x=%04x (%d words.) "
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
		   m_wsdy + lines,
		   m_dcx,
		   words,
		   m_wsdy,
		   m_wsdx));

	int log = 0;
	if (
		m_wssy == 0x3e5 && m_wssx == 0x320 // @
		// m_wssy == 0x3d8 && m_wssx == 0x3f2 // >
		//m_cr & CR_FILL_MODE
	)
	{
		log = 1;
	}

	if (m_cr & CR_INC_Y)
	{
		// copy top to bottom
		if (m_cr & CR_INC_X)
		{
			// copy left to right
			for (int line = 0; line < lines; line++)
			{
				int source_cursor = m_wssx;
				for (int word = 0; word < words; word++)
				{
					// express the fetch from the source as a 16 bit mask. we'll let the source fetch read as many
					// words as it needs to fulfill a 16 bit fetch (and hope we don't run off the end of display memory...)
					uint16_t mask = 0xffff;
					int start_bit = 0;

					uint16_t start_mask = 0xffff;
					uint16_t end_mask = 0xffff;

					if (word == 0)
					{
						// first word, figure out how many bits we need from the starting-x

						// for a start_bit = 0x0e, we want our mask to look like: 00000000 00000111
						start_bit = m_wsdx % 16;
						if (start_bit > 0)
						{
							start_mask = ((1 << (16 - start_bit + 1)) - 1);
						}
					}
					if (word == words - 1) // I'm assuming you can have both the starting bit and DEB constraining the same word...
					{
						if (m_deb == 0) {
							end_mask = 0;
						} else {
							// for a DEB of 0x0e (14) we want our mask to look like: 11111111 11111100
							end_mask = ~((0x8000 >> m_deb) - 1);
						}
					}
					// every word but the start/end is easy - it's a full word with no mask.

					mask = start_mask & end_mask;

					if (log)
					{
						MLOG2(("line %d: word %d: start_word mask = %04x, end_word mask = %04x.  mask for dest = %04x",
							   line,
							   word,
							   start_mask,
							   end_mask,
							   mask));
					}

					// get our source word
					uint16_t source_word;
					if (m_cr & CR_FILL_MODE)
					{
						source_word = 0x0000;
					}
					else
					{
						LOAD_16BITS_AT(m_wssy + line, source_cursor);
						source_cursor += 16 - start_bit;
					}

					if (log)
					{
						MLOG2((" source word = %04x", source_word));
						MLOG2((" shifted right by start_bit of %d = %04x", start_bit, source_word >> (start_bit == 0 ? 0 : start_bit - 1)));
						MLOG2((" and after masking: %04x", ((source_word >> (start_bit == 0 ? 0 : start_bit - 1)) & mask)));
						MLOG2((" existing memory word: %04x", m_image_memory[WORD_INDEX(m_wsdy + line, m_wsdx) + word]));
					}
					// at this point the source word contains at the higher order bits the data we need.  we need to make sure we shift it to the right
					// by start_bit so it lands in the right place in the destination word.
					m_image_memory[WORD_INDEX(m_wsdy + line, m_wsdx) + word] &= ~mask;															// clear our the area we're going to blit
					m_image_memory[WORD_INDEX(m_wsdy + line, m_wsdx) + word] |= ((source_word >> (start_bit == 0 ? 0 : start_bit - 1)) & mask); // OR in just the bits we need
					if (log)
					{
						MLOG2((" new memory: %04x", m_image_memory[WORD_INDEX(m_wsdy + line, m_wsdx) + word]));
					}
				}
			}
		}
		else
		{
			// need to copy right to left
			abort();
		}
	}
	else
	{
		// copy bottom to top
		if (m_cr & CR_INC_X)
		{
			// copy left to right
			abort();
		}
		else
		{
			// copy right to left
			abort();
		}
	}

	m_sr &= ~SR_BLT_IN_PROGRESS;

	m_update_flag = 1;
	m_update_pending = 0;
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

	// if ((m_cr & CR_ENABLE_DISPLAY) == 0)
	// {
	// 	// display is disabled
	// 	for (int y = 0; y < m_height; y++)
	// 	{
	// 		int dest = 0;
	// 		for (int x = 0; x < m_width; x += 16)
	// 		{
	// 			for (uint16_t mask = 0x8000; mask; mask >>= 1)
	// 			{
	// 				bitmap.pix(y, dest++) = 0;
	// 			}
	// 		}
	// 		source_ptr += (m_buffer_width - m_width) / 16;
	// 	}
	// }
	// else
	// {
	for (int y = 0; y < m_height; y++)
	{
		int dest = 0;
		for (int x = 0; x < m_width; x += 16)
		{
			uint16_t const data = *source_ptr++;
			for (uint16_t mask = 0x8000; mask; mask >>= 1)
			{
				bitmap.pix(y, dest++) = data & mask ? 0x00ffffff : 0;
			}
		}
		source_ptr += (m_buffer_width - m_width) / 16;
	}
	// }
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
	m_screen->register_vblank_callback(vblank_state_delegate(&apollo_dn300_graphics::vblank_state_changed, this));
}

/***************************************************************************
 MACHINE DRIVERS
 ***************************************************************************/

void apollo_dn300_graphics::device_add_mconfig(machine_config &config)
{
	config.set_default_layout(layout_apollo_15i);
	SCREEN(config, m_screen, SCREEN_TYPE_RASTER);
	m_screen->set_video_attributes(VIDEO_UPDATE_AFTER_VBLANK | VIDEO_ALWAYS_UPDATE);
	m_screen->set_raw(68000000, 1346, 0, 1024, 1346, 0, 1024);
	m_screen->set_screen_update(FUNC(apollo_dn300_graphics::screen_update));
}

apollo_dn300_graphics::apollo_dn300_graphics(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) : apollo_dn300_graphics(mconfig, APOLLO_DN300_GRAPHICS, tag, owner, clock)
{
}

apollo_dn300_graphics::apollo_dn300_graphics(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock) : device_t(mconfig, type, tag, owner, clock),
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
		m_image_memory = std::make_unique<uint16_t[]>(m_image_memory_size);
		assert(m_image_memory != nullptr);

		MLOG1(("device reset apollo graphics: buffer=%p size=%0x", (void *)m_image_memory.get(), m_image_memory_size));
	}

	memset(m_image_memory.get(), 0, m_image_memory_size * 2);

	//  register_vblank_callback(this);

	register_vblank_callback();
}

DEFINE_DEVICE_TYPE(APOLLO_DN300_GRAPHICS, apollo_dn300_graphics, "apollo_dn300_graphics", "Apollo Screen")
