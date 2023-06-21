// license:BSD-3-Clause
// copyright-holders:Hans Ostermeyer, R. Belmont
/*
 *
 *  Created on: April 25, 2013
 *      Author: Hans Ostermeyer
 *              Chris Toshok
 *              Vladimir Vukicevic
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
#include "apollo_dn300.lh"

// if VERBOSE is set to 3, the display will update after every blt
#undef VERBOSE
#define VERBOSE 0

#define VIDEO_SCREEN_TAG "screen"

#define DISPLAY_IRQ 4

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

// helpers to format a number as binary
static const char* tb1(uint16_t data) { static char buffer[17]; buffer[16] = 0; for (int i = 0; i < 16; i++) { buffer[15 - i] = (data & (1 << i)) ? 'X' : '.'; } return buffer; }
static const char* tb2(uint16_t data) { static char buffer[17]; buffer[16] = 0; for (int i = 0; i < 16; i++) { buffer[15 - i] = (data & (1 << i)) ? 'X' : '.'; } return buffer; }
static const char* tb3(uint16_t data) { static char buffer[17]; buffer[16] = 0; for (int i = 0; i < 16; i++) { buffer[15 - i] = (data & (1 << i)) ? 'X' : '.'; } return buffer; }

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

	//MLOG2(("writing display reg %02x (%s) with %02x", offset, graphics_reg_write_names[reg], data));

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

	//MLOG2(("reading display reg %02x (%s) = %02x", offset, graphics_reg_read_names[(offset >> 1) & 0x7], value));

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
	//MLOG2(("writing display memory %04x = %04x. new value = %04x  %s", offset, data&mem_mask, m_image_memory[offset], tb1(m_image_memory[offset])));
	// m_update_pending = 1;
}

// 64 words per scan line
#define WORD_INDEX(y, x) ((y)*64 + (x)/16)

#define WORD_INDEX_XY(x, y) ((y)*64 + (x)/16)
#define BIT_INDEX_X(x) (15 - (x % 16))

static uint16_t low_bits_mask(uint32_t num_bits)
{
	return (1 << num_bits) - 1;

}

static uint16_t high_bits_mask(uint32_t num_bits)
{
	return ~low_bits_mask(16 - num_bits);
}

void apollo_dn300_graphics::blt()
{
	m_sr |= SR_BLT_IN_PROGRESS;

	int lines = 1 + (int16_t)(~(m_dcy | 0xe000)); // only low 13 bits are used
	int words = 1 + (int16_t)(~(m_dcx | 0xffc0)); // only low  6 bits are used

	// words is the number of words involved in the blt; it is _not_ the number of bits.

	// m_deb is the bit index from the left of the last bit; so
	// to get the number of bits we don't want, we subtract it from 16
	int deb_sub_bits = 16 - (m_deb & 0xf);
	int bits_per_line =
			(16 - (m_wsdx % 16)) // number of bits from the first word
		  + ((words - 1) * 16)   // number of bits from the remaining words
		  - deb_sub_bits         // minus end word bit index
		  + 1;                   // and 1, due to 2s complement shenanigans? I think? Or due to "inclusive" numbering?

	// helper to know if we're displaying a ROM character so that we can skip various logging things
	bool is_rom_char = bits_per_line == 7 && lines == 13 && m_wssx > 500;

	if (VERBOSE > 2 || !is_rom_char) {
		MLOG1(("BLT s=% 4d, % 4d  d=% 4d, % 4d sz=% 4d, % 4d (w: %d) -- "
			"CR=%04x DCX=%04x DCY=%04x DEB=%04X (%s, %s, %s, %s)",
			m_wssx, m_wssy,
			m_wsdx, m_wsdy,
			bits_per_line, lines, words,
			m_cr, m_dcx, m_dcy, m_deb,
			(m_cr & CR_INC_Y) ? "+Y" : "-Y",
			(m_cr & CR_INC_X) ? "+X" : "-X",
			(m_cr & CR_FILL_MODE) ? "clear" : "copy",
			(m_cr & CR_ENABLE_DISPLAY) ? "en" : ""
			));
	}

	bool inc_x = (m_cr & CR_INC_X) != 0;
	bool inc_y = (m_cr & CR_INC_Y) != 0;

	// rearrange so that we always go top to bottom, left to right
	// TODO -- we need to blit in the right direction to handle overlapping regions properly.
	// we can cheat and make a copy of the source buffer if that's the case.
	if (!inc_x) {
		m_wssx -= bits_per_line;
		m_wsdx -= bits_per_line;
	}

	if (!inc_y) {
		m_wssy -= lines;
		m_wsdy -= lines;
	}

	bool clear_mode = (m_cr & CR_FILL_MODE) != 0;

	// some helpers to figure out if we want to log some bits below for debugging
	bool debug_log_bits = true;
	if (clear_mode)
		debug_log_bits = false;
	if (bits_per_line == 7 && lines == 13 && m_wssx > 500)
		debug_log_bits = false;
	if (lines > 20)
		debug_log_bits = false;

	// work top to bottom, left to right
	for (int line = 0; line < lines; line++)
	{
		int source_x = m_wssx;
		int source_y = m_wssy + line;

		int dst_x = m_wsdx;
		int dst_y = m_wsdy + line;

		int num_bits_left = bits_per_line;
		while (num_bits_left > 0) {
			// assume we need to read the sx, sy word no matter what
			uint16_t source_word = clear_mode ? 0 : m_image_memory[WORD_INDEX_XY(source_x, source_y)];
			// the number of bits we're going to copy in this iteration; cap it to
			// bits we can read from a word from source.
			uint16_t num_bits = std::min(num_bits_left, 16 - (source_x % 16));

			// align the high bit of source_word with what we're actually writing,
			// and mask off just the relevant bits
			source_word <<= (source_x % 16);
			source_word &= high_bits_mask(num_bits);

			// number of high order bits we need to skip before we start writing
			uint16_t bit_skip = dst_x % 16;

			// If all the bits we need to write fit within a single destination word, we can take a simpler path.
			// Otherwise, split the write into two parts.

			if (num_bits <= 16 - bit_skip) {
				uint16_t value = m_image_memory[WORD_INDEX_XY(dst_x, dst_y)];

				// zero out the bits we're going to write
				value &= high_bits_mask(bit_skip) | low_bits_mask(16 - bit_skip - num_bits);

				// write our bits in the appropriate place
				value |= source_word >> bit_skip;

				m_image_memory[WORD_INDEX_XY(dst_x, dst_y)] = value;

				if (debug_log_bits && value != 0) {
					MLOG2(("BLT 1 %d [% 4d] bits  % 4d,% 4d -> % 4d,% 4d s: %s d: %s", num_bits, num_bits_left, source_x, source_y, dst_x, dst_y, tb2(source_word), tb1(value)));
				}
			} else {
				// we need to split the write across two words.  I have no idea if it's legal
				// to blit out of bounds, assume it's not and we should crash.
				uint16_t first_word_bits  = 16 - bit_skip;
				uint16_t second_word_bits = num_bits - first_word_bits;

				int dst_word  = WORD_INDEX_XY(dst_x, dst_y);

				// pull out the two dest words
				uint16_t first_word  = m_image_memory[dst_word    ] & high_bits_mask(bit_skip);
				uint16_t second_word = m_image_memory[dst_word + 1] & low_bits_mask(16 - second_word_bits);

				// now shove our bits in there
				first_word  |= source_word >> bit_skip;
				second_word |= source_word << first_word_bits;

				// and write two values
				m_image_memory[dst_word    ] = first_word;
				m_image_memory[dst_word + 1] = second_word;

				if (debug_log_bits && (first_word != 0 || second_word != 0)) {
					MLOG2(("BLT 2 %d [% 4d] bits  % 4d,% 4d -> % 4d,% 4d s: %s d: %s %s", num_bits, num_bits_left, source_x, source_y, dst_x, dst_y, tb3(source_word), tb1(first_word), tb2(second_word)));
				}
			}

			source_x += num_bits;
			dst_x += num_bits;
			num_bits_left -= num_bits;
		}
	}

	m_sr &= ~SR_BLT_IN_PROGRESS;

	if ((m_cr & CR_IRQ_AFTER_BLT) != 0) {
		m_irq_cb(ASSERT_LINE);
	}

	m_update_flag = 1;
	m_update_pending = 0;

	if (VERBOSE > 3)
	{
		machine().video().frame_update(true);
	}
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
	// err.. I assume vblank_state == true means we're in vblank? but apollo_v
	// seems to imply the opposite?
	if (vblank_state) {
	    // I'm not sure if this bit should be set even if interrupts
		// aren't requested, but I don't think it would hurt?
		if ((m_cr & CR_IRQ_AFTER_FRAME) != 0)
		{
			m_sr |= SR_END_OF_FRAME_IRQ;
			m_irq_cb(ASSERT_LINE);
		}
	} else {
		// the tests sometimes fail because this bit gets unset so fast
		// because our vblank period is much faster than the CPU;
		// so it can't actually read that this bit is set when it gets
		// the interrupt.  
		m_sr &= ~SR_END_OF_FRAME_IRQ;
	}
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
	config.set_default_layout(layout_apollo_dn300);
	SCREEN(config, m_screen, SCREEN_TYPE_RASTER);
	m_screen->set_video_attributes(VIDEO_UPDATE_AFTER_VBLANK | VIDEO_ALWAYS_UPDATE);
	m_screen->set_raw(68000000, 1346, 0, 1024, 1346, 0, 1024);
	m_screen->set_screen_update(FUNC(apollo_dn300_graphics::screen_update));
}

apollo_dn300_graphics::apollo_dn300_graphics(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: apollo_dn300_graphics(mconfig, APOLLO_DN300_GRAPHICS, tag, owner, clock)
{
}

apollo_dn300_graphics::apollo_dn300_graphics(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, type, tag, owner, clock)
	, m_screen(*this, VIDEO_SCREEN_TAG)
	, m_irq_cb(*this)
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

	m_irq_cb.resolve_safe();

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
