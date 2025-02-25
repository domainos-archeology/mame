// license:BSD-3-Clause
// copyright-holders:Manuel Abadia
#include "emu.h"
#include "xorworld.h"


/***************************************************************************

  Convert the color PROMs into a more useable format.

  Xor World has three 256x4 palette PROMs (one per gun).
  The palette PROMs are connected to the RGB output this way:

  bit 3 -- 220 ohm resistor  -- RED/GREEN/BLUE
        -- 460 ohm resistor  -- RED/GREEN/BLUE
        -- 1  kohm resistor  -- RED/GREEN/BLUE
  bit 0 -- 2.2kohm resistor  -- RED/GREEN/BLUE

***************************************************************************/

void xorworld_state::xorworld_palette(palette_device &palette) const
{
	const uint8_t *color_prom = memregion("proms")->base();

	for (int i = 0; i < palette.entries(); i++)
	{
		int bit0, bit1, bit2, bit3;

		// red component
		bit0 = BIT(color_prom[0], 0);
		bit1 = BIT(color_prom[0], 1);
		bit2 = BIT(color_prom[0], 2);
		bit3 = BIT(color_prom[0], 3);
		int const r = 0x0e*bit0 + 0x1e*bit1 + 0x44*bit2 + 0x8f*bit3;
		// green component
		bit0 = BIT(color_prom[palette.entries()], 0);
		bit1 = BIT(color_prom[palette.entries()], 1);
		bit2 = BIT(color_prom[palette.entries()], 2);
		bit3 = BIT(color_prom[palette.entries()], 3);
		int const g = 0x0e*bit0 + 0x1e*bit1 + 0x44*bit2 + 0x8f*bit3;
		// blue component
		bit0 = BIT(color_prom[2 * palette.entries()], 0);
		bit1 = BIT(color_prom[2 * palette.entries()], 1);
		bit2 = BIT(color_prom[2 * palette.entries()], 2);
		bit3 = BIT(color_prom[2 * palette.entries()], 3);
		int const b = 0x0e*bit0 + 0x1e * bit1 + 0x44*bit2 + 0x8f*bit3;
		palette.set_pen_color(i, rgb_t(r, g, b));

		color_prom++;
	}
}

void xorworld_state::videoram_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	COMBINE_DATA(&m_videoram[offset]);
	m_bg_tilemap->mark_tile_dirty(offset);
}

/*
    Tile format
    -----------

    Word | Bit(s)            | Description
    -----+-FEDCBA98-76543210-+--------------------------
      0  | ----xxxx xxxxxxxx | code
      0  | xxxx---- -------- | color
*/

TILE_GET_INFO_MEMBER(xorworld_state::get_bg_tile_info)
{
	int data = m_videoram[tile_index];
	int code = data & 0x0fff;

	tileinfo.set(0, code, data >> 12, 0);
}

void xorworld_state::video_start()
{
	m_bg_tilemap = &machine().tilemap().create(
			*m_gfxdecode, tilemap_get_info_delegate(*this, FUNC(xorworld_state::get_bg_tile_info)), TILEMAP_SCAN_ROWS,
			8, 8, 32, 32);
}

/*
    Sprite Format
    -------------

    Word | Bit(s)            | Description
    -----+-FEDCBA98-76543210-+--------------------------
      0  | -------- xxxxxxxx | x position
      0  | xxxxxxxx -------- | y position
      1  | -------- ------xx | flipxy? (not used)
      1  | ----xxxx xxxxxx-- | sprite number
      1  | xxxx---- -------- | sprite color
*/

void xorworld_state::draw_sprites(bitmap_ind16 &bitmap, const rectangle &cliprect )
{
	for (int i = 0; i < 0x40; i += 2)
	{
		int sx = m_spriteram[i] & 0x00ff;
		int sy = 240 - (((m_spriteram[i] & 0xff00) >> 8) & 0xff);
		int code = (m_spriteram[i+1] & 0x0ffc) >> 2;
		int color = (m_spriteram[i+1] & 0xf000) >> 12;

		m_gfxdecode->gfx(1)->transpen(bitmap,cliprect, code, color, 0, 0, sx, sy, 0);
	}
}

uint32_t xorworld_state::screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	m_bg_tilemap->draw(screen, bitmap, cliprect, 0, 0);
	draw_sprites(bitmap, cliprect);
	return 0;
}
