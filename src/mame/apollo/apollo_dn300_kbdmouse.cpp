#include "emu.h"
#include "speaker.h"
#include "apollo_dn300_kbdmouse.h"
#include "apollo_dn300_kbd.h"

#define KEY_CODE_TABLE (apollo_dn300_kbd_device::get_code_table())

#define LOG_GENERAL (1U << 0)

#define VERBOSE (LOG_GENERAL)
#include "logmacro.h"

#define KBD_MODE_0_COMPATIBILITY 0
#define KBD_MODE_1_KEYSTATE 1
#define KBD_MODE_2_RELATIVE_CURSOR_CONTROL 2
#define KBD_MODE_3_ABSOLUTE_CURSOR_CONTROL 3

#define MAP_APOLLO_KEYS 0

namespace {

INPUT_PORTS_START( apollo_dn300_kbdmouse )
	PORT_START( "keyboard1" )
	PORT_BIT( 0x00000001, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("~ `") PORT_CODE(KEYCODE_TILDE) /* ` ~ */
	PORT_BIT( 0x00000002, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("ESC") PORT_CODE(KEYCODE_ESC) /* ESC */
	PORT_BIT( 0x00000004, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("1 !") PORT_CODE(KEYCODE_1) /* 1 ! */
	PORT_BIT( 0x00000008, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("2 @") PORT_CODE(KEYCODE_2) /* 2 " */
	PORT_BIT( 0x00000010, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("3 #") PORT_CODE(KEYCODE_3) /* 3 # */
	PORT_BIT( 0x00000020, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("4 $") PORT_CODE(KEYCODE_4) /* 4 $ */
	PORT_BIT( 0x00000040, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("5 %") PORT_CODE(KEYCODE_5) /* 5 % */
	PORT_BIT( 0x00000080, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("6 &") PORT_CODE(KEYCODE_6) /* 6 & */
	PORT_BIT( 0x00000100, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("7 \'") PORT_CODE(KEYCODE_7) /* 7 ' */
	PORT_BIT( 0x00000200, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("8 (") PORT_CODE(KEYCODE_8) /* 8 ( */
	PORT_BIT( 0x00000400, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("9 )") PORT_CODE(KEYCODE_9) /* 9 ) */
	PORT_BIT( 0x00000800, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("0") PORT_CODE(KEYCODE_0) /* 0 */
	PORT_BIT( 0x00001000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("- _") PORT_CODE(KEYCODE_MINUS) /* - _ */
	PORT_BIT( 0x00002000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("= +") PORT_CODE(KEYCODE_EQUALS) /* = + */
	PORT_BIT( 0x00004000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("@ ^") PORT_CODE(KEYCODE_BACKSLASH2) /* ~ ` */
	PORT_BIT( 0x00008000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_CODE(KEYCODE_BACKSPACE) /* Backspace */
	PORT_BIT( 0x00010000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_CODE(KEYCODE_TAB) /* Tab */
	PORT_BIT( 0x00020000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("Q") PORT_CODE(KEYCODE_Q) /* Q */
	PORT_BIT( 0x00040000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("W") PORT_CODE(KEYCODE_W) /* W */
	PORT_BIT( 0x00080000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("E") PORT_CODE(KEYCODE_E) /* E */
	PORT_BIT( 0x00100000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("R") PORT_CODE(KEYCODE_R) /* R */
	PORT_BIT( 0x00200000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("T") PORT_CODE(KEYCODE_T) /* T */
	PORT_BIT( 0x00400000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("Y") PORT_CODE(KEYCODE_Y) /* Y */
	PORT_BIT( 0x00800000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("U") PORT_CODE(KEYCODE_U) /* U */
	PORT_BIT( 0x01000000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("I") PORT_CODE(KEYCODE_I) /* I */
	PORT_BIT( 0x02000000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("O") PORT_CODE(KEYCODE_O) /* O */
	PORT_BIT( 0x04000000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("P") PORT_CODE(KEYCODE_P) /* P */
	PORT_BIT( 0x08000000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("[ {") PORT_CODE(KEYCODE_OPENBRACE) /* [ { */
	PORT_BIT( 0x10000000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("] }") PORT_CODE(KEYCODE_CLOSEBRACE) /* ] } */
	PORT_BIT( 0x20000000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_CODE(KEYCODE_ENTER) /* Return */
	PORT_BIT( 0x40000000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("A") PORT_CODE(KEYCODE_A) /* A */
	PORT_BIT( 0x80000000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("S") PORT_CODE(KEYCODE_S) /* S */

	PORT_START( "keyboard2" )
	PORT_BIT( 0x00000001, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("D") PORT_CODE(KEYCODE_D) /* D */
	PORT_BIT( 0x00000002, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("F") PORT_CODE(KEYCODE_F) /* F */
	PORT_BIT( 0x00000004, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("G") PORT_CODE(KEYCODE_G) /* G */
	PORT_BIT( 0x00000008, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("H") PORT_CODE(KEYCODE_H) /* H */
	PORT_BIT( 0x00000010, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("J") PORT_CODE(KEYCODE_J) /* J */
	PORT_BIT( 0x00000020, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("K") PORT_CODE(KEYCODE_K) /* K */
	PORT_BIT( 0x00000040, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("L") PORT_CODE(KEYCODE_L) /* L */
	PORT_BIT( 0x00000080, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("; +") PORT_CODE(KEYCODE_COLON) /* ; + */
	PORT_BIT( 0x00000100, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME(": *") PORT_CODE(KEYCODE_QUOTE) /* : * */
	PORT_BIT( 0x00000200, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("\\ |") PORT_CODE(KEYCODE_BACKSLASH) /* \\ | */
	PORT_BIT( 0x00000400, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("Z") PORT_CODE(KEYCODE_Z) /* Z */
	PORT_BIT( 0x00000800, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("X") PORT_CODE(KEYCODE_X) /* X */
	PORT_BIT( 0x00001000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("C") PORT_CODE(KEYCODE_C) /* C */
	PORT_BIT( 0x00002000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("V") PORT_CODE(KEYCODE_V) /* V */
	PORT_BIT( 0x00004000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("B") PORT_CODE(KEYCODE_B) /* B */
	PORT_BIT( 0x00008000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("N") PORT_CODE(KEYCODE_N) /* N */
	PORT_BIT( 0x00010000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("M") PORT_CODE(KEYCODE_M) /* M */
	PORT_BIT( 0x00020000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME(", <") PORT_CODE(KEYCODE_COMMA) /* , < */
	PORT_BIT( 0x00040000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME(". >") PORT_CODE(KEYCODE_STOP) /* . > */
	PORT_BIT( 0x00080000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("/ ?") PORT_CODE(KEYCODE_SLASH) /* / ? */
//??
	PORT_BIT( 0x00100000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("~ '") PORT_CODE(KEYCODE_TILDE) /* Underscore (shifted only?) */
	PORT_BIT( 0x00200000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("Space")  PORT_CODE(KEYCODE_SPACE) /* Space */
	PORT_BIT( 0x00400000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("Home")  PORT_CODE(KEYCODE_HOME) /* Home */
	PORT_BIT( 0x00800000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("Delete")  PORT_CODE(KEYCODE_DEL) /* Del */
	PORT_BIT( 0x01000000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("Roll Up")  PORT_CODE(KEYCODE_PGUP) /* Roll Up */
	PORT_BIT( 0x02000000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("Roll Down")  PORT_CODE(KEYCODE_PGDN) /* Roll Down */
	PORT_BIT( 0x04000000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("End")  PORT_CODE(KEYCODE_END) /* End */
	PORT_BIT( 0x08000000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("Cursor Left")  PORT_CODE(KEYCODE_LEFT) /* Left */
	PORT_BIT( 0x10000000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("Cursor Up")  PORT_CODE(KEYCODE_UP) /* Up */
	PORT_BIT( 0x20000000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("Cursor Right")  PORT_CODE(KEYCODE_RIGHT) /* Right */
	PORT_BIT( 0x40000000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("Cursor Down")  PORT_CODE(KEYCODE_DOWN) /* Down */
//  PORT_BIT( 0x80000000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("Numpad CLR")  PORT_CODE(KEYCODE_NUMLOCK) /* CLR */
	PORT_BIT( 0x80000000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("MENU")  PORT_CODE(KEYCODE_MENU) /* Menu = POP */

	PORT_START( "keyboard3" )
	PORT_BIT( 0x00000001, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("Numpad /")  PORT_CODE(KEYCODE_SLASH_PAD) /* / (numpad) */
	PORT_BIT( 0x00000002, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("Numpad *")  PORT_CODE(KEYCODE_ASTERISK) /* * (numpad) */
	PORT_BIT( 0x00000004, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("Numpad -")  PORT_CODE(KEYCODE_MINUS_PAD) /* - (numpad) */
	PORT_BIT( 0x00000008, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("Numpad 7")  PORT_CODE(KEYCODE_7_PAD) /* 7 (numpad) */
	PORT_BIT( 0x00000010, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("Numpad 8")  PORT_CODE(KEYCODE_8_PAD) /* 8 (numpad) */
	PORT_BIT( 0x00000020, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("Numpad 9")  PORT_CODE(KEYCODE_9_PAD) /* 9 (numpad) */
	PORT_BIT( 0x00000040, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("Numpad +")  PORT_CODE(KEYCODE_PLUS_PAD) /* + (numpad) */
	PORT_BIT( 0x00000080, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("Numpad 4")  PORT_CODE(KEYCODE_4_PAD) /* 4 (numpad) */
	PORT_BIT( 0x00000100, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("Numpad 5")  PORT_CODE(KEYCODE_5_PAD) /* 5 (numpad) */
	PORT_BIT( 0x00000200, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("Numpad 6")  PORT_CODE(KEYCODE_6_PAD) /* 6 (numpad) */
	PORT_BIT( 0x00000400, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("Numpad =") /* = (numpad) */
	PORT_BIT( 0x00000800, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("Numpad 1")  PORT_CODE(KEYCODE_1_PAD) /* 1 (numpad) */
	PORT_BIT( 0x00001000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("Numpad 2")  PORT_CODE(KEYCODE_2_PAD) /* 2 (numpad) */
	PORT_BIT( 0x00002000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("Numpad 3")  PORT_CODE(KEYCODE_3_PAD) /* 3 (numpad) */
	PORT_BIT( 0x00004000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("Numpad Enter")  PORT_CODE(KEYCODE_ENTER_PAD) /* Enter (numpad) */
	PORT_BIT( 0x00008000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("Numpad 0")  PORT_CODE(KEYCODE_0_PAD) /* 0 (numpad) */
	PORT_BIT( 0x00010000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("Numpad ,") /* , (numpad) */
	PORT_BIT( 0x00020000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("Numpad .")  PORT_CODE(KEYCODE_DEL_PAD) /* 2 (numpad) */

	PORT_BIT( 0x00040000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("F1")  PORT_CODE(KEYCODE_F1) /* F1 */
	PORT_BIT( 0x00080000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("F2")  PORT_CODE(KEYCODE_F2) /* F2 */
	PORT_BIT( 0x00100000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("F3")  PORT_CODE(KEYCODE_F3) /* F3 */
	PORT_BIT( 0x00200000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("F4")  PORT_CODE(KEYCODE_F4) /* F4 */
	PORT_BIT( 0x00400000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("F5")  PORT_CODE(KEYCODE_F5) /* F5 */
	PORT_BIT( 0x00800000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("F6")  PORT_CODE(KEYCODE_F6) /* F6 */
	PORT_BIT( 0x01000000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("F7")  PORT_CODE(KEYCODE_F7) /* F7 */
	PORT_BIT( 0x02000000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("F8")  PORT_CODE(KEYCODE_F8) /* F8 */
	PORT_BIT( 0x04000000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("F9")  PORT_CODE(KEYCODE_F9) /* F9 */
	PORT_BIT( 0x08000000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("F10") PORT_CODE(KEYCODE_F10) /* F10 */
	PORT_BIT( 0x10000000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("F11") PORT_CODE(KEYCODE_F11) /* F11 */
	PORT_BIT( 0x20000000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("F12") PORT_CODE(KEYCODE_F12) /* F12 */
	PORT_BIT( 0x40000000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("Insert") PORT_CODE(KEYCODE_INSERT) /* Insert */
//  PORT_BIT( 0x80000000, 0x0000, IPT_UNUSED )
	PORT_BIT( 0x80000000, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("WIN_R")  PORT_CODE(KEYCODE_RWIN) /* Windows Right = Next Window */

	PORT_START( "keyboard4" )
	PORT_BIT( 0x00000001, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("Caps")  PORT_CODE(KEYCODE_CAPSLOCK) /* Caps lock */
	PORT_BIT( 0x00000002, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("Shift")  PORT_CODE(KEYCODE_LSHIFT) /* Shift */
	PORT_BIT( 0x00000004, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("Ctrl")  PORT_CODE(KEYCODE_LCONTROL)  PORT_CODE(KEYCODE_RCONTROL) /* Ctrl */
	PORT_BIT( 0x00000008, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("ALT_L")  PORT_CODE(KEYCODE_LALT) /* ALT */
	PORT_BIT( 0x00000010, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("ALT_R")  PORT_CODE(KEYCODE_RALT) /* ALT GR */
	PORT_BIT( 0x00000020, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("Shift")  PORT_CODE(KEYCODE_RSHIFT) /* Shift */
	PORT_BIT( 0x00000040, IP_ACTIVE_HIGH, IPT_KEYBOARD )  PORT_NAME("Num Lock")  PORT_CODE(KEYCODE_NUMLOCK) /* Num Lock */

	// labeled as 1..3 because they're accessed as "mouse%u" through an ioport array
	PORT_START("mouse1")  // mouse buttons
	PORT_BIT( 0x00000010, IP_ACTIVE_HIGH, IPT_BUTTON1) PORT_NAME("Left mouse button") PORT_CODE(MOUSECODE_BUTTON1)
	PORT_BIT( 0x00000020, IP_ACTIVE_HIGH, IPT_BUTTON3) PORT_NAME("Right mouse button") PORT_CODE(MOUSECODE_BUTTON3)
	PORT_BIT( 0x00000040, IP_ACTIVE_HIGH, IPT_BUTTON2) PORT_NAME("Center mouse button") PORT_CODE(MOUSECODE_BUTTON2)

	PORT_START("mouse2")  // X-axis
	PORT_BIT(0x3ff, 0x000, IPT_MOUSE_X) PORT_SENSITIVITY(100) PORT_MINMAX(0x000, 0x3ff) PORT_KEYDELTA(0) PORT_PLAYER(1)
	//PORT_BIT( 0xffff, 0x00, IPT_MOUSE_X) PORT_SENSITIVITY(100) PORT_KEYDELTA(0) PORT_PLAYER(1) PORT_MINMAX(0, 1024)

	PORT_START("mouse3")  // Y-axis
	PORT_BIT(0x3ff, 0x000, IPT_MOUSE_Y) PORT_SENSITIVITY(100) PORT_MINMAX(0x000, 0x3ff) PORT_KEYDELTA(0) PORT_PLAYER(1)
	//PORT_BIT( 0xffff, 0x00, IPT_MOUSE_Y) PORT_SENSITIVITY(100) PORT_KEYDELTA(0) PORT_PLAYER(1) PORT_MINMAX(0, 1024)
INPUT_PORTS_END

} // anonymous namespace

DEFINE_DEVICE_TYPE(APOLLO_DN300_KBDMOUSE, apollo_dn300_kbdmouse_device, "apollo_dn300_kbdmouse", "Apollo Keyboard and Mouse")

apollo_dn300_kbdmouse_device::apollo_dn300_kbdmouse_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
    : device_t(mconfig, APOLLO_DN300_KBDMOUSE, tag, owner, clock)
	, device_serial_interface(mconfig, *this)
	, m_io_keyboard(*this, "keyboard%u", 1U)
	, m_io_mouse(*this, "mouse%u", 1U)
	, m_beep(*this, "beep")
	, m_tx_w(*this)
{
}

void
apollo_dn300_kbdmouse_device::device_add_mconfig(machine_config &config)
{
	/* keyboard beeper */
	SPEAKER(config, "mono").front_center();
	BEEP(config, m_beep, 1000).add_route(ALL_OUTPUTS, "mono", 1.00);
}

ioport_constructor
apollo_dn300_kbdmouse_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(apollo_dn300_kbdmouse);
}

void
apollo_dn300_kbdmouse_device::device_resolve_objects()
{
	m_tx_w.resolve_safe();
}

void
apollo_dn300_kbdmouse_device::device_start()
{
	LOG("start apollo_dn300_kbdmouse");

	m_update_timer = timer_alloc(FUNC(apollo_dn300_kbdmouse_device::update_timer_cb), this);
}

void
apollo_dn300_kbdmouse_device::device_reset()
{
    LOG("reset apollo_dn300_kbdmouse");

    m_mode = KBD_MODE_0_COMPATIBILITY;
    m_delay_ms = 500;
    m_repeat_ms = 33;
    m_last_pressed = 0;
    m_numlock_state = 0;

	memset(m_keytime, 0, sizeof(m_keytime));
	memset(m_keyon, 0, sizeof(m_keyon));

    m_last_b = 0xff; // active low
    m_last_x = 0;
    m_last_y = 0;

	m_update_timer->adjust(attotime::zero, 0, attotime::from_msec(5)); // every 5ms

    // serial config
    // This is the initial config from the ROM
	set_data_frame(1, 8, PARITY_NONE, STOP_BITS_1);
	set_rcv_rate(1200);
	set_tra_rate(1200);

	m_tx_busy = false;
    m_txring_rpos = m_txring_wpos = 0;

}

TIMER_CALLBACK_MEMBER(apollo_dn300_kbdmouse_device::update_timer_cb)
{
	send_keyboard_update();

	// Note: we omit extra traffic while keyboard is in Compatibility mode
	if (m_mode != KBD_MODE_0_COMPATIBILITY)
	{
		send_mouse_update();
	}
}

void
apollo_dn300_kbdmouse_device::send_mouse_update()
{
    // TODO delay so we don't flood mouse

	char b = m_io_mouse[0]->read();
	short rx = m_io_mouse[1]->read();
	short ry = m_io_mouse[2]->read();

	// note: only the deltas between rx/ry values and their previous values
	// are valid.  They are not meaningful absolute coordinates, and they are not
	// explicit deltas themselves.

	ry = -ry;

	if (m_last_b < 0)
	{
		m_last_b = b;
		m_last_x = rx;
		m_last_y = ry;
		return;
	}

	if (b == m_last_b && rx == m_last_x && ry == m_last_y)
	{
		return;
	}

	uint8_t mouse_data[4];

	short dx = rx - m_last_x;
	short dy = ry - m_last_y;

	// we only have 8 bits to use for the deltas, so cap.  shouldn't really happen.
	dx = (dx < -127) ? -127 : (dx > 127) ? 127 : dx; 
	dy = (dy < -127) ? -127 : (dy > 127) ? 127 : dy;

	// LOG("read_mouse: b=%02x dx=%d dy=%d (x=%d y=%d)", b, dx, dy, rx, ry);

	int bytes = 0;
	if (m_mode == KBD_MODE_0_COMPATIBILITY)
	{
		// if in compat mode, mouse data has a prefix
		mouse_data[bytes++] = 0xdf;
	} else if (m_mode != KBD_MODE_2_RELATIVE_CURSOR_CONTROL) {
		LOG("Keyboard in unexpected mode %d, setting to RELATIVE_CURSOR_CONTROL", m_mode);
		set_mode(KBD_MODE_2_RELATIVE_CURSOR_CONTROL);
	}

	// first byte is
	// 1 M R L X X Y Y (XXYY = 1111 to indicate that X/Y are valid).  MRL are active-low. b already has 1 bits in the right place
	// for pressed buttons based on the input ports.
	//mouse_data[1] = 0xf0 ^ b; // vlad -- this is what this used to be but that seems wrong
	mouse_data[bytes++] = 0x8f | ~b; // TODO we can just make the input ports for b be ACTIVE_LOW?
	mouse_data[bytes++] = (char)dx;
	mouse_data[bytes++] = (char)dy;

#if false
	// If we want to send touchpad data, we need to send 4 bytes:
	//   0: E8
	//   1: low 8 bits X
	//   2: (high 4 bits y) << 4 | (high 4 bits x)
	//   3: low 8 bits y
	// The range of X and Y coordinates is approximately 30 to 1100 for the
	// actual touchpad, no idea what our setup is here.  We need to also
	// find a way to get absolute coordinates from the mouse driver.
	mouse_data[0] = 0xe8;
	mouse_data[1] = (rx >> 4) & 0xff;
	mouse_data[2] = (((ry >> 8) & 0x0f) << 4) | ((rx >> 8) & 0x0f);
	mouse_data[3] = (ry >> 4) & 0xff;
	bytes = 4;
#endif

	LOG("MOUSE: %02x %02x %02x %02x (b: %d, dx %d dy %d)", mouse_data[0], mouse_data[1], mouse_data[2], mouse_data[3], b, dx, dy);

    tx_chars(mouse_data, bytes);

	// mouse data submitted; update current mouse state
	m_last_b = b;
	m_last_x = rx;
	m_last_y = ry;

	//m_tx_pending = 50; // mouse data packet will take 40 ms
}

void
apollo_dn300_kbdmouse_device::send_keyboard_update()
{
	int x;
#ifdef notyet
	int repeat = 0;
#endif

	for (x = 0; x < 0x80; x++)
	{
		ioport_value v = m_io_keyboard[x / 32]->read();
		if (!(v & (1 << (x % 32))))
		{
			// no key pressed
			if (m_keyon[x] != 0)
			{
#ifdef notyet
				// a key has been released
				tx_scancode(0x80 + x, 0);
#endif
				m_keytime[x] = 0;
				m_keyon[x] = 0;
				m_last_pressed = 0;
#ifdef notyet
				LOG1(("released key 0x%02x at time %d",x, m_keytime[x]));
#endif
			}
		}
		else if (m_keyon[x] == 0)
		{
			if (tx_scancode(x, 0))
			{
                // if a key was actually transmitted, flag it for repeat
				m_keytime[x] = m_mode == KBD_MODE_0_COMPATIBILITY ? m_delay_ms : m_repeat_ms;
				m_keyon[x] = 1;
				m_last_pressed = x;
				LOG("pushed key 0x%02x at time %d",x, m_keytime[x]);
			}
		}
		else if (m_last_pressed == x)
		{
#ifdef notyet
			// a key is being held; adjust delay/repeat timers
			m_keytime[x] -= 5;
			if (m_keytime[x] <= 0)
			{
				tx_scancode(x, ++repeat);
				m_keytime[x] = m_repeat;
			}
			LOG("holding key 0x%02x at time %d",x, m_keytime[x]));
#endif
		}
	}
}

bool
apollo_dn300_kbdmouse_device::tx_scancode(uint8_t code, uint8_t repeat)
{
	uint16_t key_code = 0;
	ioport_value const modifiers = m_io_keyboard[3]->read();
	uint8_t const caps = BIT(modifiers,0);
	uint8_t const shift = BIT(modifiers,1) | BIT(modifiers,5);
	uint8_t const ctrl = BIT(modifiers,2);

#if MAP_APOLLO_KEYS

	uint8_t const numlock = BIT(modifiers,6);

	// FIXME: numlock was ok for MAME with SDL1.2 but never worked for MAME with SDL2
	// nasty hack: we toggle the numlock state from the numlock Key Up transition (starting with 0 for numlock off)
	if (code == 0xe6)
		m_numlock_state = !m_numlock_state;

	LOG1(("scan_code = 0x%02x numlock = %d numlock_state = %d", code, numlock, m_numlock_state);

	if (m_numlock_state)
	{
		// don't map function keys to Apollo left keypad
		switch (code)
		{
		case 0x52: code = 0x75; break; // F1
		case 0x53: code = 0x76; break; // F2
		case 0x54: code = 0x77; break; // F3
		case 0x55: code = 0x78; break; // F4
		case 0x56: code = 0x79; break; // F5
		case 0x57: code = 0x7a; break; // F6
		case 0x58: code = 0x7b; break; // F7
		case 0x59: code = 0x7c; break; // F8
		case 0x5a: code = 0x7d; break; // F9
		case 0x5b: code = 0x74; break; // F0 = F10
		}
	}
#endif

    // mouse test
#if false
	if (code == 0x1b) { // [
        return tx_chars({0xdf, 0xff, -10, 0});
	} else if (code == 0x1c) { // ]
        return tx_chars({0xdf, 0xff, 10, 0});
	}
#endif

	auto const &entry = KEY_CODE_TABLE[code & 0x7f];
	if (m_mode == KBD_MODE_0_COMPATIBILITY)
	{
		if (code & 0x80) {
			// skip up code in ASCII mode
		} else if (repeat > 0 && !entry.auto_repeat) {
			// skip repeat in ASCII mode
		} else if (ctrl) {
			key_code = entry.control;
		} else if (shift) {
			key_code = entry.shifted;
		} else if (caps) {
			key_code = entry.caps_lock;
		} else {
			key_code = entry.unshifted;
		}

#if 0
		if (code == 0) {
			static uint16_t keycode_for_0 = 0x20;
			key_code = keycode_for_0++;
			if (key_code == 0x80) {
				key_code = 0x20;
			}
			LOG1(("tilde hack - emitted key_code = 0x%02x", key_code);
		}
#endif
	} else {
        // any other mode sends scan codes with press and release
        // values
		if (repeat > 0) {
			if (repeat == 1) {
				// auto repeat (but only for first scanned key)
				key_code = 0x7f;
			}
		} else if (code & 0x80) {
			key_code = entry.up;
		} else {
			key_code = entry.down;
		}
	}

    // if it maps to nothing, we did nothing
    if (key_code == 0)
        return false;

    LOG("scan_code = 0x%02x key_code = 0x%04x, caps/shift/ctrl = %d/%d/%d",code, key_code, caps ? 1 : 0, shift ? 1 : 0, ctrl ? 1 : 0);

    // WTF? why does stuff keep forcing back mode 1?
    if (m_mode > KBD_MODE_1_KEYSTATE)
        set_mode(KBD_MODE_1_KEYSTATE);

    if (key_code & 0xff00)
        tx_char(key_code >> 8);
    tx_char(key_code & 0xff);

	return true;
}

void
apollo_dn300_kbdmouse_device::beep(bool on)
{
    // TODO
    //m_beep.target()->set_state(on ? 1 : 0);
    //m_beep_timer->adjust(attotime::from_msec(10), 0, attotime::zero);
}

void apollo_dn300_kbdmouse_device::set_mode(uint16_t mode)
{
	LOG("keyboard set_mode: %d", mode);

    // ???? echo back?
	// tx_char(0xff);
	// tx_char(mode);
	m_mode = mode;
}

void
apollo_dn300_kbdmouse_device::tx_char(uint8_t data)
{
	if (m_tx_busy)
    {
		m_txring[m_txring_wpos++] = data;
        if (m_txring_wpos >= TXRING_SIZE)
            m_txring_wpos = 0;
        return;
	}

    m_tx_busy = true;
    transmit_register_setup(data);
}

void
apollo_dn300_kbdmouse_device::tx_chars(uint8_t *data, int count)
{
    for (int i = 0; i < count; i++)
        tx_char(data[i]);
}

void
apollo_dn300_kbdmouse_device::rx_char(uint8_t data)
{
	LOG("KBD IN rx_char <- %02x", data);

    // vlad: I have no idea what the framing format here is.
    // It looks like a message can start with 0x00 or 0xff.
    // If it's 0x00, that's a complete message.  If it's 0xff,
    // it's the start of a message.
    // I don't know what loopback mode determines -- maybe 0xff
    // messages need to be echoed back?  Or 0xff starts a sequence?
	if (data == 0x00 && m_rx_message == 0)
	{
		if (m_loopback_mode)
		{
			set_mode(KBD_MODE_0_COMPATIBILITY);
			m_loopback_mode = 0;
		}
        return;
	}

    // the original code did this as part of putdata(), which was called
    // from this routine to send data back.  I don't know why the keyboard
    // is forced into state 1.
    if (m_mode > KBD_MODE_1_KEYSTATE) {
        set_mode(KBD_MODE_1_KEYSTATE);
    }

	if (data == 0xff && m_rx_message == 0)
	{
		m_rx_message = data;
		m_loopback_mode = 1;
        tx_char(0xff);
        return;
	}

    if (m_loopback_mode)
        tx_char(data);

    m_rx_message = m_rx_message << 8 | data;

    switch (m_rx_message)
    {
		case 0xff00:
			m_mode = KBD_MODE_0_COMPATIBILITY;
			m_loopback_mode = 0;
			m_rx_message = 0;
			break;
		case 0xff01:
			m_mode = KBD_MODE_1_KEYSTATE;
			m_rx_message = 0;
			break;
		case 0xff1116:
			//putdata(ff1116_data, sizeof(ff1116_data)); // er what? we already sent back 0xff and 0x11, we're supposed to send it all again?
			m_loopback_mode = 0;
			m_rx_message = 0;
			break;
		case 0xff1117:
            // this is a finished message; original code didn't do any echo back?
            // and it keeps loopback mode enabled?
			m_rx_message = 0; // no echo?
			break;
		case 0xff1221: { // receive ID message
            const char* id = "3-@\r2-0\rSD-03863-MS\r";
            const char* german_id = "3-A\r2-0\rSD-03863-MS\r";

            tx_str(id);

			m_rx_message = 0;
			break;
        }
		case 0xff2181: // beeper on (for 300 ms)
            beep();
            m_rx_message = 0;
			break;
		case 0xff2182: // beeper off
            beep(false);
			m_rx_message = 0;
			break;
		default:
			break;
    }
}

//
// low level serial; called by tx_char/calls to rx_char
//

// get next bit to send from serial device; send it to the output port
void
apollo_dn300_kbdmouse_device::tra_callback()
{
	int bit = transmit_register_get_data_bit();
	m_tx_w(bit);
}

// enqueue another byte if present for sending via tra_callback
void
apollo_dn300_kbdmouse_device::tra_complete()    // Tx, device completed sending byte
{
    if (m_txring_rpos != m_txring_wpos) {
        transmit_register_setup(m_txring[m_txring_rpos++]);
        if (m_txring_rpos >= TXRING_SIZE)
            m_txring_rpos = 0;
    } else {
        m_tx_busy = false;
    }
}

void
apollo_dn300_kbdmouse_device::rcv_complete()    // Rx complete, byte available
{
	receive_register_extract();
	uint8_t data = get_received_char();
	printf("KBD rcv complete %02x\n", data);

    rx_char(data);
}
