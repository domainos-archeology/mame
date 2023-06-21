
#ifndef MAME_MACHINE_APOLLO_DN300_KBDMOUSE_H
#define MAME_MACHINE_APOLLO_DN300_KBDMOUSE_H

#include <vector>
#include "sound/beep.h"
#include "diserial.h"

// ======================> apollo_dn300_kbdmouse_device

class apollo_dn300_kbdmouse_device : public device_t, public device_serial_interface
{
public:
	// construction/destruction
	apollo_dn300_kbdmouse_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	auto tx_cb() { return m_tx_w.bind(); }
	// note: "rx_w" is provided by device_serial_interface

protected:
	// device-level overrides
	virtual ioport_constructor device_input_ports() const override;
	virtual void device_add_mconfig(machine_config &config) override;
	virtual void device_resolve_objects() override;
	virtual void device_start() override;
	virtual void device_reset() override;

	// calls to/from the low level serial interface
	void tx_char(uint8_t data); // enqueue a byte for sending
	void tx_chars(uint8_t *data, int count); // enqueue bytes
	int tx_chars(const std::vector<uint8_t> &data) { tx_chars((uint8_t *)&data[0], data.size()); return (int) data.size(); }
    void tx_str(const char* data) { tx_chars((uint8_t*)data, strlen(data)); }
	void rx_char(uint8_t data); // keyboard received a byte

	// low-level serial interface
	virtual void rcv_complete() override;    // Rx completed receiving byte
	virtual void tra_complete() override;    // Tx completed sending byte
	virtual void tra_callback() override;    // Tx send bit

	// set the keyboard mode
	void set_mode(uint16_t mode);
	// scan the keyboard and transmit possibly translated scancodes
	void send_keyboard_update();
	void send_mouse_update();
	// transmit translated scancode; true if something was transmitted
	bool tx_scancode(uint8_t code, uint8_t repeat);
    void beep(bool on = true);

	required_ioport_array<4> m_io_keyboard;
	required_ioport_array<3> m_io_mouse;

	required_device<beep_device> m_beep;

	TIMER_CALLBACK_MEMBER(update_timer_cb);
	emu_timer* m_update_timer;

	// mouse state
    int m_last_b;  // previous mouse button values
    int m_last_x;  // previous mouse x-axis value
    int m_last_y;  // previous mouse y-axis value

	static const int TXRING_SIZE = 64;

	// transmitter state (keyboard to CPU)
	devcb_write_line m_tx_w; // the transmitter bit output
	uint8_t m_txring[TXRING_SIZE];
	int m_txring_rpos, m_txring_wpos;
	bool m_tx_busy;

    // receiver state (CPU to keyboard)
	uint32_t m_rx_message;
	uint16_t m_loopback_mode;

	// keyboard state
	uint16_t m_mode;
	uint16_t m_delay_ms;      // key press delay after initial press
	uint16_t m_repeat_ms;     // key press repeat rate
	uint16_t m_last_pressed;  // last key pressed, for repeat key handling
	uint16_t m_numlock_state; // current num lock state
	int m_keytime[0x80];      // time until next key press (1 ms)
	uint8_t m_keyon[0x80];    // is 1 if key is pressed

};

DECLARE_DEVICE_TYPE(APOLLO_DN300_KBDMOUSE, apollo_dn300_kbdmouse_device)

#endif
