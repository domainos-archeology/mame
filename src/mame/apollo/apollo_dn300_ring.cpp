
// see http://www.bitsavers.org/pdf/apollo/002398-04_Domain_Engineering_Handbook_Rev4_Jan87.pdf

#include "emu.h"

#define VERBOSE 2
#include "apollo_dn300.h"

//// XMIT_COMMAND
#define XMIT_COMMAND_INTERRUPT_ENABLE 0x4000
#define XMIT_COMMAND_ENABLE 0x2000 // (start the transmit)
#define XMIT_COMAMND_FORCE_TRANSMIT 0x1000
// NOTES from EH87:
//  1. To start a transmit normally, use 6000.
//  2. To force a transmit, use 7000.
//  3. To stop a transmit that has already started, clear the
//     transmit enable bit.
//  4. Writing anything to this register clears the transmit
//     interrupt.


/// RCV_COMMAND
#define RCV_COMMAND_INTERRUPT_ENABLE 0x4000
#define RCV_COMMAND_ENABLE 0x2000 // (start the receive)
// NOTES from EH87:
//  1. To start a receive normally, use 6000.
//  2. To stop a receive that has already started, clear the
//     receive enable bit.


//// XMIT_STATUS
#define XMIT_STATUS_INTERRUPT_PENDING 0x8000
#define XMIT_STATUS_INTERRUPT_ENABLED 0x4000
#define XMIT_STATUS_BUSY              0x2000
#define XMIT_STATUS_DISCONNECTED      0x1000
#define XMIT_STATUS_BIPHASE_ERROR     0x0800
#define XMIT_STATUS_ESB_ERROR         0x0400 // ESB = elastic store buffer
#define XMIT_STATUS_NO_RETURN         0x0200 // a complete pkt frame never arrived
#define XMIT_STATUS_CRC_ERROR         0x0100
#define XMIT_STATUS_ACK_PARITY_ERROR  0x0080 // 0=no error, 1=error detected
#define XMIT_STATUS_EXTERNAL_ERROR    0x0040 // err during DMA, e.g. parity, bus-error
#define XMIT_STATUS_PROTOCOL_ERROR    0x0020 // the pkt hdr with FROM ID never came back
#define XMIT_STATUS_IOCOPY            0x0010 // somebody Intended to COPY -- was willing to rcv
#define XMIT_STATUS_ACK_BYTE_ERRBIT   0x0008 // sombody (anybody!) set the "error detected" bit
#define XMIT_STATUS_COPY              0x0004 // somebody did COPY the pkt
#define XMIT_STATUS_WACK              0x0002
#define XMIT_STATUS_UNDERRUN          0x0001 // DMA didn't keep up with xmit data rate
// NOTES from EH87:
//  1. A successful transmit will have a transmit status of 0014
//  2. A WACK will have a transmit status of 0012


//// RCV_STATUS
#define RCV_STATUS_INTERRUPT_PENDING 0x8000
#define RCV_STATUS_INTERRUPT_ENABLED 0x4000
#define RCV_STATUS_BUSY              0x2000
#define RCV_STATUS_DISCONNECTED      0x1000
#define RCV_STATUS_BIPHASE_ERROR     0x0800
#define RCV_STATUS_ESB_ERROR         0x0400 // ESB = elastic store buffer
#define RCV_STATUS_TIMEOUT           0x0200 // timeout (the hdr of a msg was seen, but it never ended)
#define RCV_STATUS_CRC_ERROR         0x0100
#define RCV_STATUS_ACK_PARITY_ERROR  0x0080 // 0=no error, 1=error detected
#define RCV_STATUS_EXTERNAL_ERROR    0x0040 // err during DMA, e.g. parity, bus-error
#define RCV_STATUS_DMA_END_OF_RANGE  0x0020
#define RCV_STATUS_IOCOPY            0x0010 // somebody before me Intended to COPY
#define RCV_STATUS_ACK_BYTE_ERRBIT   0x0008 // sombody before me set the "error detected" bit
#define RCV_STATUS_COPY 			 0x0004 // somebody before me did COPY the pkt
#define RCV_STATUS_WACK              0x0002 // somebody before me WACKed the pkt
#define RCV_STATUS_OVERRUN           0x0001 // DMA didn't keep up with rcv data rate


////  DIAGNOSTIC_STATUS
#define DIAG_STATUS_INTERRUPT_PENDING  0x8000 // bad_pkt_cnt_overflow interrupt
#define DIAG_STATUS_INTERRUPT_ENABLED  0x4000 // bad_pkt_cnt_overflow interrupt
#define DIAG_STATUS_CONNECTED_TO_NET   0x2000
#define DIAG_STATUS_STICKY_BIPHASE_ERR 0x1000 // error seen since bit was cleared
#define DIAG_STATUS_DELAY_ON           0x0800 // the delay is enabled
#define DIAG_STATUS_STICKY_GOOD_SEEN   0x0400 // good pkt seen since bit was cleared
#define DIAG_STATUS_STICKY_ESB_ERR     0x0200 // error seen since bit was cleared
#define DIAG_STATUS_BAD_PACKET_COUNT   0x1FF  // 9-bit counter for 1st detecting errs
// NOTES from EH87:
//  1. Counter is number of times this node found an
//     error in a packet going by (regardless of packet
//     target node ID), found the error bit in the
//     ackbyte clear, and so was the first to set the
//     error bit to one.
//  2. The bad_pkg_cnt interrupt occurs when the counter
//     counts from 255 to 256 (i.e. first uses its
//     highest order bit).
//  3. The counter sticks at 511 if more than 511 errors
//     are seen.
//  4. Writing anything to the diagnostic command
//     register (word) (see below) clears interrupt and
//     all sticky bits.


// DIAGNOSTIC_COMMAND
#define DIAG_COMMAND_DMA_TEST         0x8000
#define DIAG_COMMAND_ENABLE_INTERRUPT 0x4000
#define DIAG_COMMAND_CONNECT          0x2000
#define DIAG_COMMAND_DISCONNECT       0x1000
#define DIAG_COMMAND_DELAY_OFF        0x0800
#define DIAG_COMMAND_DELAY_ON         0x0400
#define DIAG_COMMAND_SNOOP            0x0200
// NOTES from EH87:
//  Writing anything to the register (word) clears interrupt
//  and all sticky bits in diagnostic status register


// TMASK
#define TMASK_BROADCAST           0x80
#define TMASK_HARDWARE_DIAGNOSTIC 0x40
#define TMASK_THANK_YOU           0x20
#define TMASK_PLEASE              0x10
#define TMASK_PAGING              0x08
#define TMASK_USER                0x04
#define TMASK_SOFTWARE_DIAGNOSTIC 0x02
#define TMASK_XTYPE3              0x01
// NOTES from EH87:
//  Except for BROADCAST, these bits are software defined.

DEFINE_DEVICE_TYPE(APOLLO_DN300_RING_CTRLR, apollo_dn300_ring_ctrlr_device, APOLLO_DN300_RING_TAG, "Apollo DN300 Ring controller")

apollo_dn300_ring_ctrlr_device::apollo_dn300_ring_ctrlr_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock):
    device_t(mconfig, APOLLO_DN300_RING_CTRLR, tag, owner, clock),
	irq_cb(*this),
	rcv_header_drq_wr_cb(*this),
	rcv_data_drq_wr_cb(*this),
	transmit_data_drq_wr_cb(*this)
{
}

void
apollo_dn300_ring_ctrlr_device::device_start()
{
}

void
apollo_dn300_ring_ctrlr_device::device_reset()
{
}

void
apollo_dn300_ring_ctrlr_device::map(address_map &map)
{
	map(0x0,0x1).rw(FUNC(apollo_dn300_ring_ctrlr_device::xmit_status_r), FUNC(apollo_dn300_ring_ctrlr_device::xmit_command_w));
	map(0x2,0x3).rw(FUNC(apollo_dn300_ring_ctrlr_device::rcv_status_r), FUNC(apollo_dn300_ring_ctrlr_device::rcv_command_w));
	map(0x4,0x4).rw(FUNC(apollo_dn300_ring_ctrlr_device::tmask_r), FUNC(apollo_dn300_ring_ctrlr_device::tmask_w));
	map(0x6,0x7).rw(FUNC(apollo_dn300_ring_ctrlr_device::diag_status_r), FUNC(apollo_dn300_ring_ctrlr_device::diag_command_w));
	map(0x8,0xB).rw(FUNC(apollo_dn300_ring_ctrlr_device::ring_id_r), FUNC(apollo_dn300_ring_ctrlr_device::ring_id_w));

	map(0x10,0x16).r(FUNC(apollo_dn300_ring_ctrlr_device::id_r));
}

void
apollo_dn300_ring_ctrlr_device::xmit_command_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
    SLOG1(("DN300_RING: writing xmit_command at offset %02x = %04x & %08x", offset, data, mem_mask));
}

uint16_t
apollo_dn300_ring_ctrlr_device::xmit_status_r(offs_t offset, uint16_t mem_mask)
{
	uint16_t data = m_xmit_status & mem_mask;
    SLOG1(("DN300_RING: reading xmit_status at offset %02x & %08x = %04x", offset, mem_mask, data));
    return 0;
}

void
apollo_dn300_ring_ctrlr_device::rcv_command_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
    SLOG1(("DN300_RING: writing rcv_command at offset %02x = %04x & %08x", offset, data, mem_mask));
}

uint16_t
apollo_dn300_ring_ctrlr_device::rcv_status_r(offs_t offset, uint16_t mem_mask)
{
	uint16_t data = m_rcv_status & mem_mask;
    SLOG1(("DN300_RING: reading rcv_status at offset %02x & %08x = %04x", offset, mem_mask, data));
    return 0;
}

void
apollo_dn300_ring_ctrlr_device::tmask_w(offs_t offset, uint8_t data, uint8_t mem_mask)
{
    SLOG1(("writing tmask at offset %02x = %02x & %08x", offset, data, mem_mask));
}

uint8_t
apollo_dn300_ring_ctrlr_device::tmask_r(offs_t offset, uint8_t mem_mask)
{
	uint8_t data = m_tmask & mem_mask;
    SLOG1(("DN300_RING: reading tmask at offset %02x & %08x = %02x", offset, mem_mask, data));
    return 0;
}

void
apollo_dn300_ring_ctrlr_device::diag_command_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
    SLOG1(("DN300_RING: writing diag_command at offset %02x = %04x & %08x", offset, data, mem_mask));

	// writing anything to the register clears the interrupt and all sticky bits
	// in the diagnostic status register.
	m_diag_status &= ~(
//		DIAG_STATUS_INTERRUPT_PENDING | do we clear this too?
		DIAG_STATUS_INTERRUPT_ENABLED |
		DIAG_STATUS_STICKY_BIPHASE_ERR |
		DIAG_STATUS_STICKY_GOOD_SEEN |
		DIAG_STATUS_STICKY_ESB_ERR
	);

	// irq_cb(CLEAR_LINE);

	// ... more stuff here
}

uint16_t
apollo_dn300_ring_ctrlr_device::diag_status_r(offs_t offset, uint16_t mem_mask)
{
	uint16_t data = m_diag_status & mem_mask;
    SLOG1(("DN300_RING: reading diag_status at offset %02x & %08x = %04x", offset, mem_mask, data));
    return 0;
}

void
apollo_dn300_ring_ctrlr_device::ring_id_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
    SLOG1(("DN300_RING: writing ring_id at offset %02x = %04x & %08x", offset, data, mem_mask));
}

uint16_t
apollo_dn300_ring_ctrlr_device::ring_id_r(offs_t offset, uint16_t mem_mask)
{
	uint8_t data = (offset == 0 ? m_ring_id_msb : m_ring_id_lsb) & mem_mask;
    SLOG1(("DN300_RING: reading ring_id at offset %02x & %08x = %04x", offset, mem_mask, data));
    return 0;
}

uint8_t
apollo_dn300_ring_ctrlr_device::id_r(offs_t offset, uint8_t mem_mask)
{
	uint8_t data = m_id[offset / 2] & mem_mask;
    SLOG1(("DN300_RING: reading node_id at offset %02x & %08x = %02x", offset, mem_mask, data));
    return 0;
}

uint8_t
apollo_dn300_ring_ctrlr_device::rcv_header_read_byte(offs_t offset)
{
	uint8_t data = 0;
    SLOG1(("DN300_RING: ring rcv_header_read_byte offset %02x = %02x", offset, data));
	return data;
}

void
apollo_dn300_ring_ctrlr_device::rcv_header_write_byte(offs_t offset, uint8_t data)
{
    SLOG1(("ring rcv_header_write_byte at offset %02x = %02x", offset, data));
}

uint8_t
apollo_dn300_ring_ctrlr_device::rcv_data_read_byte(offs_t offset)
{
	uint8_t data = 0;
    SLOG1(("DN300_RING: ring rcv_data_read_byte offset %02x = %02x & %08x", offset, data));
	return data;
}

void
apollo_dn300_ring_ctrlr_device::rcv_data_write_byte(offs_t offset, uint8_t data)
{
    SLOG1(("DN300_RING: ring rcv_data_write_byte at offset %02x = %02x", offset, data));
}

uint8_t
apollo_dn300_ring_ctrlr_device::transmit_read_byte(offs_t offset)
{
	uint8_t data = 0;
    SLOG1(("DN300_RING: ring transmit_read_byte offset %02x = %02x & %08x", offset, data));
	return data;
}

void
apollo_dn300_ring_ctrlr_device::transmit_write_byte(offs_t offset, uint8_t data)
{
    SLOG1(("DN300_RING: ring transmit_write_byte at offset %02x = %02x", offset, data));
}
