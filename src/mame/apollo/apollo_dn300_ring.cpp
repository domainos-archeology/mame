
// see http://www.bitsavers.org/pdf/apollo/002398-04_Domain_Engineering_Handbook_Rev4_Jan87.pdf

#include "emu.h"

#define VERBOSE 2
#include "apollo_dn300.h"

DEFINE_DEVICE_TYPE(APOLLO_DN300_RING, apollo_dn300_ring_device, APOLLO_DN300_RING_TAG, "Apollo DN300 Ring controller")

apollo_dn300_ring_device::apollo_dn300_ring_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock):
    device_t(mconfig, APOLLO_DN300_RING, tag, owner, clock),
    m_cpu(*this, "cpu")
{
}

void
apollo_dn300_ring_device::device_start()
{
}

void
apollo_dn300_ring_device::device_reset()
{
}


void
apollo_dn300_ring_device::write(offs_t offset, uint8_t data, uint8_t mem_mask)
{
    SLOG1(("writing ring at offset %02x = %02x & %08x", offset, data, mem_mask));
}

uint8_t
apollo_dn300_ring_device::read(offs_t offset, uint8_t mem_mask)
{
	uint8_t data = 0;
    SLOG1(("reading ring at offset %02x & %08x = %02x", offset, mem_mask, data));
    return 0;
}

uint8_t
apollo_dn300_ring_device::rcv_header_read_byte(offs_t offset)
{
	uint8_t data = 0;
    SLOG1(("ring rcv_header_read_byte offset %02x = %02x", offset, data));
	return data;
}

void
apollo_dn300_ring_device::rcv_header_write_byte(offs_t offset, uint8_t data)
{
    SLOG1(("ring rcv_header_write_byte at offset %02x = %02x", offset, data));
}

uint8_t
apollo_dn300_ring_device::rcv_data_read_byte(offs_t offset)
{
	uint8_t data = 0;
    SLOG1(("ring rcv_data_read_byte offset %02x = %02x & %08x", offset, data));
	return data;
}

void
apollo_dn300_ring_device::rcv_data_write_byte(offs_t offset, uint8_t data)
{
    SLOG1(("ring rcv_data_write_byte at offset %02x = %02x", offset, data));
}

uint8_t
apollo_dn300_ring_device::transmit_read_byte(offs_t offset)
{
	uint8_t data = 0;
    SLOG1(("ring transmit_read_byte offset %02x = %02x & %08x", offset, data));
	return data;
}

void
apollo_dn300_ring_device::transmit_write_byte(offs_t offset, uint8_t data)
{
    SLOG1(("ring transmit_write_byte at offset %02x = %02x", offset, data));
}
