//
// Created by tobias on 6/9/25.
//
#include "core/Spi.hpp"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

using namespace platform::wombat::core;

bool Spi::init(uint32_t speed_hz)
{
    if (m_fd >= 0) return true; // already open

    m_fd = ::open(SPI_DEVICE, O_RDWR | O_CLOEXEC);
    if (m_fd < 0)
    {
        std::perror("SPI open");
        return false;
    }

    const uint8_t mode = SPI_MODE_0, bits = 8;
    if (::ioctl(m_fd, SPI_IOC_WR_MODE, &mode) ||
        ::ioctl(m_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) ||
        ::ioctl(m_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz))
    {
        std::perror("SPI cfg");
        ::close(m_fd);
        m_fd = -1;
        return false;
    }

    /* prime TX buffer with protocol version */
    m_tx[TX_TRANSFER_VERSION] = TRANSFER_VERSION;

    m_tr = {
        .tx_buf = reinterpret_cast<unsigned long>(m_tx),
        .rx_buf = reinterpret_cast<unsigned long>(m_rx),
        .len = kBufSize,
        .speed_hz = speed_hz,
        .bits_per_word = bits
    };
    return true;
}

bool Spi::update()
{
    if (m_fd < 0) return false;
    if (::ioctl(m_fd, SPI_IOC_MESSAGE(1), &m_tr) < 0)
    {
        std::perror("SPI xfer");
        return false;
    }
    return m_rx[RX_TRANSFER_VERSION] == TRANSFER_VERSION;
}

Spi::~Spi()
{
    if (m_fd >= 0) ::close(m_fd);
}