#pragma once

template <typename T>
inline bool aerialkit_serial_io_ready(T *const *io) noexcept
{
    return io != nullptr && *io != nullptr;
}
