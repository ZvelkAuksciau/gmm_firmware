#pragma once

#include <array>
#include <cstdint>

typedef std::array<std::uint8_t, 12> UUID;
UUID readUniqueID();
