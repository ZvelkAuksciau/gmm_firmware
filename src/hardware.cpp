#include <hardware.hpp>
#include <cstring>

UUID readUniqueID()
{
    UUID out_bytes;
    memcpy(out_bytes.data(), reinterpret_cast<const void*>(0x1FFFF7E8), std::tuple_size<UUID>::value);
    return out_bytes;
}
