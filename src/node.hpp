#ifndef NODE_HPP
#define NODE_HPP

#include <uavcan/uavcan.hpp>
#include <ch.hpp>

namespace Node {

  constexpr unsigned NodePoolSize = 2048;
  uavcan::Node<NodePoolSize>& getNode();
  void publishKeyValue(const char *key, float value);

  void init(uint32_t bit_rate,
          uint8_t node_id,
          uint8_t firmware_vers_major,
          uint8_t firmware_vers_minor,
          uint32_t vcs_commit,
          uint64_t crc64);

  class uavcanNodeThread : public chibios_rt::BaseStaticThread<4096> {
    public:
      void main();
  };

}

#endif

