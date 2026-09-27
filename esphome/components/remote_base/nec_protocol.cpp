#include "nec_protocol.h"
#include "esphome/core/log.h"

namespace esphome::remote_base {

static const char *const TAG = "remote.nec";

static constexpr uint32_t HEADER_HIGH_US = 9000;
static constexpr uint32_t HEADER_LOW_US = 4500;
static constexpr uint32_t BIT_HIGH_US = 560;
static constexpr uint32_t BIT_ONE_LOW_US = 1690;
static constexpr uint32_t BIT_ZERO_LOW_US = 560;
static constexpr uint32_t BIT_REPEAT_LOW_US = 2250;

void NECProtocol::encode(RemoteTransmitData *dst, const NECData &data) {
  ESP_LOGD(TAG, "Sending NEC: address=0x%04X, command=0x%04X command_repeats=%d", data.address, data.command,
           data.command_repeats);

  dst->reserve(2 + 32 + 32 * data.command_repeats + 2 + data.repeat_frames * 4);
  dst->set_carrier_frequency(38000);

  dst->item(HEADER_HIGH_US, HEADER_LOW_US);

  for (uint16_t mask = 1; mask; mask <<= 1) {
    if (data.address & mask) {
      dst->item(BIT_HIGH_US, BIT_ONE_LOW_US);
    } else {
      dst->item(BIT_HIGH_US, BIT_ZERO_LOW_US);
    }
  }

  for (uint16_t repeats = 0; repeats < data.command_repeats; repeats++) {
    for (uint16_t mask = 1; mask; mask <<= 1) {
      if (data.command & mask) {
        dst->item(BIT_HIGH_US, BIT_ONE_LOW_US);
      } else {
        dst->item(BIT_HIGH_US, BIT_ZERO_LOW_US);
      }
    }
  }

  dst->mark(BIT_HIGH_US);

  if (data.repeat_frames > 0) {
    ESP_LOGD(TAG, "Sending NEC repeat frames (%d)", data.repeat_frames);

    // Begin the repeat frame sequence 40 ms after the command frame.
    // This will probably be a larger idle time than remote_receiver is configured for,
    // so don't expect it to be decoded together with the command frame later.
    dst->space(40000);

    for (uint16_t repeats = 0; repeats < data.repeat_frames - 1; repeats++) {
      dst->item(HEADER_HIGH_US, BIT_REPEAT_LOW_US);
      // Send repeat frames every 108 ms.
      dst->item(BIT_HIGH_US, 96000);
    }

    dst->item(HEADER_HIGH_US, BIT_REPEAT_LOW_US);
    dst->mark(BIT_HIGH_US);
  }
}
optional<NECData> NECProtocol::decode(RemoteReceiveData src) {
  NECData data{
      .address = 0,
      .command = 0,
      .command_repeats = 1,
      .repeat_frames = 0,
  };

  // Check if this is either a repeat frame or a command frame.
  // (Repeat frames are padded with sufficient idle time that receivers will probably treat them as separate messages.)
  if (src.expect_item(HEADER_HIGH_US, BIT_REPEAT_LOW_US) && src.expect_mark(BIT_HIGH_US)) {
    ESP_LOGI(TAG, "NEC repeat frame found. Deferring to raw output.");
    return {};
  } else if (src.expect_item(HEADER_HIGH_US, HEADER_LOW_US)) {
    for (uint16_t mask = 1; mask; mask <<= 1) {
      if (src.expect_item(BIT_HIGH_US, BIT_ONE_LOW_US)) {
        data.address |= mask;
      } else if (src.expect_item(BIT_HIGH_US, BIT_ZERO_LOW_US)) {
        data.address &= ~mask;
      } else {
        return {};
      }
    }

    for (uint16_t mask = 1; mask; mask <<= 1) {
      if (src.expect_item(BIT_HIGH_US, BIT_ONE_LOW_US)) {
        data.command |= mask;
      } else if (src.expect_item(BIT_HIGH_US, BIT_ZERO_LOW_US)) {
        data.command &= ~mask;
      } else {
        return {};
      }
    }

    while (src.peek_item(BIT_HIGH_US, BIT_ONE_LOW_US) || src.peek_item(BIT_HIGH_US, BIT_ZERO_LOW_US)) {
      uint16_t command = 0;
      for (uint16_t mask = 1; mask; mask <<= 1) {
        if (src.expect_item(BIT_HIGH_US, BIT_ONE_LOW_US)) {
          command |= mask;
        } else if (src.expect_item(BIT_HIGH_US, BIT_ZERO_LOW_US)) {
          command &= ~mask;
        } else {
          return {};
        }
      }

      // Make sure the extra/repeated data matches original command
      if (command != data.command) {
        return {};
      }

      data.command_repeats += 1;
    }

    src.expect_mark(BIT_HIGH_US);
    return data;
  } else {
    return {};
  }
}
void NECProtocol::dump(const NECData &data) {
  ESP_LOGI(TAG, "Received NEC: address=0x%04X, command=0x%04X command_repeats=%d", data.address, data.command,
           data.command_repeats);
}

}  // namespace esphome::remote_base
