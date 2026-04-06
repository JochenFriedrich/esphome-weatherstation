#include "weatherstation.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"
#include <cinttypes>

namespace esphome::weatherstation {

static const char *const TAG = "weatherstation";

// WeatherStationComponent

// WeatherStationProtocol

optional<WeatherStationData> WeatherStationProtocol::decode(remote_base::RemoteReceiveData src) {
  this->setup();

  WeatherStationData data;
  this->code_.resize(std::max((this->nbits_ + 7) >> 3, 8));
  uint32_t samples = (this->nbits_ + 1) * 2;                                    // sync + nbits
  uint32_t search_end = src.size() > samples ? src.size() - samples + 1 : 0ul;  // last possible sync + 1
  uint32_t search_limit = std::min(search_end, samples * 3 / 2);                // limit search

  while (src.get_index() < search_limit) {
    if (this->receive_item_(src, this->sync_high_, this->sync_low_)) {
      if (this->receive_code_(src)) {
        ESP_LOGD(TAG, "receive @%" PRIu32 " %" PRIx64 " (%d)", src.get_index(), *(uint64_t *) &this->code_[0],
                 this->nbits_);
        // found something, extend search till the end
        search_limit = search_end;
        // transform may also return false if it needs more packets to complete data
        if (this->to_data(data)) {
          ESP_LOGD(TAG, "id=%d b=%.0f ch=%d t=%.1f h=%d r=%.2f wd=%.1f ws=%.2f wg=%.2f", data.id, data.battery_level,
                   data.channel, data.temperature, data.humidity, data.rain, data.wind_direction_degrees,
                   data.wind_speed, data.wind_gust);
          return data;
        }
      }
    } else {
      src.advance(1);
    }
  }

  return {};
}

void WeatherStationProtocol::encode(remote_base::RemoteTransmitData *dst, const WeatherStationData &data) {
  this->setup();
  this->code_.resize(std::max((this->nbits_ + 7) >> 3, 8), 0);

  if (this->to_code(data)) {
    ESP_LOGD(TAG, "id=%d b=%.0f ch=%d t=%.1f h=%d r=%.2f wd=%.1f ws=%.2f wg=%.2f", data.id, data.battery_level,
             data.channel, data.temperature, data.humidity, data.rain, data.wind_direction_degrees, data.wind_speed,
             data.wind_gust);
    dst->set_carrier_frequency(38000);  // TODO: channel?
    for (int i = 0; i < this->repeat_; i++) {
      this->transmit_code_(dst);
    }
  }
}

void WeatherStationProtocol::dump(const WeatherStationData &data) {
  ESP_LOGI(TAG, "id=%d b=%.0f ch=%d t=%.1f h=%d r=%.2f wd=%.1f ws=%.2f wg=%.2f", data.id, data.battery_level,
           data.channel, data.temperature, data.humidity, data.rain, data.wind_direction_degrees, data.wind_speed,
           data.wind_gust);
}

bool WeatherStationProtocol::receive_item_(remote_base::RemoteReceiveData &src, uint32_t high, uint32_t low) const {
  if (!this->is_inverted_()) {
    if (!(this->is_ppm_() ? src.peek_mark_at_most(high, 0) : src.peek_mark(high, 0)))
      return false;
    if (!src.peek_space(low, 1))
      return false;
    src.advance(2);
  } else {
    if (src.get_index() == 0) {
      // assume space at the beginning
      if (!(this->is_ppm_() ? src.peek_mark_at_most(low, 0) : src.peek_mark(low, 0)))
        return false;
      src.advance(1);
    } else {
      if (!src.peek_space(high, 0))
        return false;
      if (!(this->is_ppm_() ? src.peek_mark_at_most(low, 1) : src.peek_mark(low, 1)))
        return false;
      src.advance(2);
    }
  }
  return true;
}

bool WeatherStationProtocol::receive_code_(remote_base::RemoteReceiveData &src) {
  uint8_t nbits = 0;

  while (nbits < this->nbits_ && src.get_index() < src.size() - 1) {
    size_t pos = !this->is_reversed_() ? nbits : this->nbits_ - nbits - 1;
    uint8_t bit = 1 << (pos & 7);
    uint8_t &dst = this->code_[pos >> 3];

    if (this->receive_item_(src, this->zero_high_, this->zero_low_)) {
      dst &= ~bit;
    } else if (this->receive_item_(src, this->one_high_, this->one_low_)) {
      dst |= bit;
    } else if (0 < this->nbits_min_ && this->nbits_min_ <= nbits) {
      return true;
    } else {
      break;
    }

    if (++nbits == this->nbits_) {
      if (src.get_index() < src.size() - 1) {
        uint32_t index = src.get_index();
        if (this->receive_item_(src, this->zero_high_, this->zero_low_) ||
            this->receive_item_(src, this->one_high_, this->one_low_)) {
          ESP_LOGD(TAG, "ignore %" PRIx64 " (%d)", *(uint64_t *) &this->code_[0], nbits);
          src.reset();
          src.advance(index);
          break;
        }
      }
      return true;
    }
  }

  return false;
}

void WeatherStationProtocol::transmit_item_(remote_base::RemoteTransmitData *dst, uint32_t high, uint32_t low) const {
  if (!this->is_inverted_()) {
    dst->mark(high);
    dst->space(low);
  } else {
    dst->space(high);
    dst->mark(low);
  }
}

void WeatherStationProtocol::transmit_code_(remote_base::RemoteTransmitData *dst) const {
  this->transmit_item_(dst, this->sync_high_, this->sync_low_);
  for (uint8_t j = 0; j < this->nbits_; j++) {
    uint8_t i = this->is_reversed_() ? j : this->nbits_ - j - 1;
    if (this->code_[i >> 3] & (1 << (i & 7))) {
      this->transmit_item_(dst, this->one_high_, this->one_low_);
    } else {
      this->transmit_item_(dst, this->zero_high_, this->zero_low_);
    }
  }
}

//

uint32_t WeatherStationProtocol::get_bits_(uint8_t pos, uint8_t nbits) const {
  if (pos + nbits > this->code_.size() * 8) {
    ESP_LOGE(TAG, "get_bits_ out of range");
    return 0;
  }

  // if (zero_pos_at_msb) {
  pos = this->nbits_ - (pos + nbits);
  // }

  uint32_t c = 0;

  if ((pos & 7) == 0) {
    for (uint8_t i = 0; i < nbits; i += 8, pos += 8) {
      c |= (uint32_t) this->code_[pos >> 3] << i;
    }
    if ((nbits & 7) != 0) {
      c &= 0xffffffff >> (32 - nbits);
    }
  } else {
    for (uint8_t i = 0; i < nbits; i++, pos++) {
      if (this->code_[pos >> 3] & (1 << (pos & 7)))
        c |= (uint32_t) 1 << i;
    }
  }

  return c;
}

void WeatherStationProtocol::set_bits_(uint8_t pos, uint8_t nbits, uint32_t c) {
  // TODO
}

// WS2032

// Not sure about the different models, sold on Aliexpress

void WeatherStation2032Protocol::setup() {
  this->sync_high_ = 2000;
  this->sync_low_ = 500;
  this->zero_high_ = 500;
  this->zero_low_ = 500;
  this->one_high_ = 1000;
  this->one_low_ = 500;
  this->nbits_ = 111;
  this->repeat_ = 3;
  this->flags_ = TYPE_PPM | REVERSED;
}

bool WeatherStation2032Protocol::to_data(WeatherStationData &data) const {
  uint8_t chksum = 0;
  for (uint8_t i = 0, pos = 0; i < 12; i++, pos += 8) {
    chksum += this->get_bits_(pos, 8);
  }
  if (this->get_bits_(96, 8) != chksum) {
    ESP_LOGV(TAG, "chksum mismatch %02X != %02X", (uint8_t) this->get_bits_(96, 8), chksum);
    return false;
  }

  // TODO: crc8

  // PRE = this->get_bits_(0, 8)
  data.id = this->get_bits_(8, 16);
  data.battery_level = (this->get_bits_(24, 8) & 1) ? 100.0f : 0;  // TODO: other flags?
  // FLAG = this->get_bits_(24, 8)
  data.wind_direction_degrees = 22.5f * this->get_bits_(32, 4);
  data.temperature = (this->get_bits_(36, 1) ? -0.1f : 0.1f) * this->get_bits_(37, 11);
  data.humidity = this->get_bits_(48, 8);
  data.wind_speed = 0.43f * this->get_bits_(56, 8);
  data.wind_gust = 0.43f * this->get_bits_(64, 8);
  data.rain = (float) this->get_bits_(72, 24);  // conversion to mm?
  // ? = this->get_bits_(104, 7)

  return true;
}

bool WeatherStation2032Protocol::to_code(const WeatherStationData &data) {
  ESP_LOGD(TAG, "TODO: encode WS2032");
  return false;
}

// Lidl Auriol 4LD*

// 4-LD5661
// 4-LD5972
// 4-LD6313
// 4-LD6654
// ... there are many models
// some don't have a humidity sensor, that field will be zero

// receive @105 67a0d0f2e0000 (52)
// id=103 b=100 ch=2 t=20.8 h=46 r=0.00 wd=0.0 ws=0.00 wg=0.00
// 0110 0111 1010 0000 1101 0000 1111 0010 1110 0000 0000 0000 0000
// ididididi b?ch temptemptempte 1111 humihumih rainrainrainrainrai

void WeatherStation4LDProtocol::setup() {
  this->sync_high_ = 4000;
  this->sync_low_ = 500;
  this->zero_high_ = 1000;
  this->zero_low_ = 500;
  this->one_high_ = 2000;
  this->one_low_ = 500;
  this->nbits_ = 52;
  this->repeat_ = 7;
  this->flags_ = TYPE_PPM | INVERTED | REVERSED;
}

bool WeatherStation4LDProtocol::to_data(WeatherStationData &data) const {
  if ((uint8_t) this->get_bits_(24, 4) != 0b1111) {  // unknown, always 0b1111?
    ESP_LOGV(TAG, "[24:27] should be 0b1111");
    return false;
  }

  data.id = (uint8_t) this->get_bits_(0, 8);
  data.battery_level = (uint8_t) this->get_bits_(8, 1) == 1 ? 100.0f : 0;
  // 0 = (uint8_t) this->get_bits_(9, 1); // ?
  data.channel = (uint8_t) this->get_bits_(10, 2);
  data.temperature = (float) ((int16_t) (this->get_bits_(12, 12) << 4)) / 160;
  data.humidity = (uint8_t) this->get_bits_(28, 8);
  data.rain = (float) this->get_bits_(36, 16) * 0.242f;
  return true;
}

bool WeatherStation4LDProtocol::to_code(const WeatherStationData &data) {
  this->set_bits_(0, 8, data.id);
  this->set_bits_(8, 1, data.battery_level > 25 ? 1 : 0);  // 25% is pretty dead
  this->set_bits_(9, 1, 0);
  this->set_bits_(10, 2, data.channel);
  this->set_bits_(12, 12, (uint64_t) ((int16_t) (data.temperature * 160) >> 4));
  this->set_bits_(24, 4, 0b1111);
  this->set_bits_(28, 8, data.humidity);
  this->set_bits_(36, 16, (uint64_t) (data.rain / 0.242f));
  return true;
}

// AHFL

void WeatherStationAHFLProtocol::setup() {
  this->sync_high_ = 600;
  this->sync_low_ = 9000;
  this->zero_high_ = 600;
  this->zero_low_ = 2100;
  this->one_high_ = 600;
  this->one_low_ = 4200;
  this->nbits_ = 42;
  this->repeat_ = 8;
  this->flags_ = TYPE_PPM | REVERSED;
}

bool WeatherStationAHFLProtocol::to_data(WeatherStationData &data) const {
  uint8_t chksum = this->get_bits_(32, 4);
  if (chksum != 0b0100) {
    ESP_LOGV(TAG, "[6:9] should be 0b0100");
    return false;
  }
  for (int i = 0; i < 32; i += 4) {
    chksum += (uint8_t) this->get_bits_(i, 4);
  }
  chksum &= 0x3f;
  if (chksum != this->get_bits_(36, 6)) {
    ESP_LOGV(TAG, "chksum mismatch %02X %02X", (uint8_t) this->get_bits_(36, 6), chksum);
    return false;
  }

  data.id = (uint16_t) this->get_bits_(0, 8);
  data.battery_level = (uint8_t) this->get_bits_(8, 1) == 0 ? 100.0f : 0;
  data.channel = (uint8_t) this->get_bits_(10, 2) + 1;
  data.temperature = (float) ((int16_t) (this->get_bits_(12, 12) << 4)) / 160;
  data.humidity = (uint8_t) this->get_bits_(24, 7);
  return true;
}

bool WeatherStationAHFLProtocol::to_code(const WeatherStationData &data) {
  ESP_LOGD(TAG, "TODO: encode ahfl");
  return true;
}

// Bresser3CH

void WeatherStationBresser3CHProtocol::setup() {
  this->sync_high_ = 750;
  this->sync_low_ = 750;
  this->zero_high_ = 250;
  this->zero_low_ = 500;
  this->one_high_ = 500;
  this->one_low_ = 250;
  this->nbits_ = 40;
  this->repeat_ = 8;
  this->flags_ = TYPE_PWM;
}

bool WeatherStationBresser3CHProtocol::to_data(WeatherStationData &data) const {
  uint8_t chksum = 0;
  for (int i = 0; i < 4; i++) {
    chksum = (chksum + (uint8_t) this->get_bits_(i * 8, 8));
  }
  chksum &= 0xff;
  if ((uint8_t) this->get_bits_(32, 8) != chksum) {
    ESP_LOGV(TAG, "chksum mismatch %x != %x", (uint8_t) this->get_bits_(32, 8), chksum);
    return false;
  }

  data.id = (uint8_t) this->get_bits_(0, 8);
  data.battery_level = (uint8_t) this->get_bits_(8, 1) == 0 ? 100.0f : 0;
  // user initiated transimission = this->get_bits_(9, 1);
  data.channel = (uint8_t) this->get_bits_(10, 2);  // is ch0 a valid option?
  data.temperature = (float) this->get_bits_(12, 12) * 0.1f - 90;
  data.temperature = (data.temperature - 32.0f) * 5.0f / 9.0f;  // F to C
  data.humidity = (uint8_t) this->get_bits_(24, 8);
  return true;
}

bool WeatherStationBresser3CHProtocol::to_code(const WeatherStationData &data) {
  ESP_LOGD(TAG, "TODO: encode Bresser3CH");
  return true;
}

// Eurochron

void WeatherStationEurochronProtocol::setup() {
  this->sync_high_ = 500;
  this->sync_low_ = 4000;
  this->zero_high_ = 500;
  this->zero_low_ = 1000;
  this->one_high_ = 500;
  this->one_low_ = 2000;
  this->nbits_ = 36;
  this->repeat_ = 8;
  this->flags_ = TYPE_PPM | REVERSED;
}

bool WeatherStationEurochronProtocol::to_data(WeatherStationData &data) const {
  // this may catch Nexus packets, if the 3rd nibblet is 1111, temperature is simply negative

  uint8_t flags = this->get_bits_(8, 8);
  if ((flags & 0b01101111) != 0) {
    ESP_LOGV(TAG, "[9:10] and [12:15] should be 0");
    return false;
  }

  data.id = (uint8_t) this->get_bits_(0, 8);
  data.battery_level = (uint8_t) this->get_bits_(8, 1) == 0 ? 100.0f : 0;
  // user initiated transimission = this->get_bits_(11, 1);
  data.humidity = (uint8_t) this->get_bits_(16, 8);
  data.temperature = (float) ((int16_t) (this->get_bits_(24, 12) << 4)) / 160;
  return true;
}

bool WeatherStationEurochronProtocol::to_code(const WeatherStationData &data) {
  ESP_LOGD(TAG, "TODO: encode Eurochron");
  return true;
}

// H10515

// Lidl H10515/DCF

// receive @74 d990a3469 (36)
// 1101 1001 1001 0000 1010 0011 0100 0110 1001
// chks humihumih Stemptemptempt 00?0 ??ch idid
// id=9 b=0 ch=1 t=16.3 h=99 r=0.00 wd=0.0 ws=0.00 wg=0.00
// ch bits are reversed

void WeatherStationH10515Protocol::setup() {
  this->sync_high_ = 500;
  this->sync_low_ = 9000;
  this->zero_high_ = 500;
  this->zero_low_ = 2000;
  this->one_high_ = 500;
  this->one_low_ = 4000;
  this->nbits_ = 36;
  this->repeat_ = 8;
  this->flags_ = TYPE_PPM;
}

bool WeatherStationH10515Protocol::to_data(WeatherStationData &data) const {
  uint8_t chksum = 0;
  for (int i = 1; i < 9; i++) {
    chksum = (chksum + (uint8_t) this->get_bits_(i * 4, 4)) & 0b1111;
  }
  chksum = ~chksum & 0b1111;
  if ((uint8_t) this->get_bits_(0, 4) != chksum) {
    ESP_LOGV(TAG, "chksum mismatch %x != %x", (uint8_t) this->get_bits_(0, 4), chksum);
    return false;
  }

  if (this->get_bits_(12, 12) == 0xdff) {  // sometimes initially it displays LL and sends this
    return false;
  }

  if (this->get_bits_(24, 4) != 0) {  // unknown, always zero(?), sometimes 0b0010 or 0b0100
    ESP_LOGV(TAG, "[24:27] should be 0");
    // return false;
  }

  data.humidity = 10.0f * this->get_bits_(4, 4) + this->get_bits_(8, 4);
  data.temperature = (this->get_bits_(12, 1) ? -0.1f : 0.1f) * this->get_bits_(13, 11);
  // ? == this->get_bits_(28, 2) // unknown, battery(?)
  data.channel = (uint8_t) ((this->get_bits_(31, 1) << 1) | this->get_bits_(30, 1));
  if (data.channel == 0) {
    ESP_LOGV(TAG, "channel invalid %d", data.channel);
    return false;
  }
  data.id = (uint8_t) this->get_bits_(32, 4);  // keeps changing between resets
  return true;
}

bool WeatherStationH10515Protocol::to_code(const WeatherStationData &data) {
  ESP_LOGD(TAG, "TODO: encode H10515");
  return true;
}

// H13726

// Lidl Auriol H13726
// Ventus WS155,
// Hama EWS 1500,
// Meteoscan W155/W160
// https://github.com/gabest11/datasheet/blob/main/auriol_protocol_v20.pdf
// Unitec W186-F
// https://github.com/merbanan/rtl_433/blob/master/src/devices/alecto.c

// Rain (TT=11 ttt=011)
// receive @76 400023602 (36)
// id=2 b=100 ch=0 t=0.0 h=0 r=0.50 wd=0.0 ws=0.00 wg=0.00
// 0100 0000 0000 0000 0010 0011 0110 0000 0010
// chks rainrainrainrainrai ?ttt uTTb ididididi

// BAD temp&humi (H10515 data, chksum okay, TODO: reject)
// E99DFF06E
// 1110 1001 1001 1101 1111 1111 0000 0110 1110
// chks humihumih temptemptempte uTTb ididididi

void WeatherStationH13726Protocol::setup() {
  this->sync_high_ = 500;
  this->sync_low_ = 9000;
  this->zero_high_ = 500;
  this->zero_low_ = 2000;
  this->one_high_ = 500;
  this->one_low_ = 4000;
  this->nbits_ = 36;
  this->repeat_ = 8;
  this->flags_ = TYPE_PPM;
}

bool WeatherStationH13726Protocol::to_data(WeatherStationData &data) const {
  // only the rain sensor was tested, the other part is beyond repair, it has spent 20 years outside

  uint8_t chksum1 = 0b1111;
  uint8_t chksum2 = 0b0111;

  for (int i = 1; i < 9; i++) {
    uint8_t b = (uint8_t) this->get_bits_(i * 4, 4);
    chksum1 = (chksum1 - b) & 0b1111;
    chksum2 = (chksum2 + b) & 0b1111;
  }

  uint8_t chksum = (uint8_t) this->get_bits_(0, 4);

  if (this->get_bits_(25, 2) != 0b11) {  // temperature and humidity
    if (chksum != chksum1) {
      ESP_LOGV(TAG, "chksum1 mismatch %x != %x", chksum, chksum1);
      return false;
    }
    data.humidity = 10.0f * this->get_bits_(4, 4) + this->get_bits_(8, 4);
    data.temperature = (float) ((int16_t) (this->get_bits_(12, 12) << 4)) / 160;
    // TODO: wind packets may also be in this transmission burst, not sure
    // return false;
  } else {
    uint8_t type = this->get_bits_(21, 3);
    if (type == 0b011) {  // rain
      // D000B3602 2.75mm
      // 1101 0000 0000 0000 1011 0011 0110 0000 0010
      if (chksum != chksum2) {
        ESP_LOGV(TAG, "chksum2 mismatch %x != %x", chksum, chksum2);
        return false;
      }
      if (this->get_bits_(20, 1) != 0) {  // always 0?
        return false;
      }
      data.rain = (float) this->get_bits_(4, 16) * 0.25f;
    } else {
      if (chksum != chksum1) {
        ESP_LOGV(TAG, "chksum1 mismatch %x != %x", chksum, chksum1);
        return false;
      }
      if (type == 0b001) {  // wind part 1
        data.wind_speed = (float) this->get_bits_(4, 8) * 0.2f;
        // we still need to see the "wind part 2"
        return false;
      } else if (type == 0b111) {  // wind part 2
        data.wind_gust = (float) this->get_bits_(4, 8) * 0.2f;
        data.wind_direction_degrees = this->get_bits_(12, 9);
      } else {
        return false;
      }
    }
  }

  data.id = this->get_bits_(28, 8);  // keeps changing between resets
  data.battery_level = this->get_bits_(27, 1) == 0 ? 100.0f : 0;
  // user initiated transimission = this->get_bits_(24, 1) == 1;
  return true;
}

bool WeatherStationH13726Protocol::to_code(const WeatherStationData &data) {
  ESP_LOGD(TAG, "TODO: encode h13726");
  return true;
}

// L08037A

// Lidl 40782 L08037A

// receive @58 a0d0dd4 (28)
// id=13 b=0 ch=1 t=22.1 h=0 r=0.00 wd=0.0 ws=0.00 wg=0.00
// 1010 0000 1101 0000 1101 1101 0100
// chkc ididididi temptemptempte ch??

void WeatherStationL08037AProtocol::setup() {
  this->sync_high_ = 500;
  this->sync_low_ = 9500;
  this->zero_high_ = 500;
  this->zero_low_ = 2000;
  this->one_high_ = 500;
  this->one_low_ = 4500;
  this->nbits_ = 28;
  this->repeat_ = 8;
  this->flags_ = TYPE_PPM | REVERSED;
}

bool WeatherStationL08037AProtocol::to_data(WeatherStationData &data) const {
  uint8_t chksum = 0b1111;
  for (int i = 1; i < 7; i++) {
    chksum = (chksum + (uint8_t) this->get_bits_(i * 4, 4)) & 0b1111;
  }
  if ((uint8_t) this->get_bits_(0, 4) != chksum) {
    ESP_LOGV(TAG, "chksum mismatch %x != %x", (uint8_t) this->get_bits_(0, 4), chksum);
    return false;
  }

  data.id = (uint8_t) this->get_bits_(4, 8);
  data.channel = (uint8_t) this->get_bits_(24, 2);
  if (data.channel == 0) {
    ESP_LOGV(TAG, "channel invalid %d", data.channel);
    return false;
  }
  data.temperature = (float) ((int16_t) (this->get_bits_(12, 12) << 4)) / 160;
  return true;
}

bool WeatherStationL08037AProtocol::to_code(const WeatherStationData &data) {
  ESP_LOGD(TAG, "TODO: encode l08037a");
  return true;
}

// NEXUS

void WeatherStationNexusProtocol::setup() {
  this->sync_high_ = 500;
  this->sync_low_ = 4000;
  this->zero_high_ = 500;
  this->zero_low_ = 1000;
  this->one_high_ = 500;
  this->one_low_ = 2000;
  this->nbits_ = 38;
  this->nbits_min_ = 36;
  this->repeat_ = 8;
  this->flags_ = TYPE_PPM | REVERSED;
}

bool WeatherStationNexusProtocol::to_data(WeatherStationData &data) const {
  // the beginning of Lidl Auriol 4LD can be accepted here without chksum, it is only filtered out by checking the
  // presence of additional valid bits before the next sync

  uint8_t chk = this->get_bits_(24, 4);
  if (chk != 0b1111 && chk != 0b1010) {
    ESP_LOGV(TAG, "[24:27] should be 0b1111 or 0b1010");
    return false;
  }

  data.id = (uint8_t) this->get_bits_(0, 8);
  data.battery_level = (uint8_t) this->get_bits_(8, 1) == 1 ? 100.0f : 0;
  data.channel = (uint8_t) this->get_bits_(10, 2) + 1;
  data.temperature = (float) ((int16_t) (this->get_bits_(12, 12) << 4)) / 160;
  data.humidity = (uint8_t) this->get_bits_(28, 8);
  return true;
}

bool WeatherStationNexusProtocol::to_code(const WeatherStationData &data) {
  ESP_LOGD(TAG, "TODO: encode nexus");
  return true;
}

// Z31743

void WeatherStationZ31743Protocol::setup() {
  this->sync_high_ = 400;
  this->sync_low_ = 9500;
  this->zero_high_ = 400;
  this->zero_low_ = 2000;
  this->one_high_ = 400;
  this->one_low_ = 4000;
  this->nbits_ = 32;
  this->repeat_ = 8;
  this->flags_ = TYPE_PPM | REVERSED;
}

bool WeatherStationZ31743Protocol::to_data(WeatherStationData &data) const {
  uint8_t msg[4];
  for (int i = 0; i < 3; i++) {
    msg[i] = (uint8_t) this->get_bits_(i * 8, 8);
  }
  msg[3] = 0;
  uint8_t crc = 0;
  for (uint8_t b : msg) {
    crc ^= b;
    for (int bit = 0; bit < 8; bit++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x31;
      } else {
        crc = (crc << 1);
      }
    }
  }
  if (crc != (uint8_t) this->get_bits_(24, 8)) {
    ESP_LOGV(TAG, "chksum mismatch %02X %02X", (uint8_t) this->get_bits_(24, 8), crc);
    return false;
  }

  data.id = (uint16_t) this->get_bits_(0, 8);
  data.battery_level = (uint8_t) this->get_bits_(8, 1) == 1 ? 100.0f : 0;
  data.temperature = (float) ((int16_t) (this->get_bits_(12, 12) << 4)) / 160;
  return true;
}

bool WeatherStationZ31743Protocol::to_code(const WeatherStationData &data) {
  ESP_LOGD(TAG, "TODO: encode z31743");
  return true;
}

// Z32171

void WeatherStationZ32171Protocol::setup() {
  this->sync_high_ = 500;
  this->sync_low_ = 8000;
  this->zero_high_ = 500;
  this->zero_low_ = 2000;
  this->one_high_ = 500;
  this->one_low_ = 4000;
  this->nbits_ = 40;
  this->repeat_ = 8;
  this->flags_ = TYPE_PPM | REVERSED;
}

bool WeatherStationZ32171Protocol::to_data(WeatherStationData &data) const {
  uint8_t msg[4];
  for (int i = 0; i < 4; i++) {
    msg[i] = (uint8_t) this->get_bits_(8 * i, 8);
  }
  // for CRC computation, channel bits are at the CRC position
  msg[1] = (msg[1] & 0x0F) | (uint8_t) this->get_bits_(36, 4) << 4;

  uint8_t crc = 0;
  for (uint8_t b : msg) {
    crc ^= b;
    for (int bit = 0; bit < 8; bit++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x30;
      } else {
        crc = (crc << 1);
      }
    }
  }
  crc = (crc >> 4) ^ (uint8_t) this->get_bits_(32, 4);
  if (crc != (uint8_t) this->get_bits_(8, 4)) {
    ESP_LOGV(TAG, "chksum mismatch %02X %02X", (uint8_t) this->get_bits_(8, 4), crc);
    return false;
  }

  data.id = (uint16_t) this->get_bits_(0, 8);
  data.battery_level = (uint8_t) this->get_bits_(13, 1) == 0 ? 100.0f : 0;
  data.temperature = (float) ((int16_t) (this->get_bits_(16, 12) - 1220) * 5 / 90.0);
  data.humidity = (uint8_t) this->get_bits_(28, 4) * 10 + (uint8_t) this->get_bits_(32, 4);
  data.channel = (uint8_t) this->get_bits_(38, 2);
  return true;
}

bool WeatherStationZ32171Protocol::to_code(const WeatherStationData &data) {
  ESP_LOGD(TAG, "TODO: encode z32171");
  return true;
}

}  // namespace esphome::weatherstation
