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
  uint32_t samples = (this->nbits_ + 1) * 2;  // sync + nbits
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

// ===========================================================================
// WeatherStationHidekiProtocol
//
// Hideki Electronics OEM protocol, used by Cresta, Mebus, Irox, Honeywell,
// RST and others on 433 MHz.
//
// Signal characteristics:
//   - No leading sync pulse.  The clock period is self-timed: the first
//     received edge defines half the clock period (clockTime = duration/2).
//   - Manchester encoded: a short edge (~clockTime) keeps the current bit
//     value; a long edge (~2*clockTime) flips it.  "Short" means within
//     [0.5*clockTime, 1.5*clockTime]; "long" means within
//     [1.5*clockTime, 3.0*clockTime].
//   - The first edge always begins as isOne = true.
//   - 9 bits per byte: 8 data bits (LSB first) + 1 stop bit (always 0).
//   - First byte must decrypt to 0x75 (the fixed header / preamble byte).
//   - Byte 3 (after decryption) encodes the payload length in bits [5:1].
//   - Two checksum bytes follow the payload.
//   - XOR encryption: transmitted_byte = encryptByte(plain_byte), where
//     encryptByte folds all bits of the byte with XOR as a running sum.
//     Decryption is the same transform: plain ^= plain << 1.
//   - cs1 = XOR of all transmitted bytes [1..n+1]  (must be 0 after decode)
//   - cs2 = secondCheck polynomial over the same range
//
// Packet layout (thermo/hygro, decrypted):
//   [0]  0x75              header
//   [1]  (ch<<5)|id        channel (1-5) and random device ID
//   [2]  0xce              size byte  → packageLength = (0xce>>1)&0x1f = 7
//   [3]  0x1e              sensor type: thermo/hygro
//   [4]  (T_tens<<4)|T_units   temperature BCD digits (always positive)
//   [5]  sign_nibble|T_hundreds  sign: 0xc = positive, 0x4 = negative
//   [6]  (H_tens<<4)|H_units    humidity BCD
//   [7]  0xff              comfort flag
//   [8]  cs1               XOR checksum  (of encrypted bytes 1..8)
//   [9]  cs2               polynomial checksum
// ===========================================================================

// ---------------------------------------------------------------------------
// secondCheck: the polynomial checksum step used by both encrypt and decrypt.
// Mirrors SensorTransmitter::secondCheck / SensorReceiver::secondCheck.
// ---------------------------------------------------------------------------
uint8_t WeatherStationHidekiProtocol::second_check_(uint8_t b) {
  uint8_t c;
  if (b & 0x80)
    b ^= 0x95;
  c = b ^ (b >> 1);
  if (b & 1)
    c ^= 0x5f;
  if (c & 1)
    b ^= 0x5f;
  return b ^ (c >> 1);
}

// ---------------------------------------------------------------------------
// decrypt_byte_: inverse of encryptByte.
// encryptByte folds all bits with running XOR: a ^= b (shifting b left).
// The inverse is simply plain ^= plain << 1 applied in place, which is what
// SensorReceiver::decryptAndCheck does: data[i] ^= data[i] << 1.
// ---------------------------------------------------------------------------
uint8_t WeatherStationHidekiProtocol::decrypt_byte_(uint8_t b) {
  return b ^ (b << 1);
}

// ---------------------------------------------------------------------------
// decrypt_and_check_: verify checksums and decrypt the buffer in place.
// buf[0]           = header (not checksummed)
// buf[1..n+1]      = encrypted payload bytes  (packageLength + 1 bytes)
// buf[n+2]         = cs1 (XOR of encrypted bytes 1..n+1)
// buf[n+3]         = cs2 (polynomial checksum)
//
// Returns true if both checksums pass.  On success the buffer bytes
// buf[1..n+1] are decrypted in place.
// ---------------------------------------------------------------------------
bool WeatherStationHidekiProtocol::decrypt_and_check_(uint8_t *buf, uint8_t package_length) const {
  uint8_t cs1 = 0;
  uint8_t cs2 = 0;
  // packageLength is the number of data bytes after the size byte, i.e.
  // indices 1 .. packageLength+1 inclusive are the encrypted payload.
  for (uint8_t i = 1; i < package_length + 2; i++) {
    cs1 ^= buf[i];
    cs2 = second_check_(buf[i] ^ cs2);
    buf[i] = decrypt_byte_(buf[i]);  // decrypt in place
  }
  if (cs1 != 0) {
    ESP_LOGV(TAG, "Hideki cs1 fail (XOR != 0, got %02X)", cs1);
    return false;
  }
  if (cs2 != buf[package_length + 2]) {
    ESP_LOGV(TAG, "Hideki cs2 fail (%02X != %02X)", cs2, buf[package_length + 2]);
    return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
// decode_thermo_hygro_: extract temperature and humidity from a decrypted
// thermo/hygro packet (type byte 0x1e).
// ---------------------------------------------------------------------------
bool WeatherStationHidekiProtocol::decode_thermo_hygro_(const uint8_t *buf, WeatherStationData &data) const {
  // buf[3] == 0x1e for thermo/hygro
  if ((buf[3] & 0x1f) != 0x1e) {
    ESP_LOGV(TAG, "Hideki: not a thermo/hygro packet (type=%02X)", buf[3]);
    return false;
  }

  // Temperature: stored as three BCD digits spread over bytes 4 and 5.
  //   buf[4] = (tens << 4) | units
  //   buf[5] = sign_nibble | hundreds
  //     sign_nibble 0xc → positive, 0x4 → negative
  int temp_raw = 100 * (buf[5] & 0x0f) + 10 * (buf[4] >> 4) + (buf[4] & 0x0f);
  bool negative = !(buf[5] & 0x80);  // bit 7 of byte 5 = sign (1 = positive)
  data.temperature = (negative ? -1.0f : 1.0f) * (float) temp_raw * 0.1f;

  // Humidity: BCD in buf[6]
  data.humidity = (uint8_t) (10 * (buf[6] >> 4) + (buf[6] & 0x0f));

  // Channel: upper 3 bits of buf[1].  Channels 5 and 6 map to physical 4 and 5
  // (channel 4 is reserved internally for rain/uv/anemo sensor type).
  uint8_t ch = buf[1] >> 5;
  if (ch >= 5)
    ch--;
  data.channel = ch;

  // Random device ID: lower 5 bits of buf[1]
  data.id = buf[1] & 0x1f;

  // Battery status is not exposed in the thermo/hygro packet; leave as default.

  return true;
}

// ---------------------------------------------------------------------------
// decode: main entry point called by ESPHome's remote_receiver infrastructure.
//
// The Hideki protocol does not fit the WeatherStationProtocol pulse-counting
// model because:
//   (a) there is no fixed sync pulse — clock is self-detected from edge 0,
//   (b) bits are encoded in edge *duration* relative to a dynamic clock, and
//   (c) each byte has 9 bits with a mandatory stop bit.
//
// We therefore override decode() directly and perform a state-machine style
// walk over the RemoteReceiveData pulse train.
// ---------------------------------------------------------------------------
optional<WeatherStationData> WeatherStationHidekiProtocol::decode(remote_base::RemoteReceiveData src) {
  // We need at least a few edges to be worth trying.
  if (src.size() < 20)
    return {};

  uint8_t buf[MAX_BYTES];

  // -------------------------------------------------------------------------
  // Outer loop: try each start position in the pulse train.
  // -------------------------------------------------------------------------
  for (uint32_t start = 0; start + 20 < src.size(); start++) {
    src.reset();
    src.advance(start);

    memset(buf, 0, sizeof(buf));

    // -----------------------------------------------------------------------
    // Clock detection.
    //
    // The Hideki start bit is always one half-period LOW followed by one
    // half-period HIGH (500 µs each).  The start bit LOW is the shortest
    // possible pulse in the frame — it never merges with a neighbour because
    // it is preceded by idle and the following HIGH half may merge with the
    // first data bit.  We therefore detect the clock by finding the shortest
    // pulse in a lookahead window rather than blindly halving the first pulse.
    //
    // Additionally, ESPHome's RemoteReceiveData alternates mark(+)/space(-)
    // starting with a mark.
    // The first half of the start bit can't be detected as it is the idle
    // level. So just assume, it's there.
    // -----------------------------------------------------------------------
    int32_t leading = src.peek(0);
    if (leading < 0) {
      // We need to start at the second half of the start bit — not a valid Hideki start.
      continue;
    }
    uint32_t clock_time = (uint32_t) (leading / 2);
    if (clock_time < 200 || clock_time > 1200) {
      continue;
    }

    // -----------------------------------------------------------------------
    // Inner loop: walk the pulse stream, tracking wf_pos (waveform position).
    //
    // wf_pos is an abstract counter that mirrors the half-bit index in the
    // transmitted waveform:
    //   wf_pos 0   = start bit first half  (LOW,  one half-period) - not used
    //   wf_pos 1   = start bit second half (HIGH, one half-period)
    //   wf_pos 2   = data byte 0, bit 0, first  half  ← data region begins
    //   wf_pos 3   = data byte 0, bit 0, second half
    //   wf_pos 4   = data byte 0, bit 1, first  half
    //   ...
    //   wf_pos 2 + 18*B + 2*b     = byte B, bit b (0..7), first  half
    //   wf_pos 2 + 18*B + 2*b + 1 = byte B, bit b (0..7), second half
    //   wf_pos 2 + 18*B + 16      = byte B, stop bit, first  half  (must be LOW)
    //   wf_pos 2 + 18*B + 17      = byte B, stop bit, second half
    //
    // The level of each half-bit is given directly by the sign of the pulse
    // that contains it: positive (mark) = HIGH = 1, negative (space) = LOW = 0.
    // No edge-toggling is needed — ESPHome reports the actual signal level.
    //
    // Each pulse spans either 1 or 2 half-bits (short or long edge).  We
    // expand every pulse into its constituent half-bits and use wf_pos to
    // decide what to do with each one.
    // -----------------------------------------------------------------------
    uint32_t wf_pos = 1;
    uint8_t package_length = 0;
    bool got_length = false;
    bool valid = true;
    // wf_pos at which the last expected half-bit arrives (set once length known).
    uint32_t wf_pos_max = 0;

    while (src.get_index() < src.size()) {
      int32_t raw_dur = src.peek(0);
      if (raw_dur == 0) {
        src.advance(1);
        continue;
      }
      src.advance(1);

      uint32_t duration = (uint32_t) (raw_dur < 0 ? -raw_dur : raw_dur);
      uint8_t  level    = (raw_dur > 0) ? 1u : 0u;  // HIGH if mark, LOW if space

      // How many half-bits does this pulse span?  Must be 1 or 2.
      uint32_t n = (duration + (clock_time >> 1)) / clock_time;  // round to nearest
      if (wf_pos == 0)
        n = 1;
      if (n < 1 || n > 2) {
        valid = false;
        break;
      }

      // Process each half-bit in this pulse.
      for (uint32_t h = 0; h < n; h++, wf_pos++) {
        if (wf_pos < 2) {
          // Start bit: wf_pos 0 (LOW) and 1 (HIGH).  Just validate.
          uint8_t expected_level = (wf_pos == 0) ? 0u : 1u;
          if (level != expected_level) {
            valid = false;
          }
          continue;
        }

        // Data region.
        uint32_t data_half  = wf_pos - 2;
        uint32_t bit_index  = data_half >> 1;          // overall bit number (includes stop bits)
        bool     is_first   = (data_half & 1) == 0;   // first or second half of this bit
        uint32_t byte_num   = bit_index / 9;           // which byte (9 bits per byte incl stop)
        uint32_t bit_in_byte = bit_index % 9;          // bit position within the byte (0..8)

        if (byte_num >= MAX_BYTES) {
          valid = false;
          break;
        }

        if (bit_in_byte < 8) {
          // Normal data bit: first half's level = the bit value.
          if (is_first) {
            if (level) {
              buf[byte_num] |= (1u << bit_in_byte);
            } else {
              buf[byte_num] &= ~(1u << bit_in_byte);
            }
          }
          // Second half must be the complement — could validate here, but
          // real hardware noise makes strict checking fragile; skip.
        } else {
          // Stop bit (bit_in_byte == 8): first half must be LOW.
          if (is_first && level != 0) {
            ESP_LOGV(TAG, "Hideki stop bit fail at byte %" PRIu32, byte_num);
            valid = false;
            break;
          }
        }

        // --- Post-byte checks (triggered on the last half of each byte's stop bit) ---
        bool last_half_of_byte = (!is_first) && (bit_in_byte == 8);

        if (last_half_of_byte && byte_num == 0) {
          // Validate header.
          if (buf[0] != 0x75) {
            ESP_LOGV(TAG, "Hideki header fail: %02X", buf[0]);
            valid = false;
            break;
          }
        }

        if (last_half_of_byte && byte_num == 2 && !got_length) {
          // Decode package length from encrypted byte[2].
          // decrypt_byte(b) = b ^ (b << 1); packageLength = (decrypted >> 1) & 0x1f.
          uint8_t decoded_size = (uint8_t) (buf[2] ^ (buf[2] << 1));
          package_length = (decoded_size >> 1) & 0x1f;
          if (package_length < 6 || package_length > 11) {
            ESP_LOGV(TAG, "Hideki package_length out of range: %d", package_length);
            valid = false;
            break;
          }
          // Last expected wf_pos = last half-bit of the cs2 byte's bit.
          // Total bytes = package_length + 3  (header + payload + cs1 + cs2).
          // Each byte = 9 bits * 2 half-bits = 18 half-bits.
          // Plus the 2 half-bits of the start bit that precede the data region.
          // Minus the 2 half-bits of the last byte that doesn't have a stop bit.
          // wf_pos of last half-bit = (package_length + 3) * 18 - 1.
          wf_pos_max = (uint32_t)(package_length + 3u) * 18u - 1u;
          got_length = true;
        }
      }

      if (!valid)
        break;

      // Stop once we have consumed the full expected packet.
      if (got_length && wf_pos > wf_pos_max)
        break;
    }  // inner pulse loop

    if (!valid || !got_length)
      continue;

    // Verify checksums and decrypt.
    if (!decrypt_and_check_(buf, package_length))
      continue;

    // Decode payload.
    WeatherStationData data{};
    if (!decode_thermo_hygro_(buf, data))
      continue;

    ESP_LOGD(TAG, "Hideki id=%d ch=%d t=%.1f h=%d", data.id, data.channel, data.temperature, data.humidity);
    return data;
  }

  return {};
}

// ---------------------------------------------------------------------------
// encode / dump — transmit is not implemented yet.
// ---------------------------------------------------------------------------
void WeatherStationHidekiProtocol::encode(remote_base::RemoteTransmitData *dst, const WeatherStationData &data) {
  ESP_LOGD(TAG, "TODO: encode Hideki");
}

void WeatherStationHidekiProtocol::dump(const WeatherStationData &data) {
  ESP_LOGI(TAG, "Hideki id=%d ch=%d t=%.1f h=%d", data.id, data.channel, data.temperature, data.humidity);
}

}  // namespace esphome::weatherstation
