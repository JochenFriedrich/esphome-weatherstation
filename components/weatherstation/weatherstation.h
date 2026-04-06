#pragma once

#include "esphome/components/remote_base/remote_base.h"

namespace esphome::weatherstation {

struct WeatherStationData {
  uint16_t id{};
  float battery_level{};
  uint8_t channel{};
  float temperature{};
  uint8_t humidity{};
  float rain{};
  float wind_direction_degrees{};
  float wind_speed{};
  float wind_gust{};
  // TODO: pressure
};

class WeatherStationComponent : public Component {
};

class WeatherStationProtocol : public remote_base::RemoteProtocol<WeatherStationData> {
 public:
  void encode(remote_base::RemoteTransmitData *dst, const WeatherStationData &data) override;
  optional<WeatherStationData> decode(remote_base::RemoteReceiveData src) override;
  void dump(const WeatherStationData &data) override;

 protected:
  uint16_t sync_high_;
  uint16_t sync_low_;
  uint16_t zero_high_;
  uint16_t zero_low_;
  uint16_t one_high_;
  uint16_t one_low_;
  uint8_t nbits_;
  uint8_t nbits_min_{};
  uint8_t repeat_;
  uint8_t flags_;

  enum WeatherStationFlag : uint8_t {
    TYPE_PWM = 0b00000000,
    TYPE_PPM = 0b00000001,
    TYPE_MANCHESTER = 0b00000010,
    TYPE_DMANCHESTER = 0b00000011,
    TYPE_MASK = 0b00000011,
    INVERTED = 0b00000100,
    REVERSED = 0b00001000,
  };

  std::vector<uint8_t> code_;

  uint32_t get_bits_(uint8_t pos, uint8_t nbits) const;
  void set_bits_(uint8_t pos, uint8_t nbits, uint32_t c);

  bool is_pwm_() const { return (this->flags_ & TYPE_MASK) == TYPE_PWM; }
  bool is_ppm_() const { return (this->flags_ & TYPE_MASK) == TYPE_PPM; }
  bool is_manchester_() const { return (this->flags_ & TYPE_MASK) == TYPE_MANCHESTER; }
  bool is_dmanchester_() const { return (this->flags_ & TYPE_MASK) == TYPE_DMANCHESTER; }
  bool is_inverted_() const { return (this->flags_ & INVERTED) != 0; }
  bool is_reversed_() const { return (this->flags_ & REVERSED) != 0; }

  bool receive_item_(remote_base::RemoteReceiveData &src, uint32_t high, uint32_t low) const;
  bool receive_code_(remote_base::RemoteReceiveData &src);
  void transmit_item_(remote_base::RemoteTransmitData *dst, uint32_t high, uint32_t low) const;
  void transmit_code_(remote_base::RemoteTransmitData *dst) const;

  virtual void setup() = 0;
  virtual bool to_data(WeatherStationData &data) const = 0;
  virtual bool to_code(const WeatherStationData &data) = 0;
};

template<typename P, typename... Ts> class WeatherStationAction : public remote_base::RemoteTransmitterActionBase<Ts...> {
 public:
  TEMPLATABLE_VALUE(uint16_t, id)
  TEMPLATABLE_VALUE(float, battery_level)
  TEMPLATABLE_VALUE(uint8_t, channel)
  TEMPLATABLE_VALUE(float, temperature)
  TEMPLATABLE_VALUE(uint8_t, humidity)
  TEMPLATABLE_VALUE(float, rain)
  TEMPLATABLE_VALUE(float, wind_direction_degrees)
  TEMPLATABLE_VALUE(float, wind_speed)
  TEMPLATABLE_VALUE(float, wind_gust)

  void encode(remote_base::RemoteTransmitData *dst, Ts... x) override {
    WeatherStationData data{};
    data.id = this->id_.value(x...);
    data.battery_level = this->battery_level_.value(x...);
    data.channel = this->channel_.value(x...);
    data.temperature = this->temperature_.value(x...);
    data.humidity = this->humidity_.value(x...);
    data.rain = this->rain_.value(x...);
    data.wind_direction_degrees = this->wind_direction_degrees_.value(x...);
    data.wind_speed = this->wind_speed_.value(x...);
    data.wind_gust = this->wind_gust_.value(x...);
    P().encode(dst, data);
  }
};

template<typename T> class WeatherStationBinarySensor : public remote_base::RemoteReceiverBinarySensorBase {
 protected:
  bool matches(remote_base::RemoteReceiveData src) override {
    auto proto = T();
    auto res = proto.decode(src);
    return res.has_value();
  }
};

#define DECLARE_WEATHER_STATION_PROTOCOL_(prefix) \
  class prefix##Protocol : public WeatherStationProtocol { \
   protected: \
    void setup() override; \
    bool to_data(WeatherStationData &data) const override; \
    bool to_code(const WeatherStationData &data) override; \
  }; \
  using prefix##BinarySensor = WeatherStationBinarySensor<prefix##Protocol>; \
  using prefix##Trigger = remote_base::RemoteReceiverTrigger<prefix##Protocol>; \
  using prefix##Dumper = remote_base::RemoteReceiverDumper<prefix##Protocol>; \
  template<typename... Ts> class prefix##Action : public WeatherStationAction<prefix##Protocol, Ts...> {};

#define DECLARE_WEATHER_STATION_PROTOCOL(prefix) DECLARE_WEATHER_STATION_PROTOCOL_(prefix)

DECLARE_WEATHER_STATION_PROTOCOL(WeatherStation2032)
DECLARE_WEATHER_STATION_PROTOCOL(WeatherStation4LD)
DECLARE_WEATHER_STATION_PROTOCOL(WeatherStationAHFL)
DECLARE_WEATHER_STATION_PROTOCOL(WeatherStationBresser3CH)
DECLARE_WEATHER_STATION_PROTOCOL(WeatherStationEurochron)
DECLARE_WEATHER_STATION_PROTOCOL(WeatherStationH10515)
DECLARE_WEATHER_STATION_PROTOCOL(WeatherStationH13726)
DECLARE_WEATHER_STATION_PROTOCOL(WeatherStationL08037A)
DECLARE_WEATHER_STATION_PROTOCOL(WeatherStationNexus)
DECLARE_WEATHER_STATION_PROTOCOL(WeatherStationZ31743)
DECLARE_WEATHER_STATION_PROTOCOL(WeatherStationZ32171)

}  // namespace esphome::weatherstation
