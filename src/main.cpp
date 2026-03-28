#include <fmt/format.h>
#include "core_pins.h"
#include <Arduino.h>
#include <array>
#include <boost/optional.hpp>
#include <charconv>
#include <chrono>
#include <cstdint>
#include <string>

void delay(std::chrono::milliseconds wait)
{
  delay(wait.count());
}

enum class Pins
{
  Fader = A0,
  Touch = 15,
};

auto toInt(Pins pin)
{
  return static_cast<unsigned int>(pin);
}

auto analogRead(Pins pin)
{
  return analogRead(toInt(pin));
}

auto digitalRead(Pins pin)
{
  return digitalRead(toInt(pin));
}

class Fader
{
  Pins faderPin;
  Pins touchPin;

public:
  Fader(Pins faderPin, Pins touchPin)
      : faderPin(faderPin),
        touchPin(touchPin)
  {
  }

  auto readVolume() const -> std::uint8_t
  {
    auto faderValue = analogRead(faderPin);
    return map(faderValue, 0, 1023, 0, 127);
  }
};

constexpr auto midiChannelNum{1};
class MidiChannel
{
  std::uint8_t CCNum{};
  boost::optional<std::uint8_t> lastValue;

public:
  explicit MidiChannel(std::uint8_t CCNum)
      : CCNum(CCNum)
  {
  }

  void setVolume(std::uint8_t value)
  {
    auto checkShouldSkip = [value](std::uint8_t lastValue) -> bool
    {
      if (lastValue == value)
        return true;
      const auto diff = static_cast<int>(lastValue) - static_cast<int>(value);
      return std::abs(diff) <= 1;
    };

    auto shouldSkip = lastValue.map(checkShouldSkip).value_or(false);
    if (shouldSkip)
      return;

    Serial.println(fmt::format("Fader: {}", value).c_str());
    usbMIDI.sendControlChange(CCNum, value, midiChannelNum);
    lastValue = value;
  }
};

constexpr int maxChannels = 1;
class Application
{
  std::array<Fader, maxChannels> faders = {Fader{Pins::Fader, Pins::Touch}};
  std::array<boost::optional<MidiChannel>, maxChannels> mappedMidi{};

public:
  void setup()
  {
    pinMode(toInt(Pins::Touch), INPUT_PULLUP);
    mappedMidi[0] = MidiChannel(7);
  }

  void loop()
  {
    for (int i = 0; i < maxChannels; i++)
    {
      if (!mappedMidi[i])
        continue;
      auto &midiChannel = mappedMidi[i].value();
      midiChannel.setVolume(faders[i].readVolume());
    }

    const auto touch = digitalRead(Pins::Touch);
    const bool touched = touch == LOW;
    const auto message = fmt::format("touched: {}, {}", touched, touch);
    Serial.println(message.c_str());
    usbMIDI.send_now();
    auto sleep = std::chrono::milliseconds(500);
    delay(sleep);
  }
};

Application app;

void setup()
{
  Serial.begin(115200);
  app.setup();
}

void loop()
{
  app.loop();
}