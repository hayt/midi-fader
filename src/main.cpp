#include <algorithm>
#include <cstdlib>
#include <fmt/format.h>
#include "core_pins.h"
#include "fmt/core.h"
#include "usb_serial.h"
#include <Arduino.h>
#include <array>
#include <boost/optional.hpp>
#include <charconv>
#include <chrono>
#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include "TouchyTouch.h"

void delay(std::chrono::milliseconds wait)
{
  delay(wait.count());
}

enum class Pins
{
  Fader = A0,
  Touch = 15,
  STBY = 12,
  IN1 = 11,
  IN2 = 10,
  PWM = 9,

};

auto toInt(Pins pin)
{
  return static_cast<unsigned int>(pin);
}

auto analogRead(Pins pin)
{
  return analogRead(toInt(pin));
}

auto analogWrite(Pins pin, std::uint8_t value)
{
  return analogWrite(toInt(pin), value);
}

auto digitalRead(Pins pin)
{
  return digitalRead(toInt(pin));
}

auto digitalWrite(Pins pin, std::uint8_t value)
{
  digitalWrite(toInt(pin), value);
}

auto boolToHighLow(bool value) -> std::uint8_t
{
  if (value)
    return HIGH;
  return LOW;
}

template <typename... Args>
void printLn(fmt::format_string<Args...> fmtString, Args &&...args)
{
  const auto message = fmt::format(fmtString, args...);
  Serial.println(message.c_str());
}

class Fader
{
  Pins faderPin;
  Pins touchPin;
  mutable TouchyTouch touchio;

public:
  Fader(Pins faderPin, Pins touchPin)
      : faderPin(faderPin),
        touchPin(touchPin)
  {
    touchio.begin(toInt(touchPin));
  }

  auto readVolume() const -> std::uint8_t
  {
    auto faderValue = analogRead(faderPin);
    return map(faderValue, 0, 1023, 0, 127);
  }

  auto update()
  {
    touchio.update();
  }

  auto readTouch() const -> bool
  {
    return touchio.touched();
    constexpr static int touchThresold = 15;
    const auto touch = analogRead(touchPin);
    printLn("touch value: {}", touch);
    return touch < touchThresold;
  }
};

class FaderPID
{
  static constexpr float kp = 1.0f;
  static constexpr float ki = 0.0f;
  static constexpr float kd = 0.0f;
  float integral = 0;
  float lastError = 0;
  float outputMin, outputMax;

public:
  FaderPID(float outputMin = -255, float outputMax = 255)
      : outputMin(outputMin),
        outputMax(outputMax)
  {
  }

  float compute(float setpoint, float input)
  {
    float error = setpoint - input;
    integral += error;
    float derivative = error - lastError;
    lastError = error;
    float output = kp * error + ki * integral + kd * derivative;
    return std::clamp(output, outputMin, outputMax);
  }

  void reset()
  {
    integral = 0;
    lastError = 0;
  }
};

class Motor
{
  std::pair<Pins, Pins> direction;
  Pins pwm;
  Pins stby;

public:
  Motor(Pins pin1, Pins pin2, Pins pwm, Pins stby)
      : direction(std::make_pair(pin1, pin2)),
        pwm(pwm),
        stby(stby)
  {
    pinMode(toInt(direction.first), OUTPUT);
    pinMode(toInt(direction.second), OUTPUT);
    pinMode(toInt(stby), OUTPUT);
    pinMode(toInt(pwm), OUTPUT);
  }

  void ready(bool on = true) const
  {
    const auto value = boolToHighLow(on);
    printLn("set stby to {}", value);
    digitalWrite(stby, value);
  }

  void setDirection(bool up) const
  {
    auto value1 = boolToHighLow(up);
    auto value2 = boolToHighLow(!up);
    digitalWrite(direction.first, value1);
    digitalWrite(direction.second, value2);
  }

  void setSpeed(std::uint8_t speed) const
  {
    analogWrite(pwm, speed);
  }

  void forward(uint8_t speed) const
  {
    printLn("Forward");
    setDirection(true);
    setSpeed(speed);
  }

  void backward(uint8_t speed) const
  {
    printLn("Backwards {}", speed);
    setDirection(false);
    setSpeed(speed);
  }

  void stop()
  {
    setSpeed(0);
    digitalWrite(direction.first, LOW);
    digitalWrite(direction.second, LOW);
  }
};

constexpr auto midiChannelNum{1};
class MidiChannel
{
  std::uint8_t CCNum{};
  boost::optional<std::uint8_t> lastValue;
  bool touching{false};

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

    printLn("Fader: {}", value);
    usbMIDI.sendControlChange(CCNum, value, midiChannelNum);
    lastValue = value;
  }

  void isTouching(bool touching)
  {
    if (touching == this->touching)
      return;
    this->touching = touching;
    printLn("touching {}", this->touching);
  }
};

auto readInput() -> boost::optional<std::string>
{
  if (Serial.available() > 0)
  {
    auto res = Serial.readStringUntil('\n');
    return std::string(res.c_str());
  }
  return boost::none;
}

auto stringToInt(std::string s) -> boost::optional<int>
{
  int value{};
  auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), value);
  if (ec == std::errc())
  {
    return boost::none;
  }
  return value;
}

auto adjustPWM(float in)
{
  constexpr auto minumumOut = 100.0f;
  return std::max(minumumOut, in);
}

class MotorizedFader
{
  std::reference_wrapper<Fader> fader;
  std::reference_wrapper<Motor> motor;
  FaderPID faderPid;
  std::uint8_t setPoint{0};

public:
  MotorizedFader(Fader &fader, Motor &motor)
      : fader(fader),
        motor(motor)
  {
  }

  void setTarget(std::uint8_t i)
  {
    setPoint = i;
    faderPid.reset();
  }

  void update()
  {
    if (fader.get().readTouch())
    {
      motor.get().stop();
      return;
    }
    auto currentValue = fader.get().readVolume();
    auto output = faderPid.compute(setPoint, currentValue);
    constexpr auto threshold = 5;
    if (std::abs(output) < threshold)
    {
      motor.get().stop();
      return;
    }
    printLn(
        "calculated output: {} setpoint: {}, currentValue: {}",
        output,
        setPoint,
        currentValue);
    if (output > 0)
    {
      motor.get().forward(adjustPWM(output));
      return;
    }
    motor.get().backward(adjustPWM(-1 * output));
  }
};

constexpr int maxChannels = 1;
class Application
{
  std::array<Fader, maxChannels> faders = {Fader{Pins::Fader, Pins::Touch}};
  std::array<boost::optional<MidiChannel>, maxChannels> mappedMidi{};
  Motor m{Pins::IN1, Pins::IN2, Pins::PWM, Pins::STBY};
  MotorizedFader mf{faders[0], m};

public:
  void setup()
  {
    mappedMidi[0] = MidiChannel(7);
    m.ready();
  }

  void loop()
  {
    using namespace std::literals;
    for (int i = 0; i < maxChannels; i++)
    {
      if (!mappedMidi[i])
        continue;
      auto &midiChannel = mappedMidi[i].value();
      faders[i].update();
      midiChannel.setVolume(faders[i].readVolume());
      midiChannel.isTouching(faders[i].readTouch());
    }
    usbMIDI.send_now();

    // auto input = readInput().flat_map(&stringToInt);
    if (Serial.available() > 0)
    {
      char c = Serial.peek();
      if (c == '>')
      {
        Serial.read(); // consume the '>'
        Serial.println("Wait for input");
        Serial.setTimeout(10000);
        auto input = Serial.parseInt();
        printLn("{}:", input);
        mf.setTarget(std::clamp(input, 0l, 128l));
      }
      else
      {
        Serial.read(); // discard debug echo
      }
    }
    mf.update();

    auto sleep = std::chrono::milliseconds(10);
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