#pragma once
// Host-only UART/time stubs. Never included in an Arduino build.
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <vector>
using byte = uint8_t;
inline uint64_t hostMicros = 0;
inline uint32_t millis() { return hostMicros / 1000; }
inline void delayMicroseconds(unsigned value) { hostMicros += value; }
inline void delay(unsigned value) { hostMicros += value*1000; }
constexpr int HIGH=1, LOW=0, OUTPUT=1, SERIAL_8N1=0, SERIAL_8E1=1, HEX=16;
inline void pinMode(int,int) {}
inline void digitalWrite(int,int) {}

class Stream {
 public:
  virtual int available() = 0;
  virtual int read() = 0;
  virtual size_t write(const uint8_t *, size_t) = 0;
  virtual size_t write(uint8_t b) { return write(&b,1); }
  virtual void flush() {}
  template<class T> void print(T) {}
  template<class T> void print(T,int) {}
  template<class T> void println(T) {}
  void println() {}
};

class HardwareSerial: public Stream {
 public:
  std::deque<byte> input;
  std::vector<std::vector<byte>> writes;
  std::function<void(const std::vector<byte>&)> onWrite;
  uint64_t readyAt = 0, byteSpacing = 0;
  explicit HardwareSerial(int=0) {}
  void begin(uint32_t,int=0,int=0,int=0) {}
  int available() override { return hostMicros >= readyAt ? input.size() : 0; }
  int read() override {
    if (!available()) return -1;
    byte b = input.front(); input.pop_front(); readyAt = hostMicros+byteSpacing; return b;
  }
  size_t write(const uint8_t* p,size_t n) override {
    std::vector<byte> packet(p,p+n); writes.push_back(packet);
    if (onWrite) onWrite(packet);
    return n;
  }
  using Stream::write;
};
inline HardwareSerial Serial;
