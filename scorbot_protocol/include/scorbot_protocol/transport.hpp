// Byte transports underneath the framing: a POSIX file descriptor (serial port, pty,
// socket pair) today, UDP later. Non-blocking, no allocation on the data path.

#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>

namespace scorbot_protocol
{

class Transport
{
public:
  virtual ~Transport() = default;
  /// Write up to `n` bytes without blocking. Returns bytes accepted (may be < n), or
  /// npos on a transport error.
  virtual std::size_t write(const uint8_t* data, std::size_t n) = 0;
  /// Read up to `cap` bytes without blocking. Returns bytes read (0 if none), or npos
  /// on a transport error / closed peer.
  virtual std::size_t read(uint8_t* out, std::size_t cap) = 0;
  /// Block until readable or `timeout` elapses. Returns true if readable.
  virtual bool waitReadable(std::chrono::milliseconds timeout) = 0;
  virtual bool isOpen() const = 0;
  virtual std::string describe() const = 0;

  static constexpr std::size_t npos = static_cast<std::size_t>(-1);
};

/// Transport over an already-open non-blocking file descriptor (owned or borrowed).
class FdTransport : public Transport
{
public:
  FdTransport() = default;
  /// `own`: close the fd on destruction.
  explicit FdTransport(int fd, bool own, std::string description = "fd");
  ~FdTransport() override;
  FdTransport(const FdTransport&) = delete;
  FdTransport& operator=(const FdTransport&) = delete;
  FdTransport(FdTransport&& other) noexcept;
  FdTransport& operator=(FdTransport&& other) noexcept;

  std::size_t write(const uint8_t* data, std::size_t n) override;
  std::size_t read(uint8_t* out, std::size_t cap) override;
  bool waitReadable(std::chrono::milliseconds timeout) override;
  bool isOpen() const override { return fd_ >= 0; }
  std::string describe() const override { return description_; }

  int fd() const { return fd_; }
  void close();

protected:
  void adopt(int fd, bool own, std::string description);

private:
  int fd_{-1};
  bool own_{false};
  bool socket_{false};  ///< write with send(MSG_NOSIGNAL): a closed peer is an error, not SIGPIPE
  std::string description_{"closed"};
};

/// Serial port: raw 8N1, no flow control, non-blocking, exclusive.
class SerialTransport : public FdTransport
{
public:
  SerialTransport() = default;
  /// Open `path` at `baud` (e.g. 921600). On failure returns false and fills `error`.
  bool open(const std::string& path, int baud, std::string* error = nullptr);
  /// True if `baud` is a rate this platform's termios supports.
  static bool supportedBaud(int baud);
};

/// Two connected in-process transports (a socketpair), for tests and the simulator.
struct TransportPair
{
  FdTransport a;
  FdTransport b;
  /// Returns false if the socketpair could not be created.
  static bool create(TransportPair& pair);
};

}  // namespace scorbot_protocol
