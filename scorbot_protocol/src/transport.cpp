#include "scorbot_protocol/transport.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

namespace scorbot_protocol
{

// ---- FdTransport -----------------------------------------------------------------------

FdTransport::FdTransport(int fd, bool own, std::string description) { adopt(fd, own, std::move(description)); }

FdTransport::~FdTransport() { close(); }

FdTransport::FdTransport(FdTransport&& other) noexcept
  : fd_(other.fd_), own_(other.own_), socket_(other.socket_), description_(std::move(other.description_))
{
  other.fd_ = -1;
  other.own_ = false;
  other.socket_ = false;
  other.description_ = "closed";
}

FdTransport& FdTransport::operator=(FdTransport&& other) noexcept
{
  if (this != &other)
  {
    close();
    fd_ = other.fd_;
    own_ = other.own_;
    socket_ = other.socket_;
    description_ = std::move(other.description_);
    other.fd_ = -1;
    other.own_ = false;
    other.socket_ = false;
    other.description_ = "closed";
  }
  return *this;
}

void FdTransport::adopt(int fd, bool own, std::string description)
{
  close();
  fd_ = fd;
  own_ = own;
  description_ = std::move(description);
  socket_ = false;
  if (fd_ >= 0)
  {
    const int flags = ::fcntl(fd_, F_GETFL, 0);
    if (flags >= 0)
      ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
    int type = 0;
    socklen_t len = sizeof(type);
    socket_ = ::getsockopt(fd_, SOL_SOCKET, SO_TYPE, &type, &len) == 0;
  }
}

void FdTransport::close()
{
  if (fd_ >= 0 && own_)
    ::close(fd_);
  fd_ = -1;
  own_ = false;
  description_ = "closed";
}

std::size_t FdTransport::write(const uint8_t* data, std::size_t n)
{
  if (fd_ < 0)
    return npos;
  std::size_t done = 0;
  while (done < n)
  {
    // A socket whose peer went away must report an error, not kill the process.
    const ssize_t r = socket_ ? ::send(fd_, data + done, n - done, MSG_NOSIGNAL) : ::write(fd_, data + done, n - done);
    if (r > 0)
    {
      done += static_cast<std::size_t>(r);
      continue;
    }
    if (r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
      break;  // kernel buffer full: report the partial write
    if (r < 0 && errno == EINTR)
      continue;
    return npos;
  }
  return done;
}

std::size_t FdTransport::read(uint8_t* out, std::size_t cap)
{
  if (fd_ < 0)
    return npos;
  for (;;)
  {
    const ssize_t r = ::read(fd_, out, cap);
    if (r > 0)
      return static_cast<std::size_t>(r);
    if (r == 0)
      return npos;  // peer closed (sockets/pty); serial ports never return 0 in non-blocking mode
    if (errno == EAGAIN || errno == EWOULDBLOCK)
      return 0;
    if (errno == EINTR)
      continue;
    return npos;
  }
}

bool FdTransport::waitReadable(std::chrono::milliseconds timeout)
{
  if (fd_ < 0)
    return false;
  pollfd p{};
  p.fd = fd_;
  p.events = POLLIN;
  const int r = ::poll(&p, 1, static_cast<int>(timeout.count()));
  return r > 0 && (p.revents & (POLLIN | POLLHUP | POLLERR));
}

// ---- SerialTransport -------------------------------------------------------------------

namespace
{
speed_t baudConstant(int baud)
{
  switch (baud)
  {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
#ifdef B460800
    case 460800: return B460800;
#endif
#ifdef B921600
    case 921600: return B921600;
#endif
#ifdef B1000000
    case 1000000: return B1000000;
#endif
#ifdef B2000000
    case 2000000: return B2000000;
#endif
    default: return 0;
  }
}
}  // namespace

bool SerialTransport::supportedBaud(int baud) { return baudConstant(baud) != 0; }

bool SerialTransport::open(const std::string& path, int baud, std::string* error)
{
  auto fail = [&](const std::string& what) {
    if (error)
      *error = what + ": " + std::strerror(errno);
    return false;
  };

  const speed_t speed = baudConstant(baud);
  if (speed == 0)
  {
    if (error)
      *error = "unsupported baud rate " + std::to_string(baud);
    return false;
  }

  const int fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0)
    return fail("open " + path);

  // Exclusive access: a second process opening the port would corrupt frames.
  ::ioctl(fd, TIOCEXCL);

  termios tio{};
  if (::tcgetattr(fd, &tio) != 0)
  {
    ::close(fd);
    return fail("tcgetattr");
  }
  ::cfmakeraw(&tio);
  tio.c_cflag |= CLOCAL | CREAD;
  tio.c_cflag &= ~static_cast<tcflag_t>(CRTSCTS);  // no hardware flow control
  tio.c_cflag &= ~static_cast<tcflag_t>(CSTOPB);   // 1 stop bit
  tio.c_cflag &= ~static_cast<tcflag_t>(PARENB);   // no parity
  tio.c_cflag = (tio.c_cflag & ~static_cast<tcflag_t>(CSIZE)) | CS8;
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  ::cfsetispeed(&tio, speed);
  ::cfsetospeed(&tio, speed);
  if (::tcsetattr(fd, TCSANOW, &tio) != 0)
  {
    ::close(fd);
    return fail("tcsetattr");
  }
  ::tcflush(fd, TCIOFLUSH);

  adopt(fd, true, path + "@" + std::to_string(baud));
  return true;
}

// ---- TransportPair ---------------------------------------------------------------------

bool TransportPair::create(TransportPair& pair)
{
  int fds[2];
  if (::socketpair(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0, fds) != 0)
    return false;
  pair.a = FdTransport(fds[0], true, "pair.a");
  pair.b = FdTransport(fds[1], true, "pair.b");
  return true;
}

}  // namespace scorbot_protocol
