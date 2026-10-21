#include "app/console.hpp"

#include <sys/select.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>

namespace ptc::app {

bool Console::take_line(std::string& out) {
  const auto nl = pending_.find('\n');
  if (nl == std::string::npos) return false;
  out.assign(pending_, 0, nl);
  pending_.erase(0, nl + 1);
  if (!out.empty() && out.back() == '\r') out.pop_back();
  return true;
}

Console::Event Console::wait(std::string& out, std::chrono::milliseconds timeout) {
  if (take_line(out)) return Event::Line;

  fd_set rfds;
  FD_ZERO(&rfds);
  FD_SET(wake_fd_, &rfds);
  if (stdin_open_) FD_SET(STDIN_FILENO, &rfds);
  const int maxfd = std::max(wake_fd_, STDIN_FILENO);

  timeval tv{};
  tv.tv_sec = static_cast<decltype(tv.tv_sec)>(timeout.count() / 1000);
  tv.tv_usec = static_cast<decltype(tv.tv_usec)>((timeout.count() % 1000) * 1000);

  const int rc = select(maxfd + 1, &rfds, nullptr, nullptr, &tv);
  if (rc < 0) return errno == EINTR ? Event::Signal : Event::Timeout;
  if (rc == 0) return Event::Timeout;

  if (FD_ISSET(wake_fd_, &rfds)) {
    char drain[64];
    while (read(wake_fd_, drain, sizeof(drain)) > 0) {
    }  // non-blocking pipe: empty it
    return Event::Signal;
  }

  if (stdin_open_ && FD_ISSET(STDIN_FILENO, &rfds)) {
    char buf[256];
    const ssize_t n = read(STDIN_FILENO, buf, sizeof(buf));
    if (n > 0) {
      pending_.append(buf, static_cast<std::size_t>(n));
    } else if (n == 0 || (errno != EINTR && errno != EAGAIN)) {
      stdin_open_ = false;  // EOF: treat unterminated text as a final line
      if (!pending_.empty()) pending_.push_back('\n');
    }
  }
  return take_line(out) ? Event::Line : Event::Timeout;
}

}  // namespace ptc::app
