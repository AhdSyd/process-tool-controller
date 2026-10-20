#include "app/console.hpp"

#include <poll.h>
#include <unistd.h>

#include <cerrno>
#include <ctime>

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

  const auto secs = std::chrono::duration_cast<std::chrono::seconds>(timeout);
  timespec ts{};
  ts.tv_sec = static_cast<std::time_t>(secs.count());
  ts.tv_nsec = static_cast<long>(std::chrono::nanoseconds(timeout - secs).count());

  pollfd pfd{STDIN_FILENO, POLLIN, 0};
  const nfds_t nfds = stdin_open_ ? 1 : 0;
  const int rc = ppoll(&pfd, nfds, &ts, &wait_mask_);

  if (rc < 0) return errno == EINTR ? Event::Interrupted : Event::Timeout;
  if (rc == 0) return Event::Timeout;

  if (pfd.revents & (POLLIN | POLLHUP | POLLERR)) {
    char buf[256];
    const ssize_t n = read(STDIN_FILENO, buf, sizeof(buf));
    if (n > 0) {
      pending_.append(buf, static_cast<std::size_t>(n));
    } else if (n == 0 || (errno != EINTR && errno != EAGAIN)) {
      stdin_open_ = false;  // EOF: treat any unterminated text as a final line
      if (!pending_.empty()) pending_.push_back('\n');
    }
  }
  return take_line(out) ? Event::Line : Event::Timeout;
}

}  // namespace ptc::app
