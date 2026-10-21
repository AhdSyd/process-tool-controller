#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>

#include "telemetry/logger.hpp"

using namespace ptc;
using namespace ptc::telemetry;

TEST(Logger, FormatsStateChangeWithReason) {
  LogRecord r;
  r.t_ns = 1'500'000'000;
  r.event = LogEvent::StateChange;
  r.a = static_cast<std::uint8_t>(State::Process);
  r.b = static_cast<std::uint8_t>(State::Aborting);
  r.c = static_cast<std::uint8_t>(Reason::OverTemperature);
  r.x = 451.25;
  char buf[256];
  const std::size_t n = format_record(r, 500'000'000, "ctl", buf, sizeof(buf));
  const std::string s(buf, n);
  EXPECT_NE(s.find("[   1.000000]"), std::string::npos) << s;
  EXPECT_NE(s.find("Process  -> Aborting"), std::string::npos) << s;
  EXPECT_NE(s.find("(OverTemperature)"), std::string::npos) << s;
  EXPECT_NE(s.find("T=451.25C"), std::string::npos) << s;
}

TEST(Logger, FormatNeverOverflowsTheBuffer) {
  LogRecord r;
  r.event = LogEvent::StateChange;
  char buf[16];
  const std::size_t n = format_record(r, 0, "ctl", buf, sizeof(buf));
  EXPECT_LT(n, sizeof(buf));
}

TEST(Logger, WritesEverythingEnqueuedBeforeStop) {
  const std::string path = ::testing::TempDir() + "ptc_logger_test.log";
  {
    auto log = std::make_unique<Logger>(path, 0, false);
    log->start();
    for (std::uint32_t i = 0; i < 1000; ++i) {
      LogRecord r;
      r.t_ns = i;
      r.event = LogEvent::WaferComplete;
      r.u = i;
      ASSERT_TRUE(log->control_queue().try_push(r));
    }
    LogRecord u;
    u.event = LogEvent::Command;
    log->ui_queue().try_push(u);
    log->stop();  // final drain must flush everything
    EXPECT_EQ(log->lines_written(), 1001u);
    EXPECT_EQ(log->dropped(), 0u);
  }
  std::ifstream in(path);
  std::stringstream ss;
  ss << in.rdbuf();
  const std::string all = ss.str();
  EXPECT_NE(all.find("WAFER      #999"), std::string::npos);
  EXPECT_NE(all.find("COMMAND"), std::string::npos);
  std::remove(path.c_str());
}

TEST(Logger, ThrowsIfFileCannotBeOpened) {
  EXPECT_THROW(Logger("/nonexistent-dir/x.log", 0, false), std::runtime_error);
}
