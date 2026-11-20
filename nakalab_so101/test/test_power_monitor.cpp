#include "nakalab_so101/PowerMonitor.hpp"

#include <gtest/gtest.h>

TEST(PowerMonitorTest, StaysNormalWithHealthyVoltages)
{
  nakalab_so101::PowerMonitor monitor({6, 5.5, 2});
  const auto update = monitor.update_from_voltages({12.0, 11.9, 12.1, 12.0, 11.8, 12.0});

  EXPECT_FALSE(update.estop_active);
  EXPECT_FALSE(update.entered_estop);
  EXPECT_DOUBLE_EQ(update.min_voltage, 11.8);
}

TEST(PowerMonitorTest, TriggersImmediatelyOnLowVoltage)
{
  nakalab_so101::PowerMonitor monitor({6, 5.5, 2});
  const auto update = monitor.update_from_voltages({12.0, 11.8, 4.9, 12.0, 12.0, 12.0});

  EXPECT_TRUE(update.estop_active);
  EXPECT_TRUE(update.entered_estop);
  EXPECT_EQ(update.reason, "low_voltage");
  EXPECT_TRUE(monitor.is_estop_active());
}

TEST(PowerMonitorTest, TriggersAfterConsecutiveCommunicationMisses)
{
  nakalab_so101::PowerMonitor monitor({6, 5.5, 2});

  const auto first = monitor.update_from_voltages({12.0, 12.0});
  EXPECT_FALSE(first.estop_active);
  EXPECT_FALSE(first.entered_estop);
  EXPECT_EQ(monitor.consecutive_misses(), 1);

  const auto second = monitor.update_from_voltages({});
  EXPECT_TRUE(second.estop_active);
  EXPECT_TRUE(second.entered_estop);
  EXPECT_EQ(second.reason, "communication_lost");
}

TEST(PowerMonitorTest, MarksReadyForRecoveryWhenHealthyVoltageRestored)
{
  nakalab_so101::PowerMonitor monitor({6, 5.5, 1});
  monitor.update_from_voltages({});
  EXPECT_TRUE(monitor.is_estop_active());

  const auto still_low = monitor.update_from_probe(4.2);
  EXPECT_FALSE(still_low.ready_for_recovery);

  const auto restored = monitor.update_from_probe(12.1);
  EXPECT_TRUE(restored.estop_active);
  EXPECT_TRUE(restored.ready_for_recovery);

  monitor.complete_recovery();
  EXPECT_FALSE(monitor.is_estop_active());
}
