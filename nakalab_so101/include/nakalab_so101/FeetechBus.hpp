#ifndef NAKALAB_SO101__FEETECH_BUS_HPP_
#define NAKALAB_SO101__FEETECH_BUS_HPP_

#include <libserial/SerialPort.h>
#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <optional>

namespace nakalab_so101
{

struct ServoState
{
  int position = 0;
  int speed = 0;
  int load = 0;
};

class FeetechBus
{
public:
  explicit FeetechBus(const std::string & port, int baudrate = 1000000);
  ~FeetechBus();

  bool connect();
  void disconnect();
  bool is_connected() const;

  // Low-level protocol
  bool write_packet(uint8_t motor_id, uint8_t instruction, const std::vector<uint8_t> & parameters);
  std::optional<std::vector<uint8_t>> read_packet();

  // High-level commands
  bool write_register(uint8_t motor_id, uint8_t address, const std::vector<uint8_t> & data);
  std::optional<std::vector<uint8_t>> read_register(uint8_t motor_id, uint8_t address, uint8_t length);

  bool sync_write_goal_positions(const std::map<uint8_t, int16_t> & motor_goal_map);
  bool sync_write_goal_states(const std::map<uint8_t, ServoState> & motor_states);

  std::map<uint8_t, int16_t> sync_read_present_positions(const std::vector<uint8_t> & motor_ids);
  std::map<uint8_t, ServoState> sync_read_present_states(const std::vector<uint8_t> & motor_ids);

  void enable_torque(const std::vector<uint8_t> & motor_ids, bool enable);

private:
  // Instruction definitions
  static constexpr uint8_t INST_PING = 0x01;
  static constexpr uint8_t INST_READ = 0x02;
  static constexpr uint8_t INST_WRITE = 0x03;
  static constexpr uint8_t INST_REG_WRITE = 0x04;
  static constexpr uint8_t INST_ACTION = 0x05;
  static constexpr uint8_t INST_SYNC_READ = 0x82;
  static constexpr uint8_t INST_SYNC_WRITE = 0x83;

  // Register definitions
  static constexpr uint8_t REG_TORQUE_ENABLE = 40;
  static constexpr uint8_t REG_GOAL_POSITION = 42;
  static constexpr uint8_t REG_GOAL_TIME = 44;
  static constexpr uint8_t REG_GOAL_SPEED = 46;
  static constexpr uint8_t REG_PRESENT_POSITION = 56;
  static constexpr uint8_t REG_PRESENT_SPEED = 58;
  static constexpr uint8_t REG_PRESENT_LOAD = 60;

  // Broadcast ID
  static constexpr uint8_t BROADCAST_ID = 0xFE;

  static LibSerial::BaudRate get_libserial_baud_rate(int baudrate);

  std::string port_;
  int baudrate_;
  LibSerial::SerialPort serial_;
  std::mutex bus_mutex_;
};

}  // namespace NAKALAB_SO101

#endif  // NAKALAB_SO101__FEETECH_BUS_HPP_
