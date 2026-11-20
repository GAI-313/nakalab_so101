#ifndef NAKALAB_SO101__FEETECH_BUS_HPP_
#define NAKALAB_SO101__FEETECH_BUS_HPP_

#include <libserial/SerialPort.h>
#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace nakalab_so101
{

struct ServoState
{
  int position = 0;
  int speed = 0;
  int load = 0;
  double voltage = 0.0;
};

struct FeetechStatusPacket
{
  uint8_t motor_id = 0;
  uint8_t error = 0;
  std::vector<uint8_t> parameters;
};

class FeetechBus
{
public:
  static constexpr uint8_t REG_RETURN_DELAY_TIME = 7;
  static constexpr uint8_t REG_MIN_POSITION_LIMIT = 9;
  static constexpr uint8_t REG_MAX_POSITION_LIMIT = 11;
  static constexpr uint8_t REG_PHASE = 18;
  static constexpr uint8_t REG_HOMING_OFFSET = 31;
  static constexpr uint8_t REG_OPERATING_MODE = 33;
  static constexpr uint8_t REG_TORQUE_ENABLE = 40;
  static constexpr uint8_t REG_GOAL_POSITION = 42;
  static constexpr uint8_t REG_GOAL_TIME = 44;
  static constexpr uint8_t REG_GOAL_SPEED = 46;
  static constexpr uint8_t REG_LOCK = 55;
  static constexpr uint8_t REG_PRESENT_POSITION = 56;
  static constexpr uint8_t REG_PRESENT_SPEED = 58;
  static constexpr uint8_t REG_PRESENT_LOAD = 60;
  static constexpr uint8_t REG_PRESENT_VOLTAGE = 62;

  static constexpr uint8_t HOMING_OFFSET_SIGN_BIT = 11;
  static constexpr uint8_t POSITION_SIGN_BIT = 15;
  static constexpr int STS3215_RESOLUTION = 4096;
  static constexpr int kDefaultNumRetry = 3;
  static constexpr int kDefaultHeaderTimeoutMs = 50;

  explicit FeetechBus(const std::string & port, int baudrate = 1000000);
  ~FeetechBus();

  bool connect();
  void disconnect();
  bool is_connected() const;

  static int decode_sign_magnitude(uint16_t raw_value, uint8_t sign_bit_index);
  static uint16_t encode_sign_magnitude(int value, uint8_t sign_bit_index);

  // Low-level protocol
  void clear_input_buffer();
  bool write_packet(uint8_t motor_id, uint8_t instruction, const std::vector<uint8_t> & parameters);
  std::optional<std::vector<uint8_t>> read_packet();
  std::optional<FeetechStatusPacket> read_status_packet(
    std::optional<uint8_t> expected_motor_id = std::nullopt,
    int header_timeout_ms = kDefaultHeaderTimeoutMs);

  // High-level commands
  bool ping(
    uint8_t motor_id,
    int num_retry = kDefaultNumRetry,
    int header_timeout_ms = kDefaultHeaderTimeoutMs);
  bool write_register(
    uint8_t motor_id,
    uint8_t address,
    const std::vector<uint8_t> & data,
    int num_retry = kDefaultNumRetry);
  std::optional<std::vector<uint8_t>> read_register(
    uint8_t motor_id,
    uint8_t address,
    uint8_t length,
    int num_retry = kDefaultNumRetry,
    int header_timeout_ms = kDefaultHeaderTimeoutMs);
  std::optional<uint8_t> read_u8(uint8_t motor_id, uint8_t address);
  std::optional<uint16_t> read_u16(uint8_t motor_id, uint8_t address);
  bool write_u8(uint8_t motor_id, uint8_t address, uint8_t value);
  bool write_u16(uint8_t motor_id, uint8_t address, uint16_t value);
  std::optional<int> read_homing_offset(uint8_t motor_id);
  bool write_homing_offset(uint8_t motor_id, int value);
  std::optional<double> read_voltage(
    uint8_t motor_id,
    int num_retry = 0,
    int header_timeout_ms = kDefaultHeaderTimeoutMs);

  bool sync_write_goal_positions(const std::map<uint8_t, int16_t> & motor_goal_map);
  bool sync_write_goal_states(const std::map<uint8_t, ServoState> & motor_states);

  std::map<uint8_t, int16_t> sync_read_present_positions(
    const std::vector<uint8_t> & motor_ids,
    int num_retry = 5);
  std::map<uint8_t, ServoState> sync_read_present_states(
    const std::vector<uint8_t> & motor_ids,
    int num_retry = kDefaultNumRetry,
    int header_timeout_ms = kDefaultHeaderTimeoutMs);

  void enable_torque(const std::vector<uint8_t> & motor_ids, bool enable);
  void unlock_eeprom(const std::vector<uint8_t> & motor_ids);
  void lock_eeprom(const std::vector<uint8_t> & motor_ids);

private:
  // Instruction definitions
  static constexpr uint8_t INST_PING = 0x01;
  static constexpr uint8_t INST_READ = 0x02;
  static constexpr uint8_t INST_WRITE = 0x03;
  static constexpr uint8_t INST_REG_WRITE = 0x04;
  static constexpr uint8_t INST_ACTION = 0x05;
  static constexpr uint8_t INST_SYNC_READ = 0x82;
  static constexpr uint8_t INST_SYNC_WRITE = 0x83;

  // Broadcast ID
  static constexpr uint8_t BROADCAST_ID = 0xFE;

  static LibSerial::BaudRate get_libserial_baud_rate(int baudrate);

  std::string port_;
  int baudrate_;
  LibSerial::SerialPort serial_;
  std::mutex bus_mutex_;
};

}  // namespace nakalab_so101

#endif  // NAKALAB_SO101__FEETECH_BUS_HPP_
