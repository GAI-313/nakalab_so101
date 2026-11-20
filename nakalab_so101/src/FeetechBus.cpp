#include "nakalab_so101/FeetechBus.hpp"
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <vector>

namespace nakalab_so101
{

FeetechBus::FeetechBus(const std::string & port, int baudrate)
: port_(port), baudrate_(baudrate)
{
}

FeetechBus::~FeetechBus()
{
  if (is_connected()) {
    disconnect();
  }
}

bool FeetechBus::connect()
{
  if (is_connected()) {
    return true;
  }
  try {
    serial_.Open(port_);
    serial_.SetBaudRate(get_libserial_baud_rate(baudrate_));
    serial_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial_.SetParity(LibSerial::Parity::PARITY_NONE);
    serial_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
  } catch (const std::exception & e) {
    std::cerr << "Failed to open serial port " << port_ << ": " << e.what() << std::endl;
    return false;
  }
  return true;
}

void FeetechBus::disconnect()
{
  if (is_connected()) {
    serial_.Close();
  }
}

bool FeetechBus::is_connected() const
{
  return serial_.IsOpen();
}

int FeetechBus::decode_sign_magnitude(uint16_t raw_value, uint8_t sign_bit_index)
{
  const uint16_t sign_bit = static_cast<uint16_t>(1U << sign_bit_index);
  const uint16_t magnitude_mask = static_cast<uint16_t>(sign_bit - 1U);
  const int magnitude = static_cast<int>(raw_value & magnitude_mask);
  return (raw_value & sign_bit) ? -magnitude : magnitude;
}

uint16_t FeetechBus::encode_sign_magnitude(int value, uint8_t sign_bit_index)
{
  const uint16_t sign_bit = static_cast<uint16_t>(1U << sign_bit_index);
  const int max_magnitude = static_cast<int>(sign_bit - 1U);
  const int clamped_magnitude = std::min(std::abs(value), max_magnitude);
  return static_cast<uint16_t>((value < 0 ? sign_bit : 0U) |
         static_cast<uint16_t>(clamped_magnitude));
}

void FeetechBus::clear_input_buffer()
{
  std::lock_guard<std::mutex> lock(bus_mutex_);
  if (is_connected()) {
    try {
      serial_.FlushInputBuffer();
    } catch (const std::exception &) {
    }
  }
}

bool FeetechBus::write_packet(
  uint8_t motor_id,
  uint8_t instruction,
  const std::vector<uint8_t> & parameters)
{
  std::lock_guard<std::mutex> lock(bus_mutex_);
  std::vector<uint8_t> packet;
  packet.push_back(0xFF);
  packet.push_back(0xFF);
  packet.push_back(motor_id);
  packet.push_back(static_cast<uint8_t>(parameters.size() + 2));
  packet.push_back(instruction);
  packet.insert(packet.end(), parameters.begin(), parameters.end());

  uint8_t checksum = 0;
  for (size_t i = 2; i < packet.size(); ++i) {
    checksum += packet[i];
  }
  checksum = static_cast<uint8_t>(~checksum);
  packet.push_back(checksum);

  try {
    serial_.Write(packet);
    serial_.DrainWriteBuffer();
  } catch (const std::exception & e) {
    std::cerr << "Serial write failed: " << e.what() << std::endl;
    return false;
  }
  return true;
}

std::optional<FeetechStatusPacket> FeetechBus::read_status_packet(
  std::optional<uint8_t> expected_motor_id,
  int header_timeout_ms)
{
  std::lock_guard<std::mutex> lock(bus_mutex_);
  const size_t timeout_ms = static_cast<size_t>(std::max(1, header_timeout_ms));
  try {
    while (true) {
      uint8_t byte = 0;
      int header_count = 0;
      while (header_count < 2) {
        serial_.ReadByte(byte, timeout_ms);
        if (byte == 0xFF) {
          header_count++;
        } else {
          header_count = 0;
        }
      }

      uint8_t id = 0;
      uint8_t length = 0;
      uint8_t error = 0;
      serial_.ReadByte(id, 10);
      serial_.ReadByte(length, 10);
      serial_.ReadByte(error, 10);

      if (length < 2) {
        continue;
      }

      std::vector<uint8_t> params_and_checksum(length - 1);
      serial_.Read(params_and_checksum, length - 1, 50);

      uint8_t checksum_received = params_and_checksum.back();
      params_and_checksum.pop_back();

      uint8_t checksum_calculated = static_cast<uint8_t>(id + length + error);
      for (const auto & p : params_and_checksum) {
        checksum_calculated = static_cast<uint8_t>(checksum_calculated + p);
      }
      checksum_calculated = static_cast<uint8_t>(~checksum_calculated);

      if (checksum_calculated != checksum_received) {
        std::cerr << "Checksum error on read." << std::endl;
        continue;
      }

      if (expected_motor_id && id != *expected_motor_id) {
        continue;
      }

      FeetechStatusPacket status;
      status.motor_id = id;
      status.error = error;
      status.parameters = std::move(params_and_checksum);
      return status;
    }
  } catch (const std::exception &) {
    // Catch timeouts and other read errors.
    return std::nullopt;
  }
}

std::optional<std::vector<uint8_t>> FeetechBus::read_packet()
{
  auto status = read_status_packet();
  if (!status) {
    return std::nullopt;
  }
  return status->parameters;
}

bool FeetechBus::ping(uint8_t motor_id, int num_retry, int header_timeout_ms)
{
  const int attempts = std::max(0, num_retry) + 1;
  for (int i = 0; i < attempts; ++i) {
    clear_input_buffer();
    if (!write_packet(motor_id, INST_PING, {})) {
      continue;
    }
    const auto status = read_status_packet(motor_id, header_timeout_ms);
    if (status && status->error == 0) {
      return true;
    }
  }
  return false;
}

bool FeetechBus::write_register(
  uint8_t motor_id,
  uint8_t address,
  const std::vector<uint8_t> & data,
  int num_retry)
{
  std::vector<uint8_t> params;
  params.push_back(address);
  params.insert(params.end(), data.begin(), data.end());

  const int attempts = std::max(0, num_retry) + 1;
  for (int i = 0; i < attempts; ++i) {
    clear_input_buffer();
    if (!write_packet(motor_id, INST_WRITE, params)) {
      continue;
    }
    if (motor_id == BROADCAST_ID) {
      return true;
    }
    const auto status = read_status_packet(motor_id);
    if (status && status->error == 0) {
      return true;
    }
  }
  return false;
}

std::optional<std::vector<uint8_t>> FeetechBus::read_register(
  uint8_t motor_id,
  uint8_t address,
  uint8_t length,
  int num_retry,
  int header_timeout_ms)
{
  const int attempts = std::max(0, num_retry) + 1;
  for (int i = 0; i < attempts; ++i) {
    clear_input_buffer();
    if (!write_packet(motor_id, INST_READ, {address, length})) {
      continue;
    }
    const auto status = read_status_packet(motor_id, header_timeout_ms);
    if (status && status->error == 0 && status->parameters.size() == length) {
      return status->parameters;
    }
  }
  return std::nullopt;
}

std::optional<uint8_t> FeetechBus::read_u8(uint8_t motor_id, uint8_t address)
{
  const auto response = read_register(motor_id, address, 1);
  if (!response || response->size() != 1) {
    return std::nullopt;
  }
  return (*response)[0];
}

std::optional<uint16_t> FeetechBus::read_u16(uint8_t motor_id, uint8_t address)
{
  const auto response = read_register(motor_id, address, 2);
  if (!response || response->size() != 2) {
    return std::nullopt;
  }
  return static_cast<uint16_t>((*response)[0]) |
         (static_cast<uint16_t>((*response)[1]) << 8);
}

bool FeetechBus::write_u8(uint8_t motor_id, uint8_t address, uint8_t value)
{
  return write_register(motor_id, address, {value});
}

bool FeetechBus::write_u16(uint8_t motor_id, uint8_t address, uint16_t value)
{
  return write_register(
    motor_id, address,
    {
      static_cast<uint8_t>(value & 0xFF),
      static_cast<uint8_t>((value >> 8) & 0xFF)
    });
}

std::optional<int> FeetechBus::read_homing_offset(uint8_t motor_id)
{
  const auto raw_value = read_u16(motor_id, REG_HOMING_OFFSET);
  if (!raw_value) {
    return std::nullopt;
  }
  return decode_sign_magnitude(*raw_value, HOMING_OFFSET_SIGN_BIT);
}

bool FeetechBus::write_homing_offset(uint8_t motor_id, int value)
{
  const int max_magnitude = (1 << HOMING_OFFSET_SIGN_BIT) - 1;
  if (std::abs(value) > max_magnitude) {
    return false;
  }
  const uint16_t raw_value = encode_sign_magnitude(value, HOMING_OFFSET_SIGN_BIT);
  return write_u16(motor_id, REG_HOMING_OFFSET, raw_value);
}

std::optional<double> FeetechBus::read_voltage(
  uint8_t motor_id,
  int num_retry,
  int header_timeout_ms)
{
  const auto response = read_register(
    motor_id, REG_PRESENT_VOLTAGE, 1, num_retry, header_timeout_ms);
  if (!response || response->size() != 1) {
    return std::nullopt;
  }
  return static_cast<double>((*response)[0]) * 0.1;
}

bool FeetechBus::sync_write_goal_positions(const std::map<uint8_t, int16_t> & motor_goal_map)
{
  std::vector<uint8_t> params;
  params.push_back(REG_GOAL_POSITION);
  params.push_back(2);

  for (const auto & [id, pos] : motor_goal_map) {
    const uint16_t encoded_pos = encode_sign_magnitude(pos, POSITION_SIGN_BIT);
    params.push_back(id);
    params.push_back(static_cast<uint8_t>(encoded_pos & 0xFF));
    params.push_back(static_cast<uint8_t>((encoded_pos >> 8) & 0xFF));
  }
  return write_packet(BROADCAST_ID, INST_SYNC_WRITE, params);
}

bool FeetechBus::sync_write_goal_states(const std::map<uint8_t, ServoState> & motor_states)
{
  std::vector<uint8_t> params;
  params.push_back(REG_GOAL_POSITION);
  params.push_back(6);

  for (const auto & [id, state] : motor_states) {
    params.push_back(id);
    const uint16_t pos = encode_sign_magnitude(state.position, POSITION_SIGN_BIT);
    const uint16_t time_val = 0;
    const uint16_t speed = static_cast<uint16_t>(state.speed);
    params.push_back(static_cast<uint8_t>(pos & 0xFF));
    params.push_back(static_cast<uint8_t>((pos >> 8) & 0xFF));
    params.push_back(static_cast<uint8_t>(time_val & 0xFF));
    params.push_back(static_cast<uint8_t>((time_val >> 8) & 0xFF));
    params.push_back(static_cast<uint8_t>(speed & 0xFF));
    params.push_back(static_cast<uint8_t>((speed >> 8) & 0xFF));
  }
  return write_packet(BROADCAST_ID, INST_SYNC_WRITE, params);
}

std::map<uint8_t, int16_t> FeetechBus::sync_read_present_positions(
  const std::vector<uint8_t> & motor_ids,
  int num_retry)
{
  std::map<uint8_t, int16_t> positions;
  for (const auto & id : motor_ids) {
    auto response = read_register(id, REG_PRESENT_POSITION, 2, num_retry);
    if (response && response->size() == 2) {
      const uint16_t raw_pos =
        static_cast<uint16_t>((*response)[0]) |
        (static_cast<uint16_t>((*response)[1]) << 8);
      positions[id] = static_cast<int16_t>(
        decode_sign_magnitude(raw_pos, POSITION_SIGN_BIT));
    }
  }
  return positions;
}

std::map<uint8_t, ServoState> FeetechBus::sync_read_present_states(
  const std::vector<uint8_t> & motor_ids,
  int num_retry,
  int header_timeout_ms)
{
  std::map<uint8_t, ServoState> states;
  for (const auto & id : motor_ids) {
    // Address 56..62 covers position (2B), speed (2B), load (2B), and voltage (1B).
    auto response = read_register(
      id, REG_PRESENT_POSITION, 7, num_retry, header_timeout_ms);
    if (response && response->size() == 7) {
      ServoState state;
      const uint16_t raw_pos =
        static_cast<uint16_t>((*response)[0]) |
        (static_cast<uint16_t>((*response)[1]) << 8);
      state.position = decode_sign_magnitude(raw_pos, POSITION_SIGN_BIT);
      state.speed =
        static_cast<int>((static_cast<uint16_t>((*response)[3]) << 8) | (*response)[2]);
      state.load =
        static_cast<int>((static_cast<uint16_t>((*response)[5]) << 8) | (*response)[4]);
      state.voltage = static_cast<double>((*response)[6]) * 0.1;
      states[id] = state;
    }
  }
  return states;
}

void FeetechBus::enable_torque(const std::vector<uint8_t> & motor_ids, bool enable)
{
  for (const auto & id : motor_ids) {
    write_register(id, REG_TORQUE_ENABLE, {static_cast<uint8_t>(enable ? 1 : 0)});
  }
}

void FeetechBus::unlock_eeprom(const std::vector<uint8_t> & motor_ids)
{
  for (const auto & id : motor_ids) {
    write_u8(id, REG_LOCK, 0);
  }
}

void FeetechBus::lock_eeprom(const std::vector<uint8_t> & motor_ids)
{
  for (const auto & id : motor_ids) {
    write_u8(id, REG_LOCK, 1);
  }
}

LibSerial::BaudRate FeetechBus::get_libserial_baud_rate(int baudrate)
{
  switch (baudrate) {
    case 9600:
      return LibSerial::BaudRate::BAUD_9600;
    case 19200:
      return LibSerial::BaudRate::BAUD_19200;
    case 38400:
      return LibSerial::BaudRate::BAUD_38400;
    case 57600:
      return LibSerial::BaudRate::BAUD_57600;
    case 115200:
      return LibSerial::BaudRate::BAUD_115200;
    case 230400:
      return LibSerial::BaudRate::BAUD_230400;
#ifdef __linux__
    case 460800:
      return LibSerial::BaudRate::BAUD_460800;
    case 500000:
      return LibSerial::BaudRate::BAUD_500000;
    case 576000:
      return LibSerial::BaudRate::BAUD_576000;
    case 921600:
      return LibSerial::BaudRate::BAUD_921600;
    case 1000000:
      return LibSerial::BaudRate::BAUD_1000000;
    case 1152000:
      return LibSerial::BaudRate::BAUD_1152000;
    case 1500000:
      return LibSerial::BaudRate::BAUD_1500000;
    case 2000000:
      return LibSerial::BaudRate::BAUD_2000000;
    case 2500000:
      return LibSerial::BaudRate::BAUD_2500000;
    case 3000000:
      return LibSerial::BaudRate::BAUD_3000000;
    case 3500000:
      return LibSerial::BaudRate::BAUD_3500000;
    case 4000000:
      return LibSerial::BaudRate::BAUD_4000000;
#endif
    default:
      return LibSerial::BaudRate::BAUD_INVALID;
  }
}

}  // namespace nakalab_so101
