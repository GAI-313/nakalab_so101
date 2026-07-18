#include "nakalab_so101/FeetechBus.hpp"
#include <vector>
#include <numeric>
#include <stdexcept>
#include <iostream>

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

bool FeetechBus::write_packet(uint8_t motor_id, uint8_t instruction, const std::vector<uint8_t> & parameters)
{
  std::lock_guard<std::mutex> lock(bus_mutex_);
  std::vector<uint8_t> packet;
  packet.push_back(0xFF);
  packet.push_back(0xFF);
  packet.push_back(motor_id);
  packet.push_back(parameters.size() + 2);
  packet.push_back(instruction);
  packet.insert(packet.end(), parameters.begin(), parameters.end());

  uint8_t checksum = 0;
  for (size_t i = 2; i < packet.size(); ++i) {
    checksum += packet[i];
  }
  checksum = ~checksum;
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

std::optional<std::vector<uint8_t>> FeetechBus::read_packet()
{
  std::lock_guard<std::mutex> lock(bus_mutex_);
  try {
    uint8_t byte;
    int header_count = 0;
    while(header_count < 2) {
        serial_.ReadByte(byte, 50);
        if (byte == 0xFF) {
            header_count++;
        } else {
            header_count = 0;
        }
    }

    uint8_t id, length, error;
    serial_.ReadByte(id, 10);
    serial_.ReadByte(length, 10);
    serial_.ReadByte(error, 10);

    if (error != 0) {
        // Error from servo
    }
    
    if (length < 2) {
        return std::nullopt;
    }

    std::vector<uint8_t> params_and_checksum(length - 1);
    serial_.Read(params_and_checksum, length - 1, 50);

    uint8_t checksum_received = params_and_checksum.back();
    params_and_checksum.pop_back();

    uint8_t checksum_calculated = id + length + error;
    for (const auto& p : params_and_checksum) {
        checksum_calculated += p;
    }
    checksum_calculated = ~checksum_calculated;

    if (checksum_calculated != checksum_received) {
        std::cerr << "Checksum error on read." << std::endl;
        return std::nullopt;
    }
    
    return params_and_checksum;

  } catch (const std::exception &) {
    // This will catch timeouts and other read errors.
    // We don't print an error because timeouts are expected.
    return std::nullopt;
  }
}


bool FeetechBus::write_register(uint8_t motor_id, uint8_t address, const std::vector<uint8_t> & data)
{
    std::vector<uint8_t> params;
    params.push_back(address);
    params.insert(params.end(), data.begin(), data.end());
    if (!write_packet(motor_id, INST_WRITE, params)) {
        return false;
    }
    read_packet();
    return true;
}

std::optional<std::vector<uint8_t>> FeetechBus::read_register(uint8_t motor_id, uint8_t address, uint8_t length)
{
    if (!write_packet(motor_id, INST_READ, {address, length})) {
        return std::nullopt;
    }
    return read_packet();
}

std::optional<int> FeetechBus::read_homing_offset(uint8_t motor_id)
{
    const auto response = read_register(motor_id, REG_HOMING_OFFSET, 2);
    if (!response || response->size() != 2) {
        return std::nullopt;
    }

    const uint16_t raw_value =
        static_cast<uint16_t>((*response)[0]) |
        (static_cast<uint16_t>((*response)[1]) << 8);
    const int magnitude = static_cast<int>(raw_value & 0x07FF);
    return raw_value & 0x0800 ? -magnitude : magnitude;
}

bool FeetechBus::sync_write_goal_positions(const std::map<uint8_t, int16_t>& motor_goal_map)
{
    std::vector<uint8_t> params;
    params.push_back(REG_GOAL_POSITION);
    params.push_back(2);

    for (const auto & [id, pos] : motor_goal_map) {
        params.push_back(id);
        params.push_back(static_cast<uint8_t>(pos & 0xFF));
        params.push_back(static_cast<uint8_t>((pos >> 8) & 0xFF));
    }
    return write_packet(BROADCAST_ID, INST_SYNC_WRITE, params);
}

bool FeetechBus::sync_write_goal_states(const std::map<uint8_t, ServoState>& motor_states)
{
    std::vector<uint8_t> params;
    params.push_back(REG_GOAL_POSITION);
    params.push_back(6);

    for (const auto & [id, state] : motor_states) {
        params.push_back(id);
        int16_t pos = state.position;
        int16_t time = 0;
        int16_t speed = state.speed;
        params.push_back(static_cast<uint8_t>(pos & 0xFF));
        params.push_back(static_cast<uint8_t>((pos >> 8) & 0xFF));
        params.push_back(static_cast<uint8_t>(time & 0xFF));
        params.push_back(static_cast<uint8_t>((time >> 8) & 0xFF));
        params.push_back(static_cast<uint8_t>(speed & 0xFF));
        params.push_back(static_cast<uint8_t>((speed >> 8) & 0xFF));
    }
    return write_packet(BROADCAST_ID, INST_SYNC_WRITE, params);
}


std::map<uint8_t, int16_t> FeetechBus::sync_read_present_positions(const std::vector<uint8_t>& motor_ids)
{
    std::map<uint8_t, int16_t> positions;
    for (const auto& id : motor_ids) {
        auto response = read_register(id, REG_PRESENT_POSITION, 2);
        if (response && response->size() == 2) {
            int16_t pos = ((*response)[1] << 8) | (*response)[0];
            positions[id] = pos;
        }
    }
    return positions;
}

std::map<uint8_t, ServoState> FeetechBus::sync_read_present_states(const std::vector<uint8_t>& motor_ids)
{
    std::map<uint8_t, ServoState> states;
    for (const auto& id : motor_ids) {
        auto response = read_register(id, REG_PRESENT_POSITION, 6);
        if (response && response->size() == 6) {
            ServoState state;
            state.position = ((*response)[1] << 8) | (*response)[0];
            state.speed = ((*response)[3] << 8) | (*response)[2];
            state.load = ((*response)[5] << 8) | (*response)[4];
            states[id] = state;
        }
    }
    return states;
}

void FeetechBus::enable_torque(const std::vector<uint8_t>& motor_ids, bool enable)
{
    for (const auto& id : motor_ids) {
        write_register(id, REG_TORQUE_ENABLE, {static_cast<uint8_t>(enable ? 1 : 0)});
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

}
