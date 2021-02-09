// Copyright 2020 F1TENTH Foundation
//
// Redistribution and use in source and binary forms, with or without modification, are permitted
// provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this list of conditions
//    and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice, this list
//    of conditions and the following disclaimer in the documentation and/or other materials
//    provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors may be used
//    to endorse or promote products derived from this software without specific prior
//    written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
// IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
// WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// -*- mode:c++; fill-column: 100; -*-

#include "vesc_driver/vesc_packet.h"

#include <cassert>
#include <iterator>
#include <memory>
#include <string>

#include "vesc_driver/vesc_packet_factory.h"
#include "vesc_driver/datatypes.h"

namespace vesc_driver
{

constexpr CRC::Parameters<crcpp_uint16, 16> VescFrame::CRC_TYPE;

VescFrame::VescFrame(int payload_size)
{
  assert(payload_size >= 0 && payload_size <= 1024);

  if (payload_size < 256)
  {
    // single byte payload size
    frame_.reset(new Buffer(VESC_MIN_FRAME_SIZE + payload_size));
    *frame_->begin() = 2;
    *(frame_->begin() + 1) = payload_size;
    payload_.first = frame_->begin() + 2;
  }
  else
  {
    // two byte payload size
    frame_.reset(new Buffer(VESC_MIN_FRAME_SIZE + 1 + payload_size));
    *frame_->begin() = 3;
    *(frame_->begin() + 1) = payload_size >> 8;
    *(frame_->begin() + 2) = payload_size & 0xFF;
    payload_.first = frame_->begin() + 3;
  }

  payload_.second = payload_.first + payload_size;
  *(frame_->end() - 1) = 3;
}

VescFrame::VescFrame(const BufferRangeConst& frame, const BufferRangeConst& payload)
{
  /* VescPacketFactory::createPacket() should make sure that the input is valid, but run a few cheap
     checks anyway */
  assert(std::distance(frame.first, frame.second) >= VESC_MIN_FRAME_SIZE);
  assert(std::distance(frame.first, frame.second) <= VESC_MAX_FRAME_SIZE);
  assert(std::distance(payload.first, payload.second) <= VESC_MAX_PAYLOAD_SIZE);
  assert(std::distance(frame.first, payload.first) > 0 &&
         std::distance(payload.second, frame.second) > 0);

  frame_.reset(new Buffer(frame.first, frame.second));
  payload_.first = frame_->begin() + std::distance(frame.first, payload.first);
  payload_.second = frame_->begin() + std::distance(frame.first, payload.second);
}

VescPacket::VescPacket(const std::string& name, int payload_size, int payload_id) :
  VescFrame(payload_size), name_(name)
{
  assert(payload_id >= 0 && payload_id < 256);
  assert(std::distance(payload_.first, payload_.second) > 0);
  *payload_.first = payload_id;
}

VescPacket::VescPacket(const std::string& name, std::shared_ptr<VescFrame> raw) :
  VescFrame(*raw), name_(name)
{
}

/*------------------------------------------------------------------------------------------------*/

VescPacketFWVersion::VescPacketFWVersion(std::shared_ptr<VescFrame> raw) :
  VescPacket("FWVersion", raw)
{
}

int VescPacketFWVersion::fwMajor() const
{
  return *(payload_.first + 1);
}

int VescPacketFWVersion::fwMinor() const
{
  return *(payload_.first + 2);
}

REGISTER_PACKET_TYPE(COMM_FW_VERSION, VescPacketFWVersion)

VescPacketRequestFWVersion::VescPacketRequestFWVersion() :
  VescPacket("RequestFWVersion", 1, COMM_FW_VERSION)
{
  uint16_t crc = CRC::Calculate(
    &(*payload_.first), std::distance(payload_.first, payload_.second), VescFrame::CRC_TYPE);
  *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------------------------------------*/

VescPacketValues::VescPacketValues(std::shared_ptr<VescFrame> raw) :
  VescPacket("Values", raw)
{
}

/*
typedef struct {

if (mask & ((uint32_t)1 << 0)) {
            buffer_append_float16(send_buffer, mc_interface_temp_fet_filtered(), 1e1, &ind); 0-1
        }
        if (mask & ((uint32_t)1 << 1)) {
            buffer_append_float16(send_buffer, mc_interface_temp_motor_filtered(), 1e1, &ind); 2-3
        }
        if (mask & ((uint32_t)1 << 2)) {
            buffer_append_float32(send_buffer, mc_interface_read_reset_avg_motor_current(), 1e2, &ind); 4-5-6-7
        }
        if (mask & ((uint32_t)1 << 3)) {
            buffer_append_float32(send_buffer, mc_interface_read_reset_avg_input_current(), 1e2, &ind); 8-9-10-11
        }
        if (mask & ((uint32_t)1 << 4)) {
            buffer_append_float32(send_buffer, mc_interface_read_reset_avg_id(), 1e2, &ind); 12
        }
        if (mask & ((uint32_t)1 << 5)) {
            buffer_append_float32(send_buffer, mc_interface_read_reset_avg_iq(), 1e2, &ind); 16
        }
        if (mask & ((uint32_t)1 << 6)) {
            buffer_append_float16(send_buffer, mc_interface_get_duty_cycle_now(), 1e3, &ind); 20
        }
        if (mask & ((uint32_t)1 << 7)) {
            buffer_append_float32(send_buffer, mc_interface_get_rpm(), 1e0, &ind); 22
        }
        if (mask & ((uint32_t)1 << 8)) {
            buffer_append_float16(send_buffer, GET_INPUT_VOLTAGE(), 1e1, &ind); 26
        }
        if (mask & ((uint32_t)1 << 9)) {
            buffer_append_float32(send_buffer, mc_interface_get_amp_hours(false), 1e4, &ind); 28
        }
        if (mask & ((uint32_t)1 << 10)) {
            buffer_append_float32(send_buffer, mc_interface_get_amp_hours_charged(false), 1e4, &ind);  32
        }
        if (mask & ((uint32_t)1 << 11)) {
            buffer_append_float32(send_buffer, mc_interface_get_watt_hours(false), 1e4, &ind);  36
        }
        if (mask & ((uint32_t)1 << 12)) {
            buffer_append_float32(send_buffer, mc_interface_get_watt_hours_charged(false), 1e4, &ind);  40
        }
        if (mask & ((uint32_t)1 << 13)) {
            buffer_append_int32(send_buffer, mc_interface_get_tachometer_value(false), &ind);  44
        }
        if (mask & ((uint32_t)1 << 14)) {
            buffer_append_int32(send_buffer, mc_interface_get_tachometer_abs_value(false), &ind); 48
        }
        if (mask & ((uint32_t)1 << 15)) {
            send_buffer[ind++] = mc_interface_get_fault(); 52
        }
        if (mask & ((uint32_t)1 << 16)) {
            buffer_append_float32(send_buffer, mc_interface_get_pid_pos_now(), 1e6, &ind);  53
        }
        if (mask & ((uint32_t)1 << 17)) {
            uint8_t current_controller_id = app_get_configuration()->controller_id;
#ifdef HW_HAS_DUAL_MOTORS
            if (mc_interface_get_motor_thread() == 2) {
                current_controller_id = utils_second_motor_id();
            }
#endif
            send_buffer[ind++] = current_controller_id;    57
        }
        if (mask & ((uint32_t)1 << 18)) {
            buffer_append_float16(send_buffer, NTC_TEMP_MOS1(), 1e1, &ind);   58
            buffer_append_float16(send_buffer, NTC_TEMP_MOS2(), 1e1, &ind);   60
            buffer_append_float16(send_buffer, NTC_TEMP_MOS3(), 1e1, &ind);   62
        }
        if (mask & ((uint32_t)1 << 19)) {
            buffer_append_float32(send_buffer, mc_interface_read_reset_avg_vd(), 1e3, &ind);
        }
        if (mask & ((uint32_t)1 << 20)) {
            buffer_append_float32(send_buffer, mc_interface_read_reset_avg_vq(), 1e3, &ind);
        }

}
*/

static int32_t GetInt32FromBuffer(const BufferRangeConst &buffer, size_t offset)
{
    return static_cast<int32_t>((static_cast<uint32_t>(*(buffer.first + offset + 0)) << 24) +
                                (static_cast<uint32_t>(*(buffer.first + offset + 1)) << 16) +
                                (static_cast<uint32_t>(*(buffer.first + offset + 2)) << 8) +
                                static_cast<uint32_t>(*(buffer.first + offset + 3)));
}

static int32_t GetInt16FromBuffer(const BufferRangeConst &buffer, size_t offset)
{
    return static_cast<int16_t>((static_cast<uint16_t>(*(buffer.first + offset + 0)) << 8) +
                                static_cast<uint16_t>(*(buffer.first + offset + 1)));
}

double VescPacketValues::temp_mos1() const
{
  int32_t v = 0;
  return static_cast<double>(v) / 10.0;
}

double VescPacketValues::temp_mos2() const
{
  int16_t v = 0;
  return static_cast<double>(v) / 10.0;
}

double VescPacketValues::current_motor() const
{
  int32_t v = GetInt32FromBuffer(payload_, 5);
  return static_cast<double>(v) / 100.0;
}

double VescPacketValues::current_in() const
{
  int32_t v = GetInt32FromBuffer(payload_, 9);
  return static_cast<double>(v) / 100.0;
}


double VescPacketValues::duty_now() const
{
  int16_t v = GetInt16FromBuffer(payload_, 21);
  return static_cast<double>(v) / 1000.0;
}

double VescPacketValues::rpm() const
{
  int32_t v = GetInt32FromBuffer(payload_, 23);
  return static_cast<double>(-1 * v);
}

double VescPacketValues::amp_hours() const
{
  int32_t v = GetInt32FromBuffer(payload_, 29);
  return static_cast<double>(v);
}

double VescPacketValues::amp_hours_charged() const
{
  int32_t v = GetInt32FromBuffer(payload_, 33);
  return static_cast<double>(v);
}

double VescPacketValues::tachometer() const
{
  int32_t v = GetInt32FromBuffer(payload_, 45);
  return static_cast<double>(v);
}

double VescPacketValues::tachometer_abs() const
{
  int32_t v = GetInt32FromBuffer(payload_, 49);
  return static_cast<double>(v);
}

int VescPacketValues::fault_code() const
{
  return static_cast<int32_t>(*(payload_.first + 53));
}

double VescPacketValues::v_in() const
{
  int32_t v = GetInt16FromBuffer(payload_, 27);
  return static_cast<double>(v / 10.);
}

double VescPacketValues::temp_pcb() const
{
  int32_t v = 0;
  return static_cast<double>(v);
}

double VescPacketValues::watt_hours() const
{
  int32_t v = 0;
  return static_cast<double>(v);
}

double VescPacketValues::watt_hours_charged() const
{
  int32_t v = 0;
  return static_cast<double>(v);
}

REGISTER_PACKET_TYPE(COMM_GET_VALUES, VescPacketValues)

VescPacketRequestValues::VescPacketRequestValues() :
  VescPacket("RequestValues", 1, COMM_GET_VALUES)
{
  uint16_t crc = CRC::Calculate(
    &(*payload_.first), std::distance(payload_.first, payload_.second), VescFrame::CRC_TYPE);
  *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------------------------------------*/


VescPacketSetDuty::VescPacketSetDuty(double duty) :
  VescPacket("SetDuty", 5, COMM_SET_DUTY)
{
  /** @todo range check duty */

  int32_t v = static_cast<int32_t>(duty * 100000.0);

  *(payload_.first + 1) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 24) & 0xFF);
  *(payload_.first + 2) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 16) & 0xFF);
  *(payload_.first + 3) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 8) & 0xFF);
  *(payload_.first + 4) = static_cast<uint8_t>(static_cast<uint32_t>(v) & 0xFF);

  uint16_t crc = CRC::Calculate(
    &(*payload_.first), std::distance(payload_.first, payload_.second), VescFrame::CRC_TYPE);
  *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------------------------------------*/

VescPacketSetCurrent::VescPacketSetCurrent(double current) :
  VescPacket("SetCurrent", 5, COMM_SET_CURRENT)
{
  int32_t v = static_cast<int32_t>(current * 1000.0);

  *(payload_.first + 1) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 24) & 0xFF);
  *(payload_.first + 2) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 16) & 0xFF);
  *(payload_.first + 3) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 8) & 0xFF);
  *(payload_.first + 4) = static_cast<uint8_t>(static_cast<uint32_t>(v) & 0xFF);

  uint16_t crc = CRC::Calculate(
    &(*payload_.first), std::distance(payload_.first, payload_.second), VescFrame::CRC_TYPE);
  *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------------------------------------*/

VescPacketSetCurrentBrake::VescPacketSetCurrentBrake(double current_brake) :
  VescPacket("SetCurrentBrake", 5, COMM_SET_CURRENT_BRAKE)
{
  int32_t v = static_cast<int32_t>(current_brake * 1000.0);

  *(payload_.first + 1) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 24) & 0xFF);
  *(payload_.first + 2) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 16) & 0xFF);
  *(payload_.first + 3) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 8) & 0xFF);
  *(payload_.first + 4) = static_cast<uint8_t>(static_cast<uint32_t>(v) & 0xFF);

  uint16_t crc = CRC::Calculate(
    &(*payload_.first), std::distance(payload_.first, payload_.second), VescFrame::CRC_TYPE);
  *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------------------------------------*/

VescPacketSetRPM::VescPacketSetRPM(double rpm) :
  VescPacket("SetRPM", 5, COMM_SET_RPM)
{
  int32_t v = static_cast<int32_t>(rpm);

  *(payload_.first + 1) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 24) & 0xFF);
  *(payload_.first + 2) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 16) & 0xFF);
  *(payload_.first + 3) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 8) & 0xFF);
  *(payload_.first + 4) = static_cast<uint8_t>(static_cast<uint32_t>(v) & 0xFF);

  uint16_t crc = CRC::Calculate(
    &(*payload_.first), std::distance(payload_.first, payload_.second), VescFrame::CRC_TYPE);
  *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------------------------------------*/

VescPacketSetPos::VescPacketSetPos(double pos) :
  VescPacket("SetPos", 5, COMM_SET_POS)
{
  /** @todo range check pos */

  int32_t v = static_cast<int32_t>(pos * 1000000.0);

  *(payload_.first + 1) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 24) & 0xFF);
  *(payload_.first + 2) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 16) & 0xFF);
  *(payload_.first + 3) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 8) & 0xFF);
  *(payload_.first + 4) = static_cast<uint8_t>(static_cast<uint32_t>(v) & 0xFF);

  uint16_t crc = CRC::Calculate(
    &(*payload_.first), std::distance(payload_.first, payload_.second), VescFrame::CRC_TYPE);
  *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------------------------------------*/

VescPacketSetServoPos::VescPacketSetServoPos(double servo_pos) :
  VescPacket("SetServoPos", 3, COMM_SET_SERVO_POS)
{
  /** @todo range check pos */

  int16_t v = static_cast<int16_t>(servo_pos * 1000.0);

  *(payload_.first + 1) = static_cast<uint8_t>((static_cast<uint16_t>(v) >> 8) & 0xFF);
  *(payload_.first + 2) = static_cast<uint8_t>(static_cast<uint16_t>(v) & 0xFF);

  uint16_t crc = CRC::Calculate(
    &(*payload_.first), std::distance(payload_.first, payload_.second), VescFrame::CRC_TYPE);
  *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

}  // namespace vesc_driver
