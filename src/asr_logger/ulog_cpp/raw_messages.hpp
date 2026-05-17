/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <cstdint>

namespace ulog_cpp {

// NOLINTBEGIN(*) Keep the original naming

enum class ULogMessageType : uint8_t {
  FORMAT = 'F',
  DATA = 'D',
  INFO = 'I',
  INFO_MULTIPLE = 'M',
  PARAMETER = 'P',
  PARAMETER_DEFAULT = 'Q',
  ADD_LOGGED_MSG = 'A',
  REMOVE_LOGGED_MSG = 'R',
  SYNC = 'S',
  DROPOUT = 'O',
  LOGGING = 'L',
  LOGGING_TAGGED = 'C',
  FLAG_BITS = 'B',
};

/* declare message data structs with byte alignment (no padding) */
#pragma pack(push, 1)

/** first bytes of the file */
struct ulog_file_header_s {
  uint8_t magic[8];
  uint64_t timestamp;
};

static constexpr uint8_t ulog_file_magic_bytes[] = {'U', 'L', 'o', 'g', 0x01, 0x12, 0x35};

/** first bytes of the crypto key file */
struct ulog_key_header_s {
  /* magic identifying the file content */
  uint8_t magic[7];

  /* version of this header file */
  uint8_t hdr_ver;

  /* file creation timestamp */
  uint64_t timestamp;

  /* crypto algorithm used for key exchange */
  uint8_t exchange_algorithm;

  /* encryption key index used for key exchange */
  uint8_t exchange_key;

  /* size of the key */
  uint16_t key_size;

  /* size of logfile crypto algoritm initialization data, e.g. nonce */
  uint16_t initdata_size;

  /* actual data (initdata+key) */
  uint8_t data[1];  // [0] is a GCC extension; [1] is the ISO C++ portable equivalent
};

/**
 * @brief Message Header for the ULog
 *
 * This header components that is in the beginning of every ULog messages that gets written into
 * Definitions section as well as the Data section of the ULog file.
 */
struct ulog_message_header_s {
  uint16_t msg_size;  ///< Size of the message excluding the header size
  uint8_t
      msg_type;  ///< Message type, which is one of the ASCII alphabet, defined in ULogMessageType
};

#define ULOG_MSG_HEADER_LEN \
  3  // Length of the header in bytes: accounts for msg_size (2 bytes) and msg_type (1 byte)

struct ulog_message_format_s {
  uint16_t msg_size;  ///< size of message - ULOG_MSG_HEADER_LEN
  uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::FORMAT);

  char format[1500];
};

struct ulog_message_add_logged_s {
  uint16_t msg_size;  ///< size of message - ULOG_MSG_HEADER_LEN
  uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::ADD_LOGGED_MSG);

  uint8_t multi_id;
  uint16_t msg_id;
  char message_name[255];
};

struct ulog_message_remove_logged_s {
  uint16_t msg_size;  ///< size of message - ULOG_MSG_HEADER_LEN
  uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::REMOVE_LOGGED_MSG);

  uint16_t msg_id;
};

struct ulog_message_sync_s {
  uint16_t msg_size;  ///< size of message - ULOG_MSG_HEADER_LEN
  uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::SYNC);

  uint8_t sync_magic[8];
};

struct ulog_message_dropout_s {
  uint16_t msg_size = sizeof(uint16_t);  ///< size of message - ULOG_MSG_HEADER_LEN
  uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::DROPOUT);

  uint16_t duration;  ///< in ms
};

struct ulog_message_data_s {
  uint16_t msg_size;  ///< size of message - ULOG_MSG_HEADER_LEN
  uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::DATA);

  uint16_t msg_id;
};

struct ulog_message_info_s {
  uint16_t msg_size;  ///< size of message - ULOG_MSG_HEADER_LEN
  uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::INFO);

  uint8_t key_len;
  char key_value_str[255];
};

struct ulog_message_info_multiple_s {
  uint16_t msg_size;  ///< size of message - ULOG_MSG_HEADER_LEN
  uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::INFO_MULTIPLE);

  uint8_t is_continued;
  uint8_t key_len;
  char key_value_str[1200];
};

struct ulog_message_logging_s {
  uint16_t msg_size;  ///< size of message - ULOG_MSG_HEADER_LEN
  uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::LOGGING);

  uint8_t log_level;
  uint64_t timestamp;
  char message[128];
};

struct ulog_message_logging_tagged_s {
  uint16_t msg_size;  ///< size of message - ULOG_MSG_HEADER_LEN
  uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::LOGGING_TAGGED);

  uint8_t log_level;
  uint16_t tag;
  uint64_t timestamp;
  char message[128];
};

struct ulog_message_parameter_s {
  uint16_t msg_size;
  uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::PARAMETER);

  uint8_t key_len;
  char key_value_str[255];
};

enum class ulog_parameter_default_type_t : uint8_t {
  system = (1 << 0),
  current_setup = (1 << 1)
};

inline ulog_parameter_default_type_t operator|(ulog_parameter_default_type_t a,
                                               ulog_parameter_default_type_t b)
{
  return static_cast<ulog_parameter_default_type_t>(static_cast<uint8_t>(a) |
                                                    static_cast<uint8_t>(b));
}

struct ulog_message_parameter_default_s {
  uint16_t msg_size;
  uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::PARAMETER_DEFAULT);

  ulog_parameter_default_type_t default_types;
  uint8_t key_len;
  char key_value_str[255];
};

#define ULOG_INCOMPAT_FLAG0_DATA_APPENDED_MASK (1 << 0)

#define ULOG_COMPAT_FLAG0_DEFAULT_PARAMETERS_MASK (1 << 0)

struct ulog_message_flag_bits_s {
  uint16_t msg_size;
  uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::FLAG_BITS);

  uint8_t compat_flags[8];
  uint8_t incompat_flags[8];
  uint64_t appended_offsets[3];
};

#pragma pack(pop)

// NOLINTEND(*)

}  // namespace ulog_cpp
