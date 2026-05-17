/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <cstdio>
#include <memory>
// GCC 14.2 raises an error in <regex> when building with -fsanitize=address
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#endif
#include <regex>
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif
#include <unordered_map>
#include <vector>

#include "writer.hpp"

namespace ulog_cpp {

/**
 * ULog serialization class which checks for integrity and correct calling order.
 * It throws an UsageException() in case of a failed integrity check.
 */
class SimpleWriter {
 public:
  explicit SimpleWriter(DataWriteCB data_write_cb, uint64_t timestamp_us);
  explicit SimpleWriter(const std::string& filename, uint64_t timestamp_us);

  ~SimpleWriter();

  template <typename T>
  void writeInfo(const std::string& key, const T& value)
  {
    _writer->messageInfo(ulog_cpp::MessageInfo(key, value));
  }

  template <typename T>
  void writeParameter(const std::string& key, const T& value)
  {
    if (_header_complete) {
      throw UsageException("Header already complete");
    }
    _writer->parameter(ulog_cpp::Parameter(key, value));
  }

  void writeMessageFormat(const std::string& name, const std::vector<Field>& fields);

  void headerComplete();

  template <typename T>
  void writeParameterChange(const std::string& key, const T& value)
  {
    if (!_header_complete) {
      throw UsageException("Header not yet complete");
    }
    _writer->parameter(ulog_cpp::Parameter(key, value));
  }

  uint16_t writeAddLoggedMessage(const std::string& message_format_name, uint8_t multi_id = 0);

  void writeTextMessage(Logging::Level level, const std::string& message, uint64_t timestamp);

  template <typename T>
  void writeData(uint16_t id, const T& data)
  {
    writeDataImpl(id, reinterpret_cast<const uint8_t*>(&data), sizeof(data));
  }

  void fsync();

 private:
  static const std::string kFormatNameRegexStr;
  static const std::regex kFormatNameRegex;
  static const std::string kFieldNameRegexStr;
  static const std::regex kFieldNameRegex;

  struct Format {
    unsigned message_size;
  };
  struct Subscription {
    unsigned message_size;
  };

  void writeDataImpl(uint16_t id, const uint8_t* data, unsigned length);

  std::unique_ptr<Writer> _writer;
  std::FILE* _file{nullptr};

  bool _header_complete{false};
  std::unordered_map<std::string, Format> _formats;
  std::vector<Subscription> _subscriptions;
};

}  // namespace ulog_cpp
