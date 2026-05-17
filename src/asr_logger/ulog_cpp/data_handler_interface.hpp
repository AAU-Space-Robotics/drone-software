/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <string>

#include "messages.hpp"

namespace ulog_cpp {

class DataHandlerInterface {
 public:
  virtual void headerComplete() {}

  virtual void error(const std::string&, bool) {}

  // Data methods
  virtual void fileHeader(const FileHeader&) {}
  virtual void messageInfo(const MessageInfo&) {}
  virtual void messageFormat(const MessageFormat&) {}
  virtual void parameter(const Parameter&) {}
  virtual void parameterDefault(const ParameterDefault&) {}
  virtual void addLoggedMessage(const AddLoggedMessage&) {}
  virtual void logging(const Logging&) {}
  virtual void data(const Data&) {}
  virtual void dropout(const Dropout&) {}
  virtual void sync(const Sync&) {}

 private:
};

}  // namespace ulog_cpp
