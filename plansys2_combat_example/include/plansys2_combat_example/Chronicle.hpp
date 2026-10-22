// Copyright 2026 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PLANSYS2_COMBAT_EXAMPLE__CHRONICLE_HPP_
#define PLANSYS2_COMBAT_EXAMPLE__CHRONICLE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"

namespace plansys2_combat_example
{

// What happens in the battle, told in a color of its own so that it stands out in the terminal:
// the arbiter in yellow, blue's controller in light cyan, red's in red
class Chronicle
{
public:
  Chronicle(const rclcpp::Logger & logger, const std::string & who, bool colors)
  : logger_(logger), color_(colors ? color_of(who) : ""), bold_(colors ? "\033[1m" : ""),
    reset_(colors ? "\033[0m" : "")
  {
  }

  void tell(const std::string & text) const
  {
    RCLCPP_INFO(logger_, "%s%s%s", color_.c_str(), text.c_str(), reset_.c_str());
  }

  // A turning point: brighter, and as a warning
  void announce(const std::string & text) const
  {
    RCLCPP_WARN(
      logger_, "%s%s%s%s", color_.c_str(), bold_.c_str(), text.c_str(), reset_.c_str());
  }

private:
  static std::string color_of(const std::string & who)
  {
    if (who == "arbiter") {
      return "\033[93m";  // yellow
    } else if (who == "blue") {
      return "\033[96m";  // light cyan: dark blue is hard to read on black
    } else if (who == "red") {
      return "\033[91m";
    }
    return "\033[95m";
  }

  rclcpp::Logger logger_;
  std::string color_;
  std::string bold_;
  std::string reset_;
};

}  // namespace plansys2_combat_example

#endif  // PLANSYS2_COMBAT_EXAMPLE__CHRONICLE_HPP_
