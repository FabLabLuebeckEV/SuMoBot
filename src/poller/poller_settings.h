#pragma once

#include "hardware_config.h"

namespace poller {
namespace settings {

bool loadParameters(hardware::PollerParameters* out);
bool saveParameters(const hardware::PollerParameters& params);

}  // namespace settings
}  // namespace poller

