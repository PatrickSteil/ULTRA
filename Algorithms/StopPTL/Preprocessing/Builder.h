#pragma once

#include "../../../DataStructures/StopPTL/Data.h"

namespace StopPTL {
class Builder {
public:
  Builder(Data &data) : data(data) {}

private:
  Data &data;
};

} // namespace StopPTL
