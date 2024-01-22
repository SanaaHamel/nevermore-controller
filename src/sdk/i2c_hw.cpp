#include "i2c_hw.hpp"

namespace nevermore {

std::array<I2C_HW, 2> i2c{*i2c0, *i2c1};

}
