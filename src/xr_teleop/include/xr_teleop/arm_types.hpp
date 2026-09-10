#ifndef ARM_TYPES_HPP
#define ARM_TYPES_HPP

#include <string>

enum Arm 
{ 
  LEFT = 1, 
  RIGHT,
  LAST 
};

inline std::string arm_to_str(Arm arm)
{
  switch (arm)
  {
    case LEFT:
      return "left";
    case RIGHT:
      return "right";
    case LAST:
      return "last";
    default:
      return "unknown";
  }
}

#endif // ARM_TYPES_HPP