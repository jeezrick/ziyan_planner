#include "pathplanner/planner_io.hpp"
#include "pathplanner/logger.hpp"

namespace ziyan_planner
{
  Point & Point::operator=(const Point & other)
  {
    // check for self assignement
    if (this == &other) {
      return *this;
    }

    x = other.x;
    y = other.y;
    return *this;
  };

  Position& Position::operator=(const Position& other) 
  {
    // check for self assignement
    if (this == &other) {
      return *this;
    }

    this->x = other.x;
    this->y = other.y;
    this->z = other.z;
    return *this;
  }

  Quaternion& Quaternion::operator=(const Quaternion& other) 
  {
    // check for self assignement
    if (this == &other) {
      return *this;
    }

    this->x = other.x;
    this->y = other.y;
    this->z = other.z;
    this->w = other.w;
    return *this;
  }

  Pose& Pose::operator=(const Pose& other) 
  {
    // check for self assignement
    if (this == &other) {
      return *this;
    }

    this->position = other.position;
    this->orientation = other.orientation;
    return *this;
  }

  void Quaternion::setEuler(const double& yaw, const double& pitch, const double& roll)
  {
    double halfYaw = double(yaw) * double(0.5);  
    double halfPitch = double(pitch) * double(0.5);  
    double halfRoll = double(roll) * double(0.5);  
    double cosYaw = cos(halfYaw);
    double sinYaw = sin(halfYaw);
    double cosPitch = cos(halfPitch);
    double sinPitch = sin(halfPitch);
    double cosRoll = cos(halfRoll);
    double sinRoll = sin(halfRoll);
    setValue(cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw,
      cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw,
      sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw,
      cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw);
  }

  void Quaternion::setRPY(const double& roll, const double& pitch, const double& yaw)
	{
		double halfYaw = double(yaw) * double(0.5);  
		double halfPitch = double(pitch) * double(0.5);  
		double halfRoll = double(roll) * double(0.5);  
		double cosYaw = cos(halfYaw);
		double sinYaw = sin(halfYaw);
		double cosPitch = cos(halfPitch);
		double sinPitch = sin(halfPitch);
		double cosRoll = cos(halfRoll);
		double sinRoll = sin(halfRoll);
		setValue(sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw, //x
      cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw, //y
      cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw, //z
      cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw); //formerly yzx
	}

}
