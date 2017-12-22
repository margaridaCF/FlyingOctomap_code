#ifndef POINT2D_LIB_H
#define POINT2D_LIB_H

#include <iostream>
#include <string>
#include <stdexcept>
#include <cmath>
#include <geometry_msgs/Vector3.h>

namespace mapper
{
	class Point2d
	{
	private:

	public:
	  float x;
	  float y;
	  Point2d(float x=0, float y=0)
	  	:x(x),y(y){};
	  Point2d(geometry_msgs::Vector3 position)
	  {
	  	x = position.x;
	  	y = position.y;
	  };
	  Point2d& operator+=(Point2d const& z2);
	  Point2d operator+(Point2d const& z2) const;
	  Point2d& operator+=(float const& constant);
	  Point2d operator+(float const& constant) const;
	  Point2d& operator-=(Point2d const& z2);
	  Point2d operator-(Point2d const& z2) const;
	  Point2d& operator-=(float const& constant);
	  Point2d operator-(float const& constant) const;
	  Point2d& operator*=(float const& constant);
	  Point2d operator*(float const& constant) const;
	  Point2d operator/(float const& denominator) const;
	  bool operator==(Point2d const& z2) const;

	  Point2d ceil() const;
	  float distance (Point2d const& p2) const;

	  std::ostream& displayString(std::ostream& sortie) const;
	  std::string displayString() const;
	  friend std::ostream& operator<<(std::ostream& s, const Point2d& c);
	};

};	

#endif // POINT2D_LIB_H