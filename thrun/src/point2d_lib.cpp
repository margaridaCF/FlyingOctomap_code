#include <point2d_lib.h>

using namespace std;
namespace mapper
{
	///  PLUS
	Point2d& Point2d::operator+=(Point2d const& z2)
	{
		x = x + z2.x;
		y = y + z2.y;
	  	return *this;
	}
	Point2d Point2d::operator+(Point2d const& z2) const
	{
	  return Point2d ( *this) += z2;
	}
	Point2d& Point2d::operator+=(float const& constant)
	{
		x = x + constant;
		y = y + constant;
	  	return *this;
	}
	Point2d Point2d::operator+(float const& constant) const
	{
	  return Point2d ( *this) += constant;
	}

	///  MINUS
	Point2d& Point2d::operator-=(Point2d const& z2)
	{
		x = x - z2.x;
		y = y - z2.y;
	  	return *this;
	}
	Point2d Point2d::operator-(Point2d const& z2) const
	{
	  return Point2d ( *this) -= z2;
	}
	Point2d& Point2d::operator-=(float const& constant)
	{
		x = x - constant;
		y = y - constant;
	  	return *this;
	}
	Point2d Point2d::operator-(float const& constant) const
	{
	  return Point2d ( *this) -= constant;
	}

	///  ==
	bool Point2d::operator==(Point2d const& z2) const
	{
	  	return (x == z2.x) && (y == z2.y);
	}

	/// MULTIPLICATION
	Point2d& Point2d::operator*=(float const& constant)
	{
		x = x * constant;
		y = y * constant;
	  	return *this;
	}
	Point2d Point2d::operator*(float const& constant) const
	{
	  return Point2d ( *this) *= constant;
	}

	///  DIVISION
	Point2d Point2d::operator/(float const& denominator) const
	{
		if (denominator == 0)
			throw std::overflow_error("Divide by zero exception");
	  	return Point2d( x / denominator, y / denominator);
	}

	///  Math functions
	Point2d Point2d::ceil() const
	{
		return Point2d(std::ceil(x), std::ceil(y));
	}
	float Point2d::distance (Point2d const& p2) const
	{
		return std::sqrt(  std::pow(x - p2.x, 2) + std::pow(y - p2.y, 2) );
	}

	///  Display  and  <<
	std::string Point2d::displayString() const
	{
	  return "(" + std::to_string(x) + "; "+ std::to_string(y) + " )";
	}
	std::ostream& Point2d::displayString(std::ostream& stream_out) const
	{
	  stream_out << "(" << x << "; " << y << " )";
	  stream_out.precision(3);
	  return stream_out;
	}
	ostream& operator<<(ostream& s, const Point2d& c){
		return c.displayString(s);
	};

}


