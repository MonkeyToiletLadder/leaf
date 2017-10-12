#ifndef _VECTOR_HPP
#define _VECTOR_HPP

#include "type_traits.hpp"

#include <math.h>

namespace uti {

  struct Vector{ double x,y; };

  using Scalar  = double;
  using Point   = Vector;

  inline Vector operator-( cr< Vector > lhs, cr< Vector > rhs ) { return { lhs.x - rhs.x, lhs.y - rhs.y }; }
  inline Vector operator+( cr< Vector > lhs, cr< Vector > rhs ) { return { lhs.x + rhs.x, lhs.y + rhs.y }; }

  inline Scalar operator*( cr< Vector > lhs, cr< Vector > rhs ) { return lhs.x * rhs.x + lhs.y * rhs.y; }

  inline Vector operator*( Scalar lhs, cr< Vector > rhs ) { return { lhs * rhs.x, lhs * rhs.y }; }
  inline Vector operator*( cr< Vector > lhs, Scalar rhs ) { return { lhs.x * rhs, lhs.y * rhs }; }
  inline Vector operator/( cr< Vector > lhs, Scalar rhs ) { return lhs * ( 1 / rhs ); }

  inline Scalar mag( cr< Vector > vector ) { return sqrt( vector * vector ); }
  inline Vector hat( cr< Vector > vector ) { return vector / mag( vector ); }
  inline Vector nrm( cr< Vector > vector ) { return { -vector.y, vector.x }; }
};

#endif