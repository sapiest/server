// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.
// See the LICENSE file in the project root for more information.
//
// complex.h
//

//
// Defines a basic complex number data type.
// https://github.com/dotnet/runtime/blob/master/src/coreclr/src/inc/complex.h
//

#pragma once

#include <cmath>
class complex
{
public:
  double r;
  double i;

  complex() : r(0), i(0) {}
  complex(double real) : r(real), i(0) {}
  complex(double real, double imag) : r(real), i(imag) {}
  complex(const complex &other) : r(other.r), i(other.i) {}
};

inline complex operator+(const complex& left, const complex& right)
{
  return complex(left.r + right.r, left.i + right.i);
}

inline complex operator-(const complex& left, const complex& right)
{
  return complex(left.r - right.r, left.i - right.i);
}

inline complex operator*(const complex& left, const complex& right)
{
  return complex(left.r * right.r - left.i * right.i,
                 left.r * right.i + left.i * right.r);
}

inline complex operator/(const complex& left, const complex& right)
{
  double denom = right.r * right.r + right.i * right.i;
  return complex((left.r * right.r + left.i * right.i) / denom,
                 (-left.r * right.i + left.i * right.r) / denom);
}

inline double abs(const complex& num)
{
    return sqrt(num.r * num.r + num.i * num.i);
}
