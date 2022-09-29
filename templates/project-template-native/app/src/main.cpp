#include "mysqrt.hpp"
#include "isqrt.hpp"
#include <iostream>

int main()
{
  std::cout << "mysqrt(5) = " << Lib1::mysqrt(5.f) << std::endl;
  std::cout << "isqrt(5) = " << Lib2::isqrt(5.f) << std::endl;
}
