#include <isqrt.hpp>
#include <mysqrt.hpp>

#include <iostream>

int main()
{
  using namespace Mfunc;
  std::cout << "mysqrt(5.0f) = " << mysqrt(5.0f) << std::endl;
  std::cout << "isqrt(5.0f) = " << isqrt(5.0f) << std::endl;
}
