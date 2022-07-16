#include "mysqrt.hpp"

namespace Mfunc
{
  float mysqrt(float x)
  {
    float rt = 1, ort = 0;
    while(ort!=rt)
    {
        ort = rt;
        rt = ((x/rt) + rt) / 2;
    }
    return rt;
  }
}
