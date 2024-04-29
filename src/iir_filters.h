#include "arm_math.h"

class FirstOrderFilter
{
    public:
        FirstOrderFilter(float32_t Ts, float32_t tau);
        float32_t step(float32_t input);
        float32_t get_value();
    private:
        float32_t _Ts;
        float32_t _tau;
        float32_t _a[2];
        float32_t _b[2];
        float32_t _yk[2];
};

