#include "iir_filters.h"


FirstOrderFilter::FirstOrderFilter(float32_t Ts, float32_t tau) {
	//if (tau < Ts) return -1;
	_Ts = Ts;
	_tau = tau;
        _a[0] = 0.0;
        _b[0] = 0.0;
	_a[1] = -( 1.0 - Ts / tau); // -exp(-Ts/tau)
	_b[1] = 1 + _a[1];
};

float32_t FirstOrderFilter::step(float32_t input) {
	_yk[0] = _b[1] * input - _a[1] * _yk[1];	
        _yk[1] = _yk[0];
	return _yk[0];
}

float32_t FirstOrderFilter::get_value() {
    return _yk[0];
}

