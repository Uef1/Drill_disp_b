#pragma once
#include <Arduino.h>

// фильтр скользящее среднее
template <typename T>
class IntEMA {
   public:
    // k2 - коэффициент как степень двойки
    T filter(T val, uint8_t k2) {
        T sum = (val - _filt) + _err;
        T div = sum >> k2;
        _err = sum - (div << k2);
        return _filt += div;
    }

    void init(T val) {
        _filt = val;
        _err = 0;
    }

    T get() {
        return _filt;
    }

    operator T() {
        return _filt;
    }

   private:
    T _filt = 0, _err = 0;
};