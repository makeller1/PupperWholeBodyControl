
#include "Utils.h"

template <typename T, size_t N>
T Utils::Maximum(const std::array<T, N> &array) {
  T max = array[0];
  for (size_t i = 0; i < N; i++) {
    max = array[i] >= max ? array[i] : max;
  }
  return max;
}

template <typename T, size_t N>
T Utils::Minimum(const std::array<T, N> &array) {
  T min = array[0];
  for (size_t i = 0; i < N; i++) {
    min = array[i] <= min ? array[i] : min;
  }
  return min;
}

template <typename T, size_t N>
std::array<T, N> Utils::Constrain(std::array<T, N> array, T min, T max) {
  std::array<T, N> out;
  for (size_t i = 0; i < array.size(); i++) {
    out[i] = (array[i] <= min) ? min : (array[i] >= max ? max : array[i]);
  }
  return out;
}

template <class T, size_t N>
std::array<T, N> Utils::MaskArray(std::array<T, N> &array,
                                  const std::array<bool, N> mask) {
  std::array<T, N> out;
  for (size_t i = 0; i < N; i++) {
    out[i] = mask[i] ? array[i] : T(0);
  }
  return out;
}

template <class T, size_t N>
std::array<T, N> Utils::ElemMultiply(const std::array<T, N> first,
                                     const std::array<T, N> second) {
  std::array<T, N> out;
  for (size_t i = 0; i < N; i++) {
    out[i] = first[i] * second[i];
  }
  return out;
}

template <size_t N>
std::array<int32_t, N> Utils::ConvertToFixedPoint(std::array<float, N> in,
                                                  float factor) {
  std::array<int32_t, N> out;
  for (size_t i = 0; i < N; i++) {
    out[i] = in[i] * factor;
  }
  return out;
}

// TODO: enforce that N==UN
template <int N, size_t UN>
std::array<float, UN> Utils::VectorToArray(BLA::Matrix<N> vec) {
  std::array<float, UN> out;
  for (int i = 0; i < N; i++) {
    out[i] = vec(i);
  }
  return out;
}

template <int N, size_t UN>
BLA::Matrix<N> Utils::ArrayToVector(std::array<float, UN> arr) {
  BLA::Matrix<N> out;
  for (unsigned int i = 0; i < UN; i++) {
    out(i) = arr[i];
  }
  return out;
}

template float Utils::Maximum(const std::array<float, 12> &array);
template float Utils::Minimum(const std::array<float, 12> &array);
template std::array<float, 12> Utils::Constrain(std::array<float, 12> array,
                                                float min, float max);
template std::array<float, 12> Utils::MaskArray(
    std::array<float, 12> &array, const std::array<bool, 12> mask);
template std::array<float, 12> Utils::ElemMultiply(
    const std::array<float, 12> first, const std::array<float, 12> second);
template std::array<int32_t, 12> Utils::ConvertToFixedPoint(
    std::array<float, 12> in, float factor);
template std::array<float, 12> Utils::VectorToArray(BLA::Matrix<12> vec);
template BLA::Matrix<12> Utils::ArrayToVector(std::array<float, 12> arr);

// start timer (ms)
double Utils::tic(int mode) {
    static double t_start;
    
    if (mode==0)
        t_start = micros();
    else {
        auto t_end = micros();
        double t_delta = (t_end-t_start)*1E-3;
        return t_delta;
    }
    return 0.0;
}
// return elapsed time (ms)
double Utils::toc() { return tic(1); }