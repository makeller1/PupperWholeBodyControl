#pragma once

#include <BasicLinearAlgebra.h>

#include <array>

class Utils {
 public:
  template <typename T, size_t N>
  static T Maximum(const std::array<T, N> &array);

  template <typename T, size_t N>
  static T Minimum(const std::array<T, N> &array);

  template <typename T, size_t N>
  static std::array<T, N> Constrain(std::array<T, N> array, T min, T max);

  template <class T, size_t N>
  static std::array<T, N> MaskArray(std::array<T, N> &array,
                                    const std::array<bool, N> mask);

  template <class T, size_t N>
  static std::array<T, N> ElemMultiply(const std::array<T, N> first,
                                       const std::array<T, N> second);
  template <size_t N>
  static std::array<int32_t, N> ConvertToFixedPoint(std::array<float, N> in,
                                                    float factor);

  template <int N, size_t UN>
  static std::array<float, UN> VectorToArray(BLA::Matrix<N> vec);

  template <int N, size_t UN>
  static BLA::Matrix<N> ArrayToVector(std::array<float, UN> arr);
};

