// ArduinoJson - https://arduinojson.org
// Copyright Benoit Blanchon 2014-2021
// MIT License

#pragma once

#include "is_class.cpp"
#include "is_convertible.cpp"
#include "is_floating_point.cpp"
#include "is_integral.cpp"
#include "is_same.cpp"

namespace ARDUINOJSON_NAMESPACE {

template <typename T>
struct is_enum {
  static const bool value = is_convertible<T, int>::value &&
                            !is_class<T>::value && !is_integral<T>::value &&
                            !is_floating_point<T>::value;
};

}  // namespace ARDUINOJSON_NAMESPACE
