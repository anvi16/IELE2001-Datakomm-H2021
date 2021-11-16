// ArduinoJson - https://arduinojson.org
// Copyright Benoit Blanchon 2014-2021
// MIT License

#pragma once

#include "../Configuration.cpp"
#include "../Namespace.cpp"

namespace ARDUINOJSON_NAMESPACE {

#if ARDUINOJSON_USE_DOUBLE
typedef double Float;
#else
typedef float Float;
#endif
}  // namespace ARDUINOJSON_NAMESPACE
