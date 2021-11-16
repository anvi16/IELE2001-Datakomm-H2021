// ArduinoJson - https://arduinojson.org
// Copyright Benoit Blanchon 2014-2021
// MIT License

#pragma once

#include "../Collection/CollectionData.cpp"
#include "../Numbers/Float.hpp"
#include "../Numbers/Integer.hpp"

namespace ARDUINOJSON_NAMESPACE {

template <typename TResult>
struct Visitor {
  typedef TResult result_type;

  TResult visitArray(const CollectionData &) {
    return TResult();
  }

  TResult visitBoolean(bool) {
    return TResult();
  }

  TResult visitFloat(Float) {
    return TResult();
  }

  TResult visitSignedInteger(Integer) {
    return TResult();
  }

  TResult visitNull() {
    return TResult();
  }

  TResult visitObject(const CollectionData &) {
    return TResult();
  }

  TResult visitUnsignedInteger(UInt) {
    return TResult();
  }

  TResult visitRawJson(const char *, size_t) {
    return TResult();
  }

  TResult visitString(const char *) {
    return TResult();
  }
};

}  // namespace ARDUINOJSON_NAMESPACE
