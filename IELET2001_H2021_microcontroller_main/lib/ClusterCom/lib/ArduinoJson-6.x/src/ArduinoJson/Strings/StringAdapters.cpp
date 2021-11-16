// ArduinoJson - https://arduinojson.org
// Copyright Benoit Blanchon 2014-2021
// MIT License

#pragma once

#include "../Strings/ConstRamStringAdapter.cpp"
#include "../Strings/RamStringAdapter.cpp"
#include "../Strings/SizedRamStringAdapter.cpp"

#if ARDUINOJSON_ENABLE_STD_STRING
#include "../Strings/StdStringAdapter.cpp"
#endif

#if ARDUINOJSON_ENABLE_ARDUINO_STRING
#include "../Strings/ArduinoStringAdapter.cpp"
#endif

#if ARDUINOJSON_ENABLE_PROGMEM
#include "../Strings/FlashStringAdapter.cpp"
#include "../Strings/SizedFlashStringAdapter.cpp"
#endif
