// ArduinoJson - https://arduinojson.org
// Copyright Benoit Blanchon 2014-2021
// MIT License

#pragma once

#include <stddef.h>
#include <stdint.h>  // for uint8_t

#include "../Memory/MemoryPool.hpp"
#include "../Misc/Visitable.hpp"
#include "../Polyfills/type_traits.cpp"
#include "../Strings/StringAdapters.cpp"
#include "../Variant/Converter.cpp"
#include "../Variant/VariantFunctions.cpp"
#include "../Variant/VariantOperators.cpp"
#include "../Variant/VariantShortcuts.cpp"
#include "../Variant/VariantTag.cpp"

namespace ARDUINOJSON_NAMESPACE {

// Forward declarations.
class ArrayRef;
class ObjectRef;

// Contains the methods shared by VariantRef and VariantConstRef
template <typename TData>
class VariantRefBase : public VariantTag {
 public:
  FORCE_INLINE bool isNull() const {
    return variantIsNull(_data);
  }

  FORCE_INLINE bool isUndefined() const {
    return !_data;
  }

  FORCE_INLINE size_t memoryUsage() const {
    return _data ? _data->memoryUsage() : 0;
  }

  FORCE_INLINE size_t nesting() const {
    return _data ? _data->nesting() : 0;
  }

  size_t size() const {
    return variantSize(_data);
  }

 protected:
  VariantRefBase(TData *data) : _data(data) {}
  TData *_data;

  friend TData *getData(const VariantRefBase &variant) {
    return variant._data;
  }
};

// A variant that can be a any value serializable to a JSON value.
//
// It can be set to:
// - a boolean
// - a char, short, int or a long (signed or unsigned)
// - a string (const char*)
// - a reference to a ArrayRef or ObjectRef
class VariantRef : public VariantRefBase<VariantData>,
                   public VariantOperators<VariantRef>,
                   public VariantShortcuts<VariantRef>,
                   public Visitable {
  typedef VariantRefBase<VariantData> base_type;
  friend class VariantConstRef;

 public:
  // Intenal use only
  FORCE_INLINE VariantRef(MemoryPool *pool, VariantData *data)
      : base_type(data), _pool(pool) {}

  // Creates an uninitialized VariantRef
  FORCE_INLINE VariantRef() : base_type(0), _pool(0) {}

  FORCE_INLINE void clear() const {
    return variantSetNull(_data);
  }

  template <typename T>
  FORCE_INLINE bool set(const T &value) const {
    return Converter<T>::toJson(*this, value);
  }

  template <typename T>
  FORCE_INLINE bool set(T *value) const {
    return Converter<T *>::toJson(*this, value);
  }

  template <typename T>
  FORCE_INLINE T as() const {
    return Converter<T>::fromJson(*this);
  }

  template <typename T>
  FORCE_INLINE bool is() const {
    return Converter<T>::checkJson(*this);
  }

  template <typename T>
  FORCE_INLINE operator T() const {
    return Converter<T>::fromJson(*this);
  }

  template <typename TVisitor>
  typename TVisitor::result_type accept(TVisitor &visitor) const {
    return variantAccept(_data, visitor);
  }

  // Change the type of the variant
  //
  // ArrayRef to<ArrayRef>()
  template <typename T>
  typename enable_if<is_same<T, ArrayRef>::value, ArrayRef>::type to() const;
  //
  // ObjectRef to<ObjectRef>()
  template <typename T>
  typename enable_if<is_same<T, ObjectRef>::value, ObjectRef>::type to() const;
  //
  // ObjectRef to<VariantRef>()
  template <typename T>
  typename enable_if<is_same<T, VariantRef>::value, VariantRef>::type to()
      const;

  VariantRef addElement() const;

  FORCE_INLINE VariantRef getElement(size_t) const;

  FORCE_INLINE VariantRef getOrAddElement(size_t) const;

  // getMember(const char*) const
  // getMember(const __FlashStringHelper*) const
  template <typename TChar>
  FORCE_INLINE VariantRef getMember(TChar *) const;

  // getMember(const std::string&) const
  // getMember(const String&) const
  template <typename TString>
  FORCE_INLINE typename enable_if<IsString<TString>::value, VariantRef>::type
  getMember(const TString &) const;

  // getOrAddMember(char*) const
  // getOrAddMember(const char*) const
  // getOrAddMember(const __FlashStringHelper*) const
  template <typename TChar>
  FORCE_INLINE VariantRef getOrAddMember(TChar *) const;

  // getOrAddMember(const std::string&) const
  // getOrAddMember(const String&) const
  template <typename TString>
  FORCE_INLINE VariantRef getOrAddMember(const TString &) const;

  FORCE_INLINE void remove(size_t index) const {
    if (_data)
      _data->remove(index);
  }
  // remove(char*) const
  // remove(const char*) const
  // remove(const __FlashStringHelper*) const
  template <typename TChar>
  FORCE_INLINE typename enable_if<IsString<TChar *>::value>::type remove(
      TChar *key) const {
    if (_data)
      _data->remove(adaptString(key));
  }
  // remove(const std::string&) const
  // remove(const String&) const
  template <typename TString>
  FORCE_INLINE typename enable_if<IsString<TString>::value>::type remove(
      const TString &key) const {
    if (_data)
      _data->remove(adaptString(key));
  }

 private:
  MemoryPool *_pool;

  friend MemoryPool *getPool(const VariantRef &variant) {
    return variant._pool;
  }
};

class VariantConstRef : public VariantRefBase<const VariantData>,
                        public VariantOperators<VariantConstRef>,
                        public VariantShortcuts<VariantConstRef>,
                        public Visitable {
  typedef VariantRefBase<const VariantData> base_type;
  friend class VariantRef;

 public:
  VariantConstRef() : base_type(0) {}
  VariantConstRef(const VariantData *data) : base_type(data) {}
  VariantConstRef(VariantRef var) : base_type(var._data) {}

  template <typename TVisitor>
  typename TVisitor::result_type accept(TVisitor &visitor) const {
    return variantAccept(_data, visitor);
  }

  template <typename T>
  FORCE_INLINE T as() const {
    return Converter<T>::fromJson(*this);
  }

  template <typename T>
  FORCE_INLINE bool is() const {
    return Converter<T>::checkJson(*this);
  }

  template <typename T>
  FORCE_INLINE operator T() const {
    return Converter<T>::fromJson(*this);
  }

  FORCE_INLINE VariantConstRef getElement(size_t) const;

  FORCE_INLINE VariantConstRef operator[](size_t index) const {
    return getElement(index);
  }

  // getMember(const std::string&) const
  // getMember(const String&) const
  template <typename TString>
  FORCE_INLINE VariantConstRef getMember(const TString &key) const {
    return VariantConstRef(
        objectGetMember(variantAsObject(_data), adaptString(key)));
  }

  // getMember(char*) const
  // getMember(const char*) const
  // getMember(const __FlashStringHelper*) const
  template <typename TChar>
  FORCE_INLINE VariantConstRef getMember(TChar *key) const {
    const CollectionData *obj = variantAsObject(_data);
    return VariantConstRef(obj ? obj->getMember(adaptString(key)) : 0);
  }

  // operator[](const std::string&) const
  // operator[](const String&) const
  template <typename TString>
  FORCE_INLINE
      typename enable_if<IsString<TString>::value, VariantConstRef>::type
      operator[](const TString &key) const {
    return getMember(key);
  }

  // operator[](char*) const
  // operator[](const char*) const
  // operator[](const __FlashStringHelper*) const
  template <typename TChar>
  FORCE_INLINE
      typename enable_if<IsString<TChar *>::value, VariantConstRef>::type
      operator[](TChar *key) const {
    return getMember(key);
  }
};

template <>
struct Converter<VariantRef> {
  static bool toJson(VariantRef variant, VariantRef value) {
    return variantCopyFrom(getData(variant), getData(value), getPool(variant));
  }
  static VariantRef fromJson(VariantRef variant) {
    return variant;
  }
  static bool checkJson(VariantRef variant) {
    VariantData *data = getData(variant);
    return !!data;
  }
  static bool checkJson(VariantConstRef) {
    return false;
  }
};

template <>
struct Converter<VariantConstRef> {
  static bool toJson(VariantRef variant, VariantConstRef value) {
    return variantCopyFrom(getData(variant), getData(value), getPool(variant));
  }

  static VariantConstRef fromJson(VariantConstRef variant) {
    return VariantConstRef(getData(variant));
  }

  static bool checkJson(VariantConstRef variant) {
    const VariantData *data = getData(variant);
    return !!data;
  }
};

}  // namespace ARDUINOJSON_NAMESPACE
