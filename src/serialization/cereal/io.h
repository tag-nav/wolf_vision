#ifndef _WOLF_SERIALIZATION_CEREAL_IO_H_
#define _WOLF_SERIALIZATION_CEREAL_IO_H_

#include <stdexcept>
#include "archives.h"

//#include <cereal/types/tuple.hpp>

namespace wolf {
namespace serialization {

inline std::string extension(const std::string& file)
{
  const std::size_t p = file.find_last_of(".");
  return (p != std::string::npos) ? file.substr(p) : "";
}

//struct Extensions
//{
//  constexpr static const char* bin  = ".bin";
//  constexpr static const char* json = ".json";
//  constexpr static const char* xml  = ".xml";

//  constexpr static const char* fall_back = json;
//};

//enum class Extensions2 : std::size_t
//{
//  BIN = 0,
////  CBIN,
//  JSON,
////  TEXT,
//  XML,
//};

//template <char... Chars>
//struct constexp_str
//{
//  using type = constexp_str<Chars...>;

//  virtual ~constexp_str() = default;

//  constexpr static const char value[sizeof...(Chars)+1] = {Chars..., '\0'};

//  constexpr static std::size_t size() { return sizeof...(Chars); }

//  constexpr static const char* c_str() { return &value[0]; }

////  constexpr static bool comp(const std::string& s) { return s == value; }

//  /*constexpr*/ bool operator == (const std::string& s) { return s == value; }

//  constexpr /*static*/ operator const char* ()  { return c_str(); }
//  constexpr /*static*/ operator std::string& () { return c_str(); }
//};

struct Extensions
{
//  template <char... Chars>
//  struct EXT : constexp_str<Chars...>
//  {
//    //
//  };

//  struct BIN  : EXT<'.','b','i','n'> { };
//  struct XML  : EXT<'.','x','m','l'> { };
//  struct JSON : EXT<'.','j','s','o','n'> { };

  struct EXT { virtual ~EXT() = default; };

  struct BIN : EXT
  {
    constexpr static const char* value  = ".bin";
    bool operator == (const std::string& s) { return value == s; }
  };

  struct XML : EXT
  {
    constexpr static const char* value  = ".xml";
    bool operator == (const std::string& s) { return value == s; }
  };

  struct JSON : EXT
  {
    constexpr static const char* value  = ".json";
    bool operator == (const std::string& s) { return value == s; }
  };
};

template <typename Ar>
void serialize_pack(Ar&&)
{
  // end of expansion
}

/// @todo demangle typeid.name ?

template <typename Ar, typename T, typename... Args>
void serialize_pack(Ar&& archive, T&& object, Args&&... args)
{
  archive( cereal::make_nvp(typeid(T).name(), std::forward<T>(object)) );
  serialize_pack(archive, std::forward<Args>(args)...);
}

template <typename Ar, typename S, typename T>
void serialize(S& stream, T&& object)
{
  Ar archive(stream);
  archive( cereal::make_nvp(typeid(T).name(), std::forward<T>(object)) );
}

template <typename Ar, typename S, typename T, typename... Args>
void serialize(S& stream, T&& object, Args&&... args)
{
  Ar archive(stream);
  archive( cereal::make_nvp(typeid(T).name(), std::forward<T>(object)) );

  serialize_pack(archive, std::forward<Args>(args)...);
}

template <typename EXT, typename InAr, typename OutAr>
struct Serializer
{
  template <typename S, typename... T>
  static void serialize_in(S& stream, T&... object)
  {
    serialize<InAr>(stream, object...);
  }

  template <typename S, typename... T>
  static void serialize_out(S& stream, T&&... object)
  {
    serialize<OutAr>(stream, std::forward<T>(object)...);
  }

  template <typename... T>
  static void save(std::string filename, T&&... o)
  {
    const std::string ext = serialization::extension(filename);

    if (ext != EXT::value) filename += EXT::value;

    std::ofstream os(filename);
    serialize_out(os, std::forward<T>(o)...);
  }

  template <typename... T>
  static void load(std::string filename, T&... o)
  {
    const std::string ext = serialization::extension(filename);

    if (ext != EXT::value) filename += EXT::value;

    std::ifstream is(filename);
    serialize_in(is, o...);
  }
};

using SerializerBin = Serializer<Extensions::BIN,
                                 cereal::BinaryInputArchive,
                                 cereal::BinaryOutputArchive>;

using SerializerXML = Serializer<Extensions::XML,
                                 cereal::XMLInputArchive,
                                 cereal::XMLOutputArchive>;

using SerializerJSON = Serializer<Extensions::JSON,
                                  cereal::JSONInputArchive,
                                  cereal::JSONOutputArchive>;


} /* namespace serialization */

template <typename... T>
void save(const std::string& filename, T&&... o)
throw(std::runtime_error)
{
  const std::string ext = serialization::extension(filename);

  if (ext == serialization::Extensions::BIN::value)
  {
    serialization::SerializerBin::save(filename, std::forward<T>(o)...);
  }
  else if (ext == serialization::Extensions::JSON::value)
  {
    serialization::SerializerJSON::save(filename, std::forward<T>(o)...);
  }
  else if (ext == serialization::Extensions::XML::value)
  {
    serialization::SerializerXML::save(filename, std::forward<T>(o)...);
  }
  else if (ext == "") // falback is json
  {
    serialization::SerializerJSON::save(filename, std::forward<T>(o)...);
  }
  else
  {
    throw std::runtime_error("Unknown file extension : " + filename);
  }
}

template <typename... T>
void load(const std::string& filename, T&... o)
{
  const std::string ext = serialization::extension(filename);

  if (ext == serialization::Extensions::BIN::value)
  {
    serialization::SerializerBin::load(filename, o...);
  }
  else if (ext == serialization::Extensions::XML::value)
  {
    serialization::SerializerXML::load(filename, o...);
  }
  else if (ext == serialization::Extensions::JSON::value)
  {
    serialization::SerializerJSON::load(filename, o...);
  }
  else if (ext == "") // falback is json
  {
    serialization::SerializerJSON::load(filename, o...);
  }
  else
  {
    throw std::runtime_error("Unknown file extension : " + filename);
  }
}

} /* namespace wolf */

#endif /* _WOLF_SERIALIZATION_CEREAL_IO_H_ */
