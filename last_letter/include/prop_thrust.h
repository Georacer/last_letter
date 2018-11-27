#ifndef last_letter_MESSAGE_prop_thrust_H
#define last_letter_MESSAGE_prop_thrust_H

#include <ros/service_traits.h>
#include <last_letter/prop_thrustRequest.h>
#include <last_letter/prop_thrustResponse.h>


namespace last_letter
{

struct prop_thrust
{

typedef prop_thrustRequest Request;
typedef prop_thrustResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct prop_thrust
} // namespace last_letter


namespace ros
{
namespace service_traits
{

template<>
struct MD5Sum< ::last_letter::prop_thrust > {
  static const char* value()
  {
    return "6a2e34150c00229791cc89ff309fff21";
  }

  static const char* value(const ::last_letter::prop_thrust&) { return value(); }
};

template<>
struct DataType< ::last_letter::prop_thrust > {
  static const char* value()
  {
    return "last_letter/prop_thrust";
  }

  static const char* value(const ::last_letter::prop_thrust&) { return value(); }
};


// service_traits::MD5Sum< ::last_letter::prop_thrustRequest> should match
// service_traits::MD5Sum< ::last_letter::prop_thrust >
template<>
struct MD5Sum< ::last_letter::prop_thrustRequest>
{
  static const char* value()
  {
    return MD5Sum< ::last_letter::prop_thrust >::value();
  }
  static const char* value(const ::last_letter::prop_thrustRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::last_letter::prop_thrustRequest> should match
// service_traits::DataType< ::last_letter::prop_thrust >
template<>
struct DataType< ::last_letter::prop_thrustRequest>
{
  static const char* value()
  {
    return DataType< ::last_letter::prop_thrust >::value();
  }
  static const char* value(const ::last_letter::prop_thrustRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::last_letter::prop_thrustResponse> should match
// service_traits::MD5Sum< ::last_letter::prop_thrust >
template<>
struct MD5Sum< ::last_letter::prop_thrustResponse>
{
  static const char* value()
  {
    return MD5Sum< ::last_letter::prop_thrust >::value();
  }
  static const char* value(const ::last_letter::prop_thrustResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::last_letter::prop_thrustResponse> should match
// service_traits::DataType< ::last_letter::prop_thrust >
template<>
struct DataType< ::last_letter::prop_thrustResponse>
{
  static const char* value()
  {
    return DataType< ::last_letter::prop_thrust >::value();
  }
  static const char* value(const ::last_letter::prop_thrustResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // last_letter_MESSAGE_prop_thrust_H
