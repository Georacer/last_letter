#include <sstream>
#include <stdexcept>

#include "last_letter_lib/body/body.hpp"

namespace last_letter_lib {
namespace body {

std::unique_ptr<Body> buildBody(ParameterManager config)
{
    const int bodyType = config.get<int>("bodyType");
    const string body_name = config.get<string>("name");

    std::unique_ptr<Body> body;
    std::cout << "building body model type " << bodyType << ":\n";
    switch (bodyType)
    {
    case 0:
        std::cout << "selecting the gravity-only body" << std::endl;
        body = std::make_unique<Body>(body_name);
        break;
    default:
        std::stringstream ss;
        ss << "Error while constructing body: Unknown type " << bodyType;
        throw runtime_error(ss.str());
    }
    body->initialize(config);
    return body;
}

} // namespace body
} // namespace last_letter_lib
