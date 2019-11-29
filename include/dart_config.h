#pragma once

#include <string>

#ifndef TYSOC_BACKEND_PHYSICS_DART
    #define TYSOC_BACKEND_PHYSICS_DART "../libtysocPhysicsDart.so"
#endif

namespace tysoc {
namespace config {

    namespace physics
    {
        const std::string DART = std::string( TYSOC_BACKEND_PHYSICS_DART );
    }

}}