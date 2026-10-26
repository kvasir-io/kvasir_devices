/// Every chip description includes the description vocabulary (Engine/Description.hpp), never the engine: a chip
/// header that pulls in a Device.hpp or Engine/Engine.hpp makes this file fail to compile. Engine/Hooks.hpp, which only
/// the engine includes, defines KVASIR_ENGINE_HOOKS.
#include <kvasir/Devices/Engine/Groups.hpp>
#include <kvasir/Devices/I2C/chips/All.hpp>

#ifdef KVASIR_ENGINE_HOOKS
    #error \
      "a chip description (or Groups.hpp) includes the engine: include ../../Engine/Description.hpp instead of ../Device.hpp"
#endif

int main() { return 0; }
