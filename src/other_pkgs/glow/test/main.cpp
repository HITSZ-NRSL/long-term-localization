#include <gtest/gtest.h>

#include <glow/glbase.h>

#include <glow/util/X11OffscreenContext.h>
#include <glow/GlState.h>
#include <glow/GlCapabilities.h>

using namespace glow;

int main(int argc, char** argv) {
  X11OffscreenContext ctx(3,3);

  /* try it out */
  inititializeGLEW();

  if (argc > 1 && std::string(argv[1]) == "info") {
    std::cout << "Device's OpenGL Capabilities: \n" << std::endl;
    std::cout << GlCapabilities::getInstance() << std::endl;

    std::cout << "Device's current OpenGL state: \n" << std::endl;
    std::cout << GlState::queryAll() << std::endl;
  }
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
