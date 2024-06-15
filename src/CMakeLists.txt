add_definitions(-D_USE_MATH_DEFINES)

# ========== place libraries and executables in outermost dir ==========

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})


add_executable(CDTDemo CDTDemo/main.cpp core.h Triangulation.h Triangulation.cpp Grid.h Grid.cpp)
target_link_libraries(CDTDemo PRIVATE sfml-graphics sfml-window sfml-system ImGui-SFML::ImGui-SFML)

add_executable(ShadowsDemo Shadows/main.cpp core.h Shadows/VisibilityField.h Shadows/VisibilityField.cpp 
                Shadows/MapGrid.h Shadows/MapGrid.cpp 
                Triangulation.h Triangulation.cpp Grid.h Grid.cpp)
target_link_libraries(ShadowsDemo PRIVATE sfml-graphics sfml-window sfml-system ImGui-SFML::ImGui-SFML)


add_executable(PathFindingDemo PathFinding/main.cpp core.h PathFinding/PathFinder.h PathFinding/PathFinder.cpp
                Shadows/MapGrid.h Shadows/MapGrid.cpp PathFinding/ReducedTriangulationGraph.h PathFinding/ReducedTriangulationGraph.cpp
                Triangulation.h Triangulation.cpp Grid.h Grid.cpp)
target_link_libraries(PathFindingDemo PRIVATE sfml-graphics sfml-window sfml-system ImGui-SFML::ImGui-SFML)

if(WIN32)
if (BUILD_SHARED_LIBS)
add_custom_command(TARGET example_exe POST_BUILD
COMMAND ${CMAKE_COMMAND} -E copy_if_different
$<TARGET_FILE:ImGui-SFML::ImGui-SFML>
$<TARGET_FILE:sfml-graphics>
$<TARGET_FILE:sfml-window>
$<TARGET_FILE:sfml-system>
$<TARGET_FILE_DIR:example_exe>)
endif()
endif()

#### THIS IS FOR GOOGLE TEST
enable_testing()
include(GoogleTest)

add_executable(test_all Tests/main.cpp core.h Triangulation.h Triangulation.cpp Grid.h Grid.cpp)
target_link_libraries(test_all PRIVATE gtest)

gtest_discover_tests(test_all)

