
set(GUISample_SRC
    #Source
    main.cpp
    lmap.cpp
    robview.cpp
    sampapp.cpp
)

add_executable(GUISample ${GUISample_SRC})

target_include_directories(GUISample PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(GUISample PRIVATE -DCIBERQTAPP)
target_link_libraries(GUISample Qt5::Widgets Qt5::Xml RobSock)

set_target_properties(GUISample PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
