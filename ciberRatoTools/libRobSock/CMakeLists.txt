
set(RobSock_SRC
    #Source
    cmeasures.cpp
    croblink.cpp
    csimparam.cpp
    netif.cpp
    RobSock.cpp
    structureparser.cpp
)

add_library(RobSock SHARED ${RobSock_SRC})

target_include_directories(RobSock PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(RobSock PRIVATE -DCIBERQTAPP)
target_link_libraries(RobSock Qt5::Widgets Qt5::Xml)

set_target_properties(RobSock PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
