
set(logplayer_SRC
    #Source
    cbbeacon.cpp
    cbclient.cpp
    cbgrid.cpp
    cbgridhandler.cpp
    cblab.cpp
    cblabhandler.cpp
    cbparameters.cpp
    cbparamhandler.cpp
    cbpoint.cpp
    cbposition.cpp
    cbreceptionhandler.cpp
    cbreceptionist.cpp
    cblogplayer.cpp
    cbrobot.cpp
    cbtarget.cpp
    cbview.cpp
    cbviewhandler.cpp
    cbwall.cpp
    cbloghandler.cpp
    logplayer.cpp
    # Forms
    logplayerGUI.ui
)

add_executable(logplayer ${logplayer_SRC})

target_include_directories(logplayer PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(logplayer Qt5::Widgets Qt5::Network Qt5::Xml)

set_target_properties(logplayer PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
