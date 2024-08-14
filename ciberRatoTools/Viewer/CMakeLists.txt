
set(viewer_SRC
    #Source
    crmainparameters.cpp
    crqparamhandler.cpp
    main.cpp
    Lab/crbeacon.cpp
    Lab/crgrid.cpp
    Lab/crlab.cpp
    Lab/crrobot.cpp
    Lab/crtarget.cpp
    Lab/crvertice.cpp
    Lab/crwall.cpp
    Comm/crqcomm.cpp
    Comm/crqcommhandler.cpp
    Comm/crqreplyhandler.cpp
    crqrobotinfo.cpp
    crqlabview.cpp
    crqdataview.cpp
    crqscene.cpp
    # Forms
    crqrobotinfo.ui
    crqlabview.ui
    crqdataview.ui
    # Resources
    default.qrc
)

add_executable(Viewer ${viewer_SRC})

target_include_directories(Viewer PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(Viewer Qt5::Widgets Qt5::Network Qt5::Xml Qt5::Multimedia)

set_target_properties(Viewer PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
