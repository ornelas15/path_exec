
set(sim_SRC
    # Source
    cbactionhandler.cpp
    cbbeacon.cpp
    cbbutton.cpp
    cbclient.cpp
    cbcontrolpanel.cpp
    cbgraph.cpp
    cbgrid.cpp
    cbgridhandler.cpp
    cblab.cpp
    cblabdialog.cpp
    cblabhandler.cpp
    cbmanagerobots.cpp
    cbmotor.cpp
    cbpanel.cpp
    cbparamdialog.cpp
    cbparameters.cpp
    cbparamhandler.cpp
    cbpoint.cpp
    cbposition.cpp
    cbreceptionhandler.cpp
    cbreceptionist.cpp
    cbrobotaction.cpp
    cbrobotbeacon.cpp
    cbrobot.cpp
    cbrobotinfo.cpp
    cbsensor.cpp
    cbsimulator.cpp
    cbsimulatorGUI.cpp
    cbtarget.cpp
    cbutils.cpp
    cbview.cpp
    cbviewhandler.cpp
    cbwall.cpp
    simulator.cpp
    # Forms
    cbcontrolpanel.ui
    cblabdialog.ui
    cbmanagerobots.ui
    cbparamdialogbase.ui
    cbrobotinfo.ui
    cbsimulatorGUI.ui
    # Resources
    default.qrc
)

add_executable(simulator ${sim_SRC})

target_include_directories(simulator PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(simulator Qt5::Widgets Qt5::Network Qt5::Xml)

set_target_properties(simulator PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
