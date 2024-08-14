
set(robsample_SRC
    #Source
    mainRob.c
    robfunc.c
)

add_executable(robsample ${robsample_SRC})

target_include_directories(robsample PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(robsample RobSock)

set_target_properties(robsample PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
