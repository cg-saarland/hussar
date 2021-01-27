add_library(termiWin ${CMAKE_CURRENT_LIST_DIR}/termiWin.c ${CMAKE_CURRENT_LIST_DIR}/termiWin.h)
target_include_directories(termiWin PUBLIC ${CMAKE_CURRENT_LIST_DIR})