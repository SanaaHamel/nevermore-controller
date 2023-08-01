if(NOT DEFINED EXTRA_MODULES_DIR)
  message(FATAL_ERROR "`EXTRA_MODULES_DIR` is not defined")
endif()

set(TARGETS "")

foreach(_arg RANGE ${CMAKE_ARGC})
  if(DEFINED FOUND_BEGINNING_OF_LIST)
    if(NOT "${CMAKE_ARGV${_arg}}" STREQUAL "")
      list(APPEND TARGETS "${CMAKE_ARGV${_arg}}")
    endif()
  elseif("${CMAKE_ARGV${_arg}}" STREQUAL "--")
    set(FOUND_BEGINNING_OF_LIST "")
  endif()
endforeach()

list(REMOVE_DUPLICATES TARGETS)

# bind info to vars
include(${EXTRA_MODULES_DIR}/GetGitRevisionDescription.cmake)
string(TIMESTAMP BUILD_TIMESTAMP "%Y-%m-%d %H:%M:%S UTC" UTC)
git_describe_working_tree(GIT_DESCRIPTION --always)

foreach(TARGET ${TARGETS})
  cmake_path(REMOVE_EXTENSION TARGET LAST_ONLY OUTPUT_VARIABLE DEST)
  configure_file("${TARGET}" "${DEST}" @ONLY)
endforeach()
