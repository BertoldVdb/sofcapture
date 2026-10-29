# Writes the git version to OUT. Only on a change, so nothing rebuilds needlessly.
execute_process(COMMAND git describe --always --tags --dirty
    WORKING_DIRECTORY ${SRC}
    OUTPUT_VARIABLE v OUTPUT_STRIP_TRAILING_WHITESPACE)
set(text "#define SOFGNSS_VERSION \"${v}\"\n")
if(EXISTS ${OUT})
  file(READ ${OUT} old)
endif()
if(NOT text STREQUAL "${old}")
  file(WRITE ${OUT} "${text}")
endif()
