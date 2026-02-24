if(NOT EXISTS "/home/pherro/PX4-Autopilot/gz_ws/build/asv_sim2/install_manifest.txt")
  message(FATAL_ERROR "Cannot find install manifest: '/home/pherro/PX4-Autopilot/gz_ws/build/asv_sim2/install_manifest.txt'")
endif(NOT EXISTS "/home/pherro/PX4-Autopilot/gz_ws/build/asv_sim2/install_manifest.txt")

file(READ "/home/pherro/PX4-Autopilot/gz_ws/build/asv_sim2/install_manifest.txt" files)
string(REGEX REPLACE "\n" ";" files "${files}")
foreach(file ${files})
  message(STATUS "Uninstalling '$ENV{DESTDIR}${file}'")
  if(IS_SYMLINK "$ENV{DESTDIR}${file}" OR EXISTS "$ENV{DESTDIR}${file}")
    execute_process(COMMAND
      /usr/bin/cmake -E remove $ENV{DESTDIR}${file}
      OUTPUT_VARIABLE rm_out
      RESULT_VARIABLE rm_retval
      )
    if(rm_retfal AND NOT rm_retval STREQUAL 0)
      message(FATAL_ERROR "Problem when removing '$ENV{DESTDIR}${file}'")
    endif()
  else(IS_SYMLINK "$ENV{DESTDIR}${file}" OR EXISTS "$ENV{DESTDIR}${file}")
    message(STATUS "File '$ENV{DESTDIR}${file}' does not exist.")
  endif()
endforeach(file)
