if(_GLOW_COMPILE_INCLUDED_)
  return()
endif()
# guard again multiple inclusion:
set(_GLOW_COMPILE_INCLUDED_ TRUE)
# get directory of the macros (inside the macro it will return the CMakeLists.txt directory of calling code)
set(GLOW_MODULE_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})


## CompileShaders
## --------------
## Generates a cpp file with initialization code for the cache.
## 
## COMPILE_SHADERS(<filename> <sources> ...)
## 
##   Write source file with given <filename> to initialize shader cache 
##   with given one or multiple <sources>. 
##
##
macro(COMPILE_SHADERS outfile )
  
  # Avoid overwriting non-cache files.
  if(EXISTS ${outfile})
    file(READ ${outfile} HEADER LIMIT 17)
    if(NOT "${HEADER}" STREQUAL "")
    
      string(STRIP ${HEADER} HEADER)
      
      if(NOT "${HEADER}" STREQUAL "// GLOW_CACHE V10")
        message(FATAL_ERROR "Trying to overwrite file '${outfile}', which appears not to be a generated cache file!")
      endif()
    endif()
  endif()
    
  set(absolute_filenames "")
  foreach(file ${ARGN})
    get_filename_component(abs_file ${file} ABSOLUTE)
    if(EXISTS ${abs_file}) 
      set(absolute_filenames ${absolute_filenames} ${abs_file})
    else()
      message(WARNING "Shader with filename '${file}' does not exists!")
    endif()
  endforeach()
  
  #message(STATUS "Running COMPILE_SHADERS ${outfile}")
  #message(STATUS "absolute_filenames = ${absolute_filenames}")
  
  
  set(GLOW_CACHE_CPP_FILENAME ${outfile} CACHE INTERNAL "output filename")
  set(GLOW_CACHE_SHADER_FILENAMES ${absolute_filenames} CACHE INTERNAL "shader filenames")
  
  # Needed to get the command line options:    
  separate_arguments(SHADER_ARG UNIX_COMMAND "${GLOW_CACHE_SHADER_FILENAMES}")  
  
  
  add_custom_command(OUTPUT ${outfile}
    COMMAND ${CMAKE_COMMAND} 
    ARGS -DGLOW_CACHE_CPP_FILENAME:INTERNAL=${GLOW_CACHE_CPP_FILENAME} -DGLOW_CACHE_SHADER_FILENAMES:INTERNAL=${SHADER_ARG} -P ${GLOW_MODULE_DIRECTORY}/GenCppFile.cmake
    DEPENDS ${ARGN} ${GLOW_MODULE_DIRECTORY}/GenCppFile.cmake ${GLOW_MODULE_DIRECTORY}/GlowShaderCompilation.cmake
    VERBATIM) # <-- VERBATIM needed to escape the semicolons.
    
 
    
endmacro()