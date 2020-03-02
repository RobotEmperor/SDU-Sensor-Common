CMAKE_MINIMUM_REQUIRED ( VERSION 2.8 )
 
PROJECT ( "SDU-Sensor-Common" )
SET ( PROJECT_VERSION_MAJOR 0 )
SET ( PROJECT_VERSION_MINOR 1 )

SET ( CMAKE_C_COMPILER "gcc" )

#INCLUDE_DIRECTORIES (${CMAKE_SOURCE_DIR}/include)
 
add_compile_options(-std=c++11)
add_subdirectory(pybind11)
 
find_package(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES (${EIGEN3_INCLUDE_DIR})

set(ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:${CATKIN_DEVEL_PREFIX}/lib/pkgconfig")
find_package(PkgConfig)
pkg_check_modules(yaml_cpp yaml-cpp REQUIRED)

set(LIB_HEADER_FILES
   ${CMAKE_SOURCE_DIR}/sensor_filter/include/sensor_filter/sensor_filter.h
   ${CMAKE_SOURCE_DIR}/sdu_sensor/include/ft_filter/ft_filter.h
   ${CMAKE_SOURCE_DIR}/sdu_sensor/include/tool_estimation/tool_estimation.h
)

INCLUDE_DIRECTORIES (${CMAKE_SOURCE_DIR}/sensor_filter/include
                     ${CMAKE_SOURCE_DIR}/sdu_sensor/include)

INCLUDE_DIRECTORIES (${yaml_cpp_INCLUDE_DIRS})

set( sdu_sensor_SOURCE_FILES 
  sensor_filter/src/sensor_filter.cpp
  sdu_sensor/src/ft_filter.cpp
  sdu_sensor/src/tool_estimation.cpp
)
link_directories(/usr/local/lib)
add_library(sdu_sensor SHARED ${sdu_sensor_SOURCE_FILES})
set_target_properties(sdu_sensor PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(sdu_sensor sdu_math ${yaml_cpp_LIBRARIES} )

# Generate python bindings using pybind11
pybind11_add_module(ft_filter sdu_sensor/src/ft_filter_py.cpp)
target_link_libraries(ft_filter PRIVATE sdu_sensor sdu_math ${yaml_cpp_LIBRARIES})
target_include_directories(ft_filter PRIVATE ${EIGEN3_INCLUDE_DIR} ${yaml_cpp_INCLUDE_DIRS})

if (CMAKE_COMPILER_IS_GNUCXX)
		# Give error in Linux if library is not fully linked (like Windows)
		set_target_properties(sdu_sensor PROPERTIES LINK_FLAGS -Wl,--no-undefined) 
	endif()

	if(NOT DEFINED LIB_INSTALL_DIR)
		set(LIB_INSTALL_DIR "lib")
	endif()
	if(NOT DEFINED BIN_INSTALL_DIR)
		set(BIN_INSTALL_DIR "bin")
	endif()
	if(NOT DEFINED INCLUDE_INSTALL_DIR)
		set(INCLUDE_INSTALL_DIR "include")
	else()
		set(INCLUDE_INSTALL_DIR "${INCLUDE_INSTALL_DIR}")
	endif()

	install(TARGETS sdu_sensor
			LIBRARY DESTINATION ${LIB_INSTALL_DIR}
			ARCHIVE DESTINATION ${LIB_INSTALL_DIR}
			RUNTIME DESTINATION ${BIN_INSTALL_DIR}
			INCLUDES DESTINATION ${INCLUDE_INSTALL_DIR}
			)

        install(
			FILES
			${LIB_HEADER_FILES}
			DESTINATION ${INCLUDE_INSTALL_DIR}/sdu_sensor
			COMPONENT sdu_sensor
			)

# Install python bindings to site-packages
execute_process(COMMAND "${PYTHON_EXECUTABLE}" "-c"
		"from distutils import sysconfig as s;import sys;import struct;
print('.'.join(str(v) for v in sys.version_info));
print(sys.prefix);
print(s.get_python_inc(plat_specific=True));
print(s.get_python_lib(plat_specific=True));
print(s.get_config_var('SO'));
print(hasattr(sys, 'gettotalrefcount')+0);
print(struct.calcsize('@P'));
print(s.get_config_var('LDVERSION') or s.get_config_var('VERSION'));
print(s.get_config_var('LIBDIR') or '');
print(s.get_config_var('MULTIARCH') or '');
"
		RESULT_VARIABLE _PYTHON_SUCCESS
		OUTPUT_VARIABLE _PYTHON_VALUES
		ERROR_VARIABLE _PYTHON_ERROR_VALUE)

# Convert the process output into a list
if(WIN32)
	string(REGEX REPLACE "\\\\" "/" _PYTHON_VALUES ${_PYTHON_VALUES})
endif()
string(REGEX REPLACE ";" "\\\\;" _PYTHON_VALUES ${_PYTHON_VALUES})
string(REGEX REPLACE "\n" ";" _PYTHON_VALUES ${_PYTHON_VALUES})
list(GET _PYTHON_VALUES 0 _PYTHON_VERSION_LIST)
list(GET _PYTHON_VALUES 1 PYTHON_PREFIX)
list(GET _PYTHON_VALUES 2 PYTHON_INCLUDE_DIR)
list(GET _PYTHON_VALUES 3 PYTHON_SITE_PACKAGES)
list(GET _PYTHON_VALUES 4 PYTHON_MODULE_EXTENSION)
list(GET _PYTHON_VALUES 5 PYTHON_IS_DEBUG)
list(GET _PYTHON_VALUES 6 PYTHON_SIZEOF_VOID_P)
list(GET _PYTHON_VALUES 7 PYTHON_LIBRARY_SUFFIX)
list(GET _PYTHON_VALUES 8 PYTHON_LIBDIR)
list(GET _PYTHON_VALUES 9 PYTHON_MULTIARCH)

# The built-in FindPython didn't always give the version numbers
string(REGEX REPLACE "\\." ";" _PYTHON_VERSION_LIST ${_PYTHON_VERSION_LIST})
list(GET _PYTHON_VERSION_LIST 0 PYTHON_VERSION_MAJOR)
list(GET _PYTHON_VERSION_LIST 1 PYTHON_VERSION_MINOR)
list(GET _PYTHON_VERSION_LIST 2 PYTHON_VERSION_PATCH)

# Make sure all directory separators are '/'
string(REGEX REPLACE "\\\\" "/" PYTHON_PREFIX ${PYTHON_PREFIX})
string(REGEX REPLACE "\\\\" "/" PYTHON_INCLUDE_DIR ${PYTHON_INCLUDE_DIR})
string(REGEX REPLACE "\\\\" "/" PYTHON_SITE_PACKAGES ${PYTHON_SITE_PACKAGES})

message("-- Python bindings install path is: " ${PYTHON_SITE_PACKAGES})
install(TARGETS ft_filter DESTINATION ${PYTHON_SITE_PACKAGES})
