# - Find Nite
# 
# If the NITE_INCLUDE is defined,  will be used as search path.
# The following standard variables get defined:
#  Nite_FOUND:    true if found
#  Nite_INCLUDE_DIRS: the directory that contains the include file
#  Nite_LIBRARY_DIR: the directory that contains the library

IF(PKG_CONFIG_FOUND)
  PKG_CHECK_MODULES(NiTE2)
ENDIF()

FIND_PATH(NITE2_INCLUDE_DIRS
  NAMES NiTE.h
  PATHS
    "/opt/include"
    "/opt/local/include"
    "/usr/include"
    "/usr/local/include"
    ENV NITE2_INCLUDE64
    ENV PROGRAMFILES
    ENV ProgramW6432
  HINTS ${NITE2_INCLUDE_DIRS}
  PATH_SUFFIXES
    ni2
    nite2
    NiTE2/Include
)

FIND_LIBRARY(NITE2_LIBRARY
  NAMES NiTE2 ${NITE2_LIBRARIES}
  PATHS
   "/opt/lib"
    "/opt/local/lib"
    "/usr/lib"
    "/usr/local/lib"
    ENV NITE2_LIB64
    ENV PROGRAMFILES
    ENV ProgramW6432
  HINTS ${NITE2_LIBRARY_DIRS}
)


#GET_FILENAME_COMPONENT(NITE2_LIBRARY_DIR ${NITE2_LIBRARY} DIRECTORY)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(Nite2 FOUND_VAR Nite2_FOUND
  REQUIRED_VARS NITE2_INCLUDE_DIRS NITE2_LIBRARY)
