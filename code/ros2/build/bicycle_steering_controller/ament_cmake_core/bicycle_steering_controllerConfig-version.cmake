# generated from ament/cmake/core/templates/nameConfig-version.cmake.in
set(PACKAGE_VERSION "2.37.0")

set(PACKAGE_VERSION_EXACT False)
set(PACKAGE_VERSION_COMPATIBLE False)

if("${PACKAGE_FIND_VERSION}" VERSION_EQUAL "${PACKAGE_VERSION}")
  set(PACKAGE_VERSION_EXACT True)
  set(PACKAGE_VERSION_COMPATIBLE True)
endif()

if("${PACKAGE_FIND_VERSION}" VERSION_LESS "${PACKAGE_VERSION}")
  set(PACKAGE_VERSION_COMPATIBLE True)
endif()
