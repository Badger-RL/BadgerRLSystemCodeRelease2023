set(ASMJIT_ROOT_DIR "${BHUMAN_PREFIX}/Util/CompiledNN/3rdParty/asmjit/src")

file(GLOB_RECURSE ASMJIT_SOURCES CONFIGURE_DEPENDS
    "${ASMJIT_ROOT_DIR}/*.cpp" "${ASMJIT_ROOT_DIR}/*.h")

add_library(asmjit${TARGET_SUFFIX} STATIC ${ASMJIT_SOURCES})
set_property(TARGET asmjit${TARGET_SUFFIX} PROPERTY FOLDER "Libs/${TARGET_SUFFIX}")
if(BUILD_DESKTOP)
  set_property(TARGET asmjit${TARGET_SUFFIX} PROPERTY POSITION_INDEPENDENT_CODE ON)
  target_link_libraries(asmjit${TARGET_SUFFIX} PRIVATE $<$<PLATFORM_ID:Linux>:-lrt>)
  set_property(TARGET asmjit${TARGET_SUFFIX} PROPERTY XCODE_ATTRIBUTE_OTHER_LIBTOOLFLAGS -no_warning_for_no_symbols)
else()
  target_link_libraries(asmjit${TARGET_SUFFIX} PRIVATE -lrt-2.31)
endif()
target_include_directories(asmjit${TARGET_SUFFIX} SYSTEM PUBLIC "${ASMJIT_ROOT_DIR}")
target_compile_definitions(asmjit${TARGET_SUFFIX} PUBLIC ASMJIT_STATIC ASMJIT_NO_BUILDER ASMJIT_NO_COMPILER ASMJIT_NO_DEPRECATED ASMJIT_NO_INTROSPECTION ASMJIT_NO_LOGGING ASMJIT_NO_TEXT ASMJIT_NO_VALIDATION)
target_compile_options(asmjit${TARGET_SUFFIX} PRIVATE $<$<CXX_COMPILER_ID:Clang>:-Wno-bitwise-instead-of-logical>)

target_link_libraries(asmjit${TARGET_SUFFIX} PRIVATE Flags::Default)

source_group(TREE "${ASMJIT_ROOT_DIR}/asmjit" FILES ${ASMJIT_SOURCES})
