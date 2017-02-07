# Find the TI PRU Code Generation Tools.
# This will define the following:
# TICGTPRU_FOUND        If all the necessary headers, binaries, and
#                       libs are found.
# TICGTPRU_ROOT         Root directory of the TI CGT Installation.
# TICGTPRU_LIBRARIES    Directory path to the libs.
# TICGTPRU_INCLUDE_DIRS Directory path to the headers.
# TICGTPRU_BIN_DIR      Directory path to the binaries.
# TICGTPRU_CLPRU        Full path to the 'clpru' compiler/linker.
# TICGTPRU_HEXPRU       Full path to the 'hexpru' converter.
# XXD                   Full path to the 'xxd' utility.

find_package(catkin REQUIRED COMPONENTS
    pru_utils
)

if(EXISTS "/opt/ti/ti-cgt-pru_2.1.4")
    set(TICGTPRU_ROOT "/opt/ti/ti-cgt-pru_2.1.4")
elseif(EXISTS "/usr/share/ti/cgt-pru")
    set(TICGTPRU_ROOT "/usr/share/ti/cgt-pru")
else()
    message(FATAL_ERROR "Could not find root of TI PRU install.")
endif()

set(TICGTPRU_LIBRARIES "${TICGTPRU_ROOT}/lib")
set(TICGTPRU_INCLUDE_DIRS "${TICGTPRU_ROOT}/include")

# Check that the bin directory actually exists.  On the ARM installs,
# the binaries are installed under /usr/bin.
if(EXISTS "${TICGTPRU_ROOT}/bin")
    set(TICGTPRU_BIN_DIR "${TICGTPRU_ROOT}/bin")
else()
    set(TICGTPRU_BIN_DIR "/usr/bin")
endif()

# Now actually try and find the compilers we need.
find_program(TICGTPRU_CLPRU clpru PATHS ${TICGTPRU_BIN_DIR})
if(NOT TICGTPRU_CLPRU)
    message(FATAL_ERROR "clpru was not found but is required!")
endif()

find_program(TICGTPRU_HEXPRU hexpru PATHS ${TICGTPRU_BIN_DIR})
if(NOT TICGTPRU_HEXPRU)
    message(FATAL_ERROR "hexpru was not found but is required!")
endif()

# XXD is required to convert the Text and Data bins to C Headers.
find_program(XXD xxd PATHS "/usr/bin")
if(NOT XXD)
    message(FATAL_ERROR "xxd was not found but is required!")
endif()


# Super hacky.  Make the assumption that the first directory path is the 
# base include directory.
list(GET pru_utils_INCLUDE_DIRS 0 BASE_INCLUDE_DIR)
if(NOT BASE_INCLUDE_DIR)
    message(FATAL_ERROR "Hacky - Could not determine the base include dir!")
else()
    set(PRU_ICSS_INCLUDE_DIR "${BASE_INCLUDE_DIR}/pru_utils/pru_icss")
endif()

SET(TICGTPRU_FOUND TRUE)

# Macro to compile PRU C source.
macro(TICGTPRU_generate_headers_macro SOURCE_FILE)
    get_filename_component(BASE_NAME "${SOURCE_FILE}" NAME_WE)

    add_custom_target(TICGTPRU_generate_headers ALL
        # Compile.
        COMMAND ${TICGTPRU_CLPRU} -v3 -Ooff -I=${CMAKE_CURRENT_SOURCE_DIR}/include/
            -I=${PRU_ICSS_INCLUDE_DIR} -I=${TICGTPRU_INCLUDE_DIRS}
            --define=am3359 --define=pru0 --diag_warning=225 --diag_wrap=off --display_error_number
            --hardware_mac=on --endian=little --preproc_with_compile
            --preproc_dependency=${BASE_NAME}.d ${CMAKE_CURRENT_SOURCE_DIR}/${SOURCE_FILE}

        # Link.
        COMMAND ${TICGTPRU_CLPRU} -v3 -Ooff --define=am3359 --define=pru0 --diag_warning=225 --diag_wrap=off
            --display_error_number --hardware_mac=on --endian=little -z -mmain.map --heap_size=0x100
            --stack_size=0x100 -i${TICGTPRU_LIBRARIES} -i${TICGTPRU_INCLUDE_DIRS} --reread_libs
            --display_error_number --warn_sections --diag_wrap=off --xml_link_info=${BASE_NAME}.xml
            --rom_model -o ${BASE_NAME}.out ${BASE_NAME}.obj
            ${PRU_ICSS_INCLUDE_DIR}/AM335x_PRU.cmd -llibc.a

        # Make target directory for generated headers.
        COMMAND mkdir -p ${CMAKE_BINARY_DIR}/generated_headers

        # Convert OBJ to Text and Data binaries.
        COMMAND ${TICGTPRU_HEXPRU} ${PRU_ICSS_INCLUDE_DIR}/AM335x_hexpru.cmd ${BASE_NAME}.out
                -o ${BASE_NAME}_text.bin
                -o ${BASE_NAME}_data.bin

        # Convert Text and Data binaries to C headers.
        COMMAND ${XXD} -i ${BASE_NAME}_text.bin >
            ${CMAKE_BINARY_DIR}/generated_headers/${BASE_NAME}_text.h
        COMMAND ${XXD} -i ${BASE_NAME}_data.bin >
            ${CMAKE_BINARY_DIR}/generated_headers/${BASE_NAME}_data.h

        COMMENT "Generating PRU headers."
        VERBATIM
    )
endmacro()

