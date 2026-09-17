cmake_minimum_required(VERSION 3.21)

# ============================================================
# Deployment directory
# ============================================================
file(MAKE_DIRECTORY "${DEPLOY_DIR}")
find_program(OBJDUMP x86_64-w64-mingw32-objdump REQUIRED)
find_program(STRIP x86_64-w64-mingw32-strip REQUIRED)

# ============================================================
# Copy executables
# ============================================================
foreach(APP IN ITEMS "${APP1}" "${APP2}" "${DYN}" "${AUX}")
    if(NOT EXISTS "${APP}")
        message(FATAL_ERROR "Executable not found: ${APP}")
    endif()
    get_filename_component(APP_NAME "${APP}" NAME)
    message(STATUS "Deploying executable: ${APP_NAME}")
    file(COPY "${APP}" DESTINATION "${DEPLOY_DIR}")
    if(STRIP)
        execute_process(COMMAND "${STRIP}" "${DEPLOY_DIR}/${APP_NAME}")
    endif()
endforeach()

# ============================================================
# Find recursive DLL dependencies
# ============================================================
message(STATUS "Runtime DLL search directory:")
message(STATUS "  ${MINGW_BIN_DIR}")
set(RESOLVED_DEPS)
get_filename_component(APP_NAME "${DYN}" NAME)
set(PENDING_FILES "${DEPLOY_DIR}/${APP_NAME}")

while(PENDING_FILES)
    list(POP_FRONT PENDING_FILES CURRENT_FILE)
    list(APPEND RESOLVED_DEPS "${CURRENT_FILE}")
    execute_process(
        COMMAND "${OBJDUMP}" -p "${CURRENT_FILE}"
        OUTPUT_VARIABLE OBJDUMP_OUTPUT
        RESULT_VARIABLE RESULT
    )
    if(NOT RESULT EQUAL 0)
        message(FATAL_ERROR "objdump failed for ${CURRENT_FILE}")
    endif()
    string(REGEX MATCHALL "DLL Name: [^\r\n]+" DLL_LINES "${OBJDUMP_OUTPUT}")
    foreach(LINE IN LISTS DLL_LINES)
        string(REGEX REPLACE
            "^DLL Name:[ \t]*" "" DLL_NAME "${LINE}")
        string(TOLOWER "${DLL_NAME}" DLL_NAME_LOWER)
        # Ignore Windows system DLLs
        if(DLL_NAME_LOWER MATCHES
            "^(kernel32|user32|advapi32|shell32|ole32|oleaut32|ws2_32|gdi32|comdlg32|api-ms-win-|ucrtbase|vcruntime).*\\.dll$"
        )
            continue()
        endif()

        find_file(DLL_PATH NAMES "${DLL_NAME}" PATHS "${MINGW_BIN_DIR}" "${BUILD_DIR}" NO_DEFAULT_PATH)

        if(DLL_PATH)
            list(FIND RESOLVED_DEPS "${DLL_PATH}" DEP_INDEX)
            if(DEP_INDEX EQUAL -1)
                list(APPEND RESOLVED_DEPS "${DLL_PATH}")
                list(APPEND PENDING_FILES "${DLL_PATH}")
            endif()
            unset(DLL_PATH CACHE)
        else()
            message(WARNING "Could not find DLL: ${DLL_NAME}")
        endif()
    endforeach()
endwhile()

list(REMOVE_DUPLICATES RESOLVED_DEPS)

# ============================================================
# Copy detected DLL dependencies
# ============================================================
message(STATUS "")
message(STATUS "Runtime dependencies:")
foreach(DEP IN LISTS RESOLVED_DEPS)
    get_filename_component(
        DEP_NAME "${DEP}" NAME )
    message(STATUS "  Deploying: ${DEP_NAME}")
    file(COPY "${DEP}" DESTINATION "${DEPLOY_DIR}" )
endforeach()

# ============================================================
# Strip binaries to reduce deployment size
# Deployment summary
# ============================================================
set(TRACKED_FILENAMES)
# Collect filenames of resolved dependencies & MinGW runtimes only
foreach(BIN IN LISTS RESOLVED_DEPS MINGW_RUNTIME_DLLS)
    if(EXISTS "${BIN}")
        get_filename_component(FNAME "${BIN}" NAME)
        list(APPEND TRACKED_FILENAMES "${FNAME}")
    endif()
endforeach()
list(REMOVE_DUPLICATES TRACKED_FILENAMES)

# if(STRIP)
#     message(STATUS "")
#     message(STATUS "Stripping debug symbols from binaries...")
# endif()

set(DEPLOY_SIZE_BYTES 0)
foreach(FNAME IN LISTS TRACKED_FILENAMES)
    set(DEPLOYED_BIN "${DEPLOY_DIR}/${FNAME}")

    if(EXISTS "${DEPLOYED_BIN}")
        # Strip if enabled
        # if(STRIP)
        #     execute_process(COMMAND "${STRIP}" "${DEPLOYED_BIN}" OUTPUT_QUIET ERROR_QUIET)
        # endif()
        
        # Calculate size for summary (excluding APP1 and APP2)
        file(SIZE "${DEPLOYED_BIN}" FILE_SIZE)
        math(EXPR DEPLOY_SIZE_BYTES "${DEPLOY_SIZE_BYTES} + ${FILE_SIZE}")
    endif()
endforeach()
# Calculate size with 1 decimal place precision and rounding
math(EXPR DEPLOY_SIZE_TENTHS "(${DEPLOY_SIZE_BYTES} * 10 + 524288) / 1048576")
math(EXPR DEPLOY_SIZE_INT "${DEPLOY_SIZE_TENTHS} / 10")
math(EXPR DEPLOY_SIZE_DEC "${DEPLOY_SIZE_TENTHS} % 10")

message(STATUS "========================================")
message(STATUS "Deployment completed")
message(STATUS "Deployment size: ${DEPLOY_SIZE_INT}.${DEPLOY_SIZE_DEC} MB")
message(STATUS "Directory:")
message(STATUS "  ${DEPLOY_DIR}")
message(STATUS "========================================")