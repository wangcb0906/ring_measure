cmake_minimum_required(VERSION 3.21)

# ============================================================
# Deployment directory
# ============================================================
file(MAKE_DIRECTORY "${DEPLOY_DIR}")


# ============================================================
# Copy executables
# ============================================================
foreach(APP IN ITEMS "${APP1}" "${APP2}")
    if(NOT EXISTS "${APP}")
        message(FATAL_ERROR "Executable not found: ${APP}")
    endif()
    get_filename_component(APP_NAME "${APP}" NAME)
    message(STATUS "Deploying executable: ${APP_NAME}")
    file(COPY "${APP}" DESTINATION "${DEPLOY_DIR}")
endforeach()

# ============================================================
# Find recursive DLL dependencies
# ============================================================
message(STATUS "Runtime DLL search directory:")
message(STATUS "  ${MINGW_BIN_DIR}")
find_program(OBJDUMP x86_64-w64-mingw32-objdump REQUIRED)
set(RESOLVED_DEPS)
set(PENDING_FILES "${APP1};${APP2}")

while(PENDING_FILES)
    list(POP_FRONT PENDING_FILES CURRENT_FILE)
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

        # Ignore Windows system DLLs
        if(DLL_NAME MATCHES
            "^(KERNEL32|USER32|ADVAPI32|SHELL32|OLE32|OLEAUT32|WS2_32|GDI32|COMDLG32|api-ms-win-|ucrtbase|vcruntime).*\\.dll$"
        )
            continue()
        endif()

        find_file(DLL_PATH NAMES "${DLL_NAME}" PATHS "${MINGW_BIN_DIR}" NO_DEFAULT_PATH)

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
# MinGW runtime DLLs
# ============================================================
if(MINGW)
    set(MINGW_RUNTIME_DLL_NAMES
        libstdc++-6.dll libgcc_s_seh-1.dll libwinpthread-1.dll)

    set(MINGW_RUNTIME_DLLS)
    foreach(DLL_NAME IN LISTS MINGW_RUNTIME_DLL_NAMES)
        find_file(
            MINGW_RUNTIME_DLL NAMES "${DLL_NAME}"
            PATHS ${MINGW_BIN_DIR} NO_DEFAULT_PATH
        )
        if(NOT MINGW_RUNTIME_DLL)
            message(FATAL_ERROR
                "Could not find required MinGW runtime DLL: " "${DLL_NAME}"
            )
        endif()
        list(APPEND MINGW_RUNTIME_DLLS "${MINGW_RUNTIME_DLL}")
    endforeach()

    message(STATUS "")
    message(STATUS "MinGW runtime dependencies:")

    foreach(DLL IN LISTS MINGW_RUNTIME_DLLS)
        get_filename_component( DLL_NAME "${DLL}" NAME)
        message(STATUS "  Deploying: ${DLL_NAME}")
        file(COPY "${DLL}" DESTINATION "${DEPLOY_DIR}")
        unset(MINGW_RUNTIME_DLL CACHE)
    endforeach()

endif()


# ============================================================
# Report unresolved dependencies
# ============================================================
if(UNRESOLVED_DEPS)
    message(WARNING "")
    message(WARNING "The following runtime dependencies could not be resolved:")
    foreach(DEP IN LISTS UNRESOLVED_DEPS)
        message(WARNING "  ${DEP}")
    endforeach()
endif()

# ============================================================
# Deployment summary
# ============================================================
execute_process(
    COMMAND du -sb "${DEPLOY_DIR}"
    OUTPUT_VARIABLE DEPLOY_SIZE
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
string(REGEX MATCH "^[0-9]+" DEPLOY_SIZE_BYTES "${DEPLOY_SIZE}")
math(EXPR DEPLOY_SIZE_MB "${DEPLOY_SIZE_BYTES} / 1024 / 1024" )

message(STATUS "========================================")
message(STATUS "Deployment completed")
message(STATUS "Deployment size: ${DEPLOY_SIZE_MB} MB")
message(STATUS "Directory:")
message(STATUS "  ${DEPLOY_DIR}")
message(STATUS "========================================")