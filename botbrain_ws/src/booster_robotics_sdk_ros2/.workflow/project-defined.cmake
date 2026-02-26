macro(closure_import_install_project)
    if (DEFINED ARGV0 AND NOT "${ARGV0}" STREQUAL "")
        cmake_language(CALL ${ARGV0})
    endif ()
endmacro()


macro(closure_link_target_exe i_exe_target)
endmacro()


macro(closure_link_target_lib i_lib_target)
endmacro()


macro(closure_generate_project_name)
    get_filename_component(PLUGIN_TARGET_NAME "${CMAKE_CURRENT_SOURCE_DIR}" NAME)
    string(REPLACE "-" "_" C_MAKE_CUSTOM_SUB_LINE_TARGET_NAME ${PLUGIN_TARGET_NAME})
    string(REPLACE "_" "-" C_MAKE_CUSTOM_MID_LINE_TARGET_NAME ${PLUGIN_TARGET_NAME})
    set(C_MAKE_CUSTOM_PROJECT_NAME "project_${C_MAKE_CUSTOM_SUB_LINE_TARGET_NAME}")
endmacro()


macro(closure_import_prepare_project)
endmacro()

closure_generate_project_name()
closure_import_prepare_project()
