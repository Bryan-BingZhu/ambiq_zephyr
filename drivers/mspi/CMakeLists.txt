
if (CONFIG_CUSTOM_MSPI_DRIVER)
  # Add header file to the set of global include paths.
  zephyr_include_directories(./)
  list(APPEND SYSCALL_INCLUDE_DIRS ${AMBIQ_DIR}/drivers/mspi)
  zephyr_library()
  zephyr_library_sources(
    costom_mspi_ambiq.c
    )
endif()
