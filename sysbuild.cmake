if(SB_CONFIG_BUILD_NATIVE_SIM AND NOT TARGET native_sim)
  ExternalZephyrProject_Add(
    APPLICATION native_sim
    SOURCE_DIR  ${APP_DIR}
    BOARD       native_sim/native/64
    BUILD_ONLY  TRUE
  )
endif()

if(SB_CONFIG_BUILD_PIC_TEST AND NOT TARGET pic_test)
  ExternalZephyrProject_Add(
    APPLICATION pic_test
    SOURCE_DIR  ${APP_DIR}/tests/pic_test
    BOARD       genie_idcm_ram/88mw320/cpu0
    BUILD_ONLY  TRUE
  )
  if(TARGET sesame)
    add_dependencies(pic_test sesame)
  endif()
endif()
