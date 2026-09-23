if(SB_CONFIG_BUILD_NATIVE_SIM AND NOT TARGET native_sim)
  ExternalZephyrProject_Add(
    APPLICATION native_sim
    SOURCE_DIR  ${APP_DIR}
    BOARD       native_sim/native/64
    BUILD_ONLY  TRUE
  )
endif()
