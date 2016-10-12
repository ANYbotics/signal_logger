# Default kindr is on
option(SILO_USE_KINDR "Use kindr in signal_logger?" ON)
if (SILO_USE_KINDR)
  MESSAGE(STATUS "Signal logger is using kindr!")
	add_definitions(-DSILO_USE_KINDR)
else(SILO_USE_KINDR)
  MESSAGE(STATUS "Signal logger is not using kindr!")
endif(SILO_USE_KINDR)
