cmake_minimum_required(VERSION 2.8.12)
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 OLD)
    cmake_policy(SET CMP0005 NEW)
    cmake_policy(SET CMP0017 NEW)
endif(COMMAND cmake_policy)

PROJECT(locomanipulation)

# ---------------------
# Compile only_feet
# ---------------------
set(LOCOMAN_only_feet ON CACHE BOOL "Flag to compile only_feet")
IF(LOCOMAN_only_feet)
	add_subdirectory(only_feet)
ENDIF(LOCOMAN_only_feet)

# ---------------------
# Compile only_feet
# ---------------------
set(LOCOMAN_feet_r_hand ON CACHE BOOL "Flag to compile feet_r_hand")
IF(LOCOMAN_feet_r_hand)
	add_subdirectory(feet_r_hand)
ENDIF(LOCOMAN_feet_r_hand)

# ---------------------
# Compile only_feet
# ---------------------
set(LOCOMAN_feet_hands ON CACHE BOOL "Flag to compile feet_hands")
IF(LOCOMAN_feet_hands)
	add_subdirectory(feet_hands)
ENDIF(LOCOMAN_feet_hands)



