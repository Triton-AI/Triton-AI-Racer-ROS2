builtin_interfaces/Time stamp
# Interpretation of the flag is package and competition dependent.
uint8 race_flag 0

# The interpretation here are purely recommendations.
uint8 STOP = 0 # Stop the vehicle immidiately and safely wherever it is.
uint8 RED = 1 # Caution; reduced speed; no overtaking; return to pit.
uint8 YELLOW = 2 # Caution; reduced speed; no overtaking.
uint8 GREEN = 3 # Normal race.
uint8 BLACK = 4 # Return to pit.
uint8 CHECKERED = 5 # Race finished.

# These are additional flag recommendations used by the Indy Autonomous Challenge.
uint8 ORANGE = 6 # Start the engine.
uint8 PURPLE = 7 # Shutdown the engine.
