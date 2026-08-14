# Execution path mode, defined in ExecutionPathType.
int32 LOCAL_PATH = 0
int32 HOMING_PATH = 1
int32 GLOBAL_PATH = 2

# Set one of above values.
int32 mode

# The path is in forward direction compared to current exploration direction or not
bool is_forward