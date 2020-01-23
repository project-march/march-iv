Header header

# The type of instruction.
int8 type

# All possible instruction types.
int8 STOP = 0
int8 GAIT = 1
int8 PAUSE = 2
int8 CONTINUE = 3
int8 INCREMENT_STEP_SIZE = 4
int8 DECREMENT_STEP_SIZE = 5

# If the type is a GAIT this is the name of the gait.
string gait_name

