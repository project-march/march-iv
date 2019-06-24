# The error code
int16 error_code

# Readable message that describes the error
string error_message

# The type of the error, the type is used to determine how to handle the error
uint8 type
uint8 FATAL             = 0     # The error indicates a serious malfunctioning of the system this is unrecoverable
uint8 NON_FATAL         = 1
# More error types should be added
