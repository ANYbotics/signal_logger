# name of the log element (including all namespaces)
string name
# indicator if var is logged
bool is_logged
# divider of the update frequency
int32 divider
# buffer size
int32 buffer_size
# is buffer full?
bool is_buffer_full
# is buffer looping ?
bool is_buffer_looping
# Desired action
int8 SAVE_AND_PUBLISH_VAR=0
int8 SAVE_VAR=1
int8 PUBLISH_VAR=2
int8 action