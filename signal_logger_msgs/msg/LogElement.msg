## Contains the properties of a logger element with name
## 'name'. The flag 'is_logged' determines whether the 
## element is currently logged. The collection frequency is
## given by 'divider' -> update_freq = controller_freq/divider.
## The 'action' defines if the data logged should be published,
## saved or both. The buffer size is defined by 'buffer_size'.
## If old elements are overwritten by newer ones in the buffer
## (looping) 'is_buffer_looping' is set to true. Additionally
## an inidicator of the buffer fill state is given by 'is_buffer_full'.

string name
bool is_logged
int32 divider

int8 SAVE_AND_PUBLISH_VAR=0
int8 SAVE_VAR=1
int8 PUBLISH_VAR=2
int8 action

int32 buffer_size
bool is_buffer_full
bool is_buffer_looping