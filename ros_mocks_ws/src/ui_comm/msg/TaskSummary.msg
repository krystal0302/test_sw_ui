string robot_id
string task_id
string task_name
uint32 state
uint32 STATE_QUEUED=0
uint32 STATE_ACTIVE=1
uint32 STATE_COMPLETED=2
uint32 STATE_FAILED=3

uint32 priority
uint32 PRIORITY_LOW=0
uint32 PRIORITY_NORMAL=1
uint32 PRIORITY_HIGH=2

float32 complete_percent
string status # a brief summary of the current status of the task, for UI's
string role

builtin_interfaces/Time submission_time # submission_time is when the task was submitted to FLM
builtin_interfaces/Time start_time # when FLM actually began processing the task
builtin_interfaces/Time end_time 
# When this message is a summary of an in-process task, the end_time field is an estimate. 
# When this message is a summary of a completed or failed task, end_time is the actual time.
