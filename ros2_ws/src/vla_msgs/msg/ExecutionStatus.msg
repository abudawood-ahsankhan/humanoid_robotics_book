string plan_id
string action_id
string status  # pending, in_progress, completed, failed
float32 progress  # 0.0 to 1.0
string[] feedback
builtin_interfaces/Time start_time
builtin_interfaces/Time estimated_completion