#
# Message from CarMaker ROS Node to external ROS Node
#

# General
Header  header                                 # General ROS Header (optional)
time    time                                   # ROS time when message was sent (optional)
int64   cycleno                                # Cycle number since simulation start (optional)
float64 synthdelay                             # Synthetic delay to check synchronization 

float64 velocity
float64 heading
float64 pitch
float64 roll
