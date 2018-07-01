# These store the latency data for a particular measurement point
# This data is the latency from a previous measurement point as it moves through the system

# this is the node associated with this measurement point
uint8 LOCATOR=10

uint8 node_id

bool send

float64 cur_delay

float64 min_delay

float64 max_delay

float64 avg_delay