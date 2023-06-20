# using std_msg.Float32MultiArray to track the execution time of each part of the algorithm

# spec:
#   |---layout:
#   |     |---dim: 
#   |      |    |---[0]: 
#   |      |    |    |--- label: "time"
#   |      |    |    |--- size:   sec of timestamp
#   |      |    |    |--- stride: usec of timestamp
#   |      |    |
#   |      |    |---[1]~[n]:
#   |      |         |--- label: field name of n parts of the algorithm
#   |      | 
#   |      |---data_offset: number of parts of the algorithm
#   |
#   |---data:
#         |---[0]~[n-1]: time consumption in ms of n parts of the algorithm

        
from std_msgs.msg import Float32MultiArray 