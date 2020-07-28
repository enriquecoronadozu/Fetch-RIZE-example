# This example does not need rosbridge, it directly publish to rize

# Add these lines to your python script 
import nep
import time
# (For ROS 1.0 use)
node = nep.node('python_sender','ROS')

# (For ROS 2.0 use) 
#node = nep.node('python_sender','ROS2')

pub = node.new_pub('/blackboard','json')

# Dummy perceptual function
def isHeadTouched():
	return true
# Publish the current state when 
while True:
	# Here your code that recognize something
	# Example: 
	head_touched = isHeadTouched()
	if head_touched:
		msg = {'primitive':'touched', 'input':{"head":1}, "robot":"ROS"}	
		pub.publish(msg)
		print("send perception ...")
	time.sleep(10)

# In {'primitive':'touched', 'input':{"head":1}, "robot":"ROS"} -- {"head":1} means 100% probability that is touched 


