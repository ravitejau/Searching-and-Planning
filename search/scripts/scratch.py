	from problem import *
from server import *
import subprocess
import time

ros_core = subprocess.Popen("roscore")
ros_server = subprocess.Popen("rosrun search server.py -d 5 -n 10 -s 10", shell=True)
time.sleep(5)

print("\n Ravi")
print(ros_server)

ros_server.kill()
ros_core.kill()
subprocess.call(["pkill", "-f", "rosrun"])
subprocess.call(["pkill", "roscore"])
