import csv
import rospy
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from std_msgs.msg import Int16
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import pickle

pa = PoseArray()
pa.header.frame_id = "/map"
ma = MarkerArray()

lock = False
loaded = True
load_success = False
n = 0

to_store = 6

def callback_locker(data):
	global lock
	global pa
	global ma
	if lock == False:
		lock = True
		pama = []
		pama.append(pa)
		pama.append(ma)
		with open('poster.objects', 'wb') as storage:
			pickle.dump(pama, storage)		

def callback(data):

	global lock
	global pa
	global ma
	global to_store
	global load_success
	global n
	global file_name_from_button	

	if lock == False:
		print('testing')
		pa.header.stamp = rospy.Time(0)
		pa.header.seq = n
		n = n + 1		
		#pose_in = Pose()
		#o = data.pose.orientation
		#(o1x, o1y, o1z) = euler_from_quaternion([o.x, o.y, o.z, o.w])
		print('testing')		
		pa.poses.append(data.pose)
		print('testing')
		if len(pa.poses) > to_store:
			pa.poses = pa.poses[1:]
		global publisher
		publisher.publish(pa)

		animals = ['Cat', 'Camel', 'Rabbit', 'Donkey', 'Croc', 'Pig', '-']
		marker_in = Marker()
		marker_in.header.frame_id = "/map"
		marker_in.ns = ""
		marker_in.id = 0
		marker_in.type = marker_in.TEXT_VIEW_FACING
		marker_in.action = marker_in.ADD
		marker_in.pose = data.pose
		marker_in.text = ""	
		marker_in.scale.x = 1.0
		marker_in.scale.y = 1.0
		marker_in.scale.z = 0.750
		marker_in.color.r = 1.0
		marker_in.color.g = 1.0
		marker_in.color.b = 1.0
		marker_in.color.a = 1.0	
		ma.markers.append(marker_in)
		if len(ma.markers) > to_store:
			ma.markers = ma.markers[1:]
		for idx in range(len(ma.markers)):
			ma.markers[idx].text = animals[idx]
			ma.markers[idx].id = idx
		global publisher_text	
		publisher_text.publish(ma)
		
		global loaded
		if loaded == True:
			poster_list = []
			poster_list.append(['x', 'y', 'angle'])
			for idx in range(len(pa.poses)):
				pose = pa.poses[idx]
				po = pose.orientation
				(ox, oy, oz) = euler_from_quaternion([po.x, po.y, po.z, po.w])
				poster_list.append([pose.position.x, pose.position.y, oz])
		
			with open("RewardData/"+file_name_from_button, 'w') as rloc:
				print("saving to: " + file_name_from_button)
				wr = csv.writer(rloc, dialect='excel')
				wr.writerows(poster_list)
				rloc.close()
def callback_button(data):

	global file_name_from_button
	file_name_from_button = data.data;	

	with open("RewardData/"+file_name_from_button, 'r') as f:
		reader = csv.reader(f)
		poster_locations = list(reader)
	poster_locations = poster_locations[1:]
	print(poster_locations)
	for i in range(len(poster_locations)):
		print('updated')		
		ps = Pose()
		ps.position.x = float(poster_locations[i][0])
		ps.position.y = float(poster_locations[i][1])
		ps.position.z = 0
		print('updated')
		print(poster_locations[i])
		q = quaternion_from_euler(0, 0, float(poster_locations[i][2]))
		print('updated')		
		ps.orientation.x = q[0]
		ps.orientation.y = q[1]
		ps.orientation.z = q[2]
		ps.orientation.w = q[3]
		print('updated')
		data = PoseStamped()
		data.pose = ps
		callback(data)
		load_success = True


	for i in range(3):
		publisher_text.publish(ma)
		publisher.publish(pa)
		print(i)
		rospy.sleep(1)

global file_name_from_button
file_name_from_button = "" 

publisher = rospy.Publisher('poster_comb', PoseArray, queue_size=1)
publisher_text = rospy.Publisher('poster_comb_text', MarkerArray, queue_size=1)
rospy.Subscriber('poster', PoseStamped, callback)
rospy.Subscriber('trigger_msgs', Int16, callback_locker)
rospy.Subscriber('file_name', String, callback_button)
rospy.init_node('curr_posters', anonymous=True)




rospy.spin()


