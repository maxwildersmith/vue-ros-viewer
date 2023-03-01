#!/usr/bin/env python3
from flask import Flask, request
from flask_cors import CORS, cross_origin
import rospy
import json
from geometry_msgs.msg import Pose
from waypoints.msg import Waypoint, WaypointArray
import threading

HOST = "0.0.0.0"
PORT = 6565
app = Flask('connector')
CORS(app)

position = [47.362947,8.541062,0,0,0,0,1]
waypoint_set = False

def updatePosition(data):
	global position
	print(f"got {data}")
	position = [data.position.x, data.position.y, data.position.z, data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]

def json2Way(obj):
	way = Waypoint()
	way.lat = obj['lat']
	way.lng = obj['lng']
	way.min_depth = obj['depth_min']
	way.max_depth = obj['depth_max']
	return way

threading.Thread(target=lambda: rospy.init_node('waypoint_connector', disable_signals=True)).start()
rospy.Subscriber('/position', Pose, updatePosition)
pub = rospy.Publisher('/way', WaypointArray, queue_size=10)

@app.route('/waypoints', methods=['POST'])
@cross_origin()
def setWaypoints():
	global waypoint_set
	data = json.loads(request.form['waypoints'])
	wayArray = WaypointArray()
	wayArray.waypoints = [json2Way(way) for way in data]
	waypoint_set = True
	pub.publish(wayArray)
	return 'Success', 200

@app.route('/status', methods=['GET'])
def getPosition():
	status = {'code': 2 if waypoint_set else 1, 'pos': position}
	print('sending ',status)
	return json.dumps(status)

if __name__ =='__main__':
	try:
		app.run(host=HOST)
	except rospy.ROSInterruptException:
		pass
