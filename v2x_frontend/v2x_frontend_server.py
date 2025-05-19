import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import v2x_cohdainterfaces.msg as v2xmsg

import v2x_frontend.map_models as models

from flask import Flask, render_template
from flask_socketio import SocketIO

import json
import logging
from threading import Lock


class v2x_frontend_server(Node):
    def __init__(self):
        # Set up logging
        self.logger = logging.getLogger(__name__)
        logging.basicConfig(level=logging.INFO)
                
        # Initialize ROS 2 first, so logger is available
        super().__init__('v2x_frontend_server')
        
        # Initialize Flask app
        self.app = Flask(__name__)
        self.app.static_folder = 'static'
        self.app.template_folder = 'templates'

        self.app.route('/')(self.index)

        # Initialize SocketIO
        self.socketio = SocketIO(self.app, cors_allowed_origins='*', async_mode='threading')

        self.socketio.on_event('connect', self.handle_connect)
        self.socketio.on_event('disconnect', self.handle_disconnect)
        self.socketio.on_event('message', self.handle_message)
        
        # Start ROS 2 subscription
        self.GPS_subscription = self.create_subscription(
            NavSatFix,
            'Cohda_Signals/GPS',
            self.gps_callback,
            10
        )

        self.MAPEM_subscription = self.create_subscription(
            v2xmsg.Mapem,
            'Cohda_Signals/MAPEM',
            self.mapem_callback,
            10
        )

        self.SPATEM_subscription = self.create_subscription(
            v2xmsg.Spatem,
            'Cohda_Signals/SPATEM',
            self.spatem_callback,
            10
        )
        
        self.get_logger().info("ROS2 node and subscription initialized")

        # variables
        self.last_gps = None
        self.Map = dict()


    def get_logger(self):
        return super().get_logger()  # Use ROS2 logger

    def index(self):
        return render_template('index.html')

    def handle_connect(self):
        self.get_logger().info(f'Client connected')
        self.socketio.emit('message', 'Connected to V2X Frontend Server')

    def handle_disconnect(self):
        self.get_logger().info(f'Client disconnected')

    def handle_message(self, data):
        self.get_logger().info(f'Received message: {data}')
        self.socketio.emit('message', f'Server received: {data}')


    # Callbacks
    def gps_callback(self, msg):
        self.get_logger().info(f"Received GPS data from ROS: {msg.latitude}, {msg.longitude}")
        data = {
            "latitude": msg.latitude,
            "longitude": msg.longitude,
        }
        self.last_gps = (msg.latitude, msg.longitude)
        self.socketio.emit('gps_data', json.dumps(data))  

        # Get the closest intersection
        closest_intersection, closest_distance = self.get_closest_intersection(self.last_gps)
        if closest_intersection is None:
            self.get_logger().info("No intersections found")
            return
        
        # emit the closest intersection
        closest_intersection_obj: models.Intersection = self.Map[closest_intersection]
        closest_lane, distance = closest_intersection_obj.get_closest_lane(self.last_gps)
        if closest_lane is not None:
            self.get_logger().info(f"Closest lane in intersection {closest_intersection_obj.id}: {closest_lane}")
            self.socketio.emit('intersection', json.dumps(closest_intersection_obj.export()))
        else:
            self.get_logger().warning(f"No lanes found in intersection {closest_intersection_obj.id}")
            self.socketio.emit('intersection', json.dumps(closest_intersection_obj.export()))

    def mapem_callback(self, msg: v2xmsg.Mapem):
        for intersection in msg.map.intersections.intersectiongeometrylist:
            intersection: v2xmsg.Intersectiongeometry
            intersectionID: str = f"{msg.header.stationid.stationid}_{intersection.id.id.intersectionid}"
            

            if intersectionID in self.Map:
                self.Map[intersectionID].update(intersection)
            else:
                self.Map[intersectionID] = models.Intersection(intersection, intersectionID)
            
            intersection :models.Intersection = self.Map[intersectionID]
            self.socketio.emit('intersection', json.dumps(intersection.export()))            


    def spatem_callback(self, msg: v2xmsg.Spatem):
        for intersection in msg.spat.intersections.intersectionstatelist:
            intersection: v2xmsg.Intersection
            intersectionID: str = f"{msg.header.stationid.stationid}_{intersection.id.id.intersectionid}"

            if intersectionID in self.Map:
                self.Map[intersectionID].update_spat(intersection)
                self.get_logger().info(f"SPaT data for Intersection ID: {intersectionID}")
                self.socketio.emit('intersection', json.dumps(self.Map[intersectionID].export()))
            else:
                self.get_logger().warning(f"SPaT data received for unknown intersection ID: {intersectionID}")


    def get_closest_intersection(self, gps_point: tuple):
        closest_intersection: models.Intersection = None
        closest_distance = float('inf')
        for intersectionID, intersection in self.Map.items():
            intersection: models.Intersection
            distance = intersection.get_distance_to_refpoint(gps_point)
            if distance < closest_distance:
                closest_distance = distance
                closest_intersection = intersectionID

        if closest_intersection is not None:
            self.get_logger().info(f"Closest intersection: {closest_intersection}")
        else:
            self.get_logger().warning("No intersections found")

        return closest_intersection, closest_distance

    def get_closest_lane(self, gps_point: tuple):
        # get the closest intersection
        closest_intersection = None
        closest_distance = float('inf')
        for intersectionID, intersection in self.Map.items():
            intersection: models.Intersection
            distance = intersection.get_distance_to_refpoint(gps_point)
            if distance < closest_distance:
                closest_distance = distance
                closest_intersection = intersectionID
            

        # get the closest lane in the closest intersection
        if closest_intersection is not None:
            closest_intersection_obj : models.Intersection = self.Map[closest_intersection]
            closest_lane = closest_intersection_obj.get_closest_lane(gps_point)
            if closest_lane is not None:
                self.get_logger().info(f"Closest lane in intersection {closest_intersection_obj.id}: {closest_lane.LaneID}")
            else:
                self.get_logger().warning(f"No lanes found in intersection {closest_intersection}")


    def run_flask(self):
        # Start the background task only after Flask is running
        self.socketio.start_background_task(rclpy.spin, self)
        self.socketio.run(self.app, host="0.0.0.0", port=5000, debug=False, use_reloader=False)



def main():
    # Initialize ROS 2
    rclpy.init()

    server = v2x_frontend_server()
    
    # Start Flask server
    server.run_flask()

    # Shutdown the node
    server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()