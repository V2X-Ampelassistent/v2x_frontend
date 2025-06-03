import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import v2x_cohdainterfaces.msg as v2xmsg
import v2x_services.srv as v2x_services
import can_bridge.msg as canmsg
import v2x_frontend.map_models as models

from flask import Flask, render_template
from flask_socketio import SocketIO

import json
import logging
from threading import Lock

import math

MAX_INTERSECTION_DISTANCE = 500  # meters

class v2x_frontend_server(Node):
    def __init__(self):
        # Set up logging
        self.logger = logging.getLogger(__name__)
        logging.basicConfig(level=logging.INFO)
                
        # Initialize ROS 2 first, so logger is available
        super().__init__('v2x_frontend_server')
        
        # Intitialize Services
        self.traffic_light_info_service = self.create_service(
            v2x_services.TrafficLightInfo,
            'TrafficLightInfo',
            self.traffic_light_info_callback
        )

        self.warning_info_display = self.create_service(
            v2x_services.WarningInfoDisplay,
            'WarningInfoDisplay',
            self.warning_info_display_callback
        )

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
            v2xmsg.GPS,
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
        
        self.CAN_subscription = self.create_subscription(
            canmsg.EvitoState,
            'EvitoState',
            self.can_callback,
            10
        )

        self.get_logger().info("ROS2 node and subscription initialized")

        # variables
        self.last_gps_point = None
        self.gps_direction = None
        self.Map = dict()
        self.current_intersection_id = None
        self.current_lane_id = None
        self.last_can_msg = None


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
    def gps_callback(self, msg: v2xmsg.GPS):
        self.get_logger().info(f"Received GPS data from ROS: {msg.lat}, {msg.lon}")
        
        # # determine direction of travel
        # if self.last_gps_point is not None:
        #     directory = math.atan2(
        #         msg.lon - self.last_gps_point[1],
        #         msg.lat - self.last_gps_point[0]
        #     )
        #     # filter the direction to prevent noise
        #     if self.gps_direction is None:
        #         self.gps_direction = directory
        #     else:
        #         Filter = 0.2
        #         self.gps_direction = self.gps_direction * (1 - Filter) + directory * Filter

        #     # self.gps_direction = math.degrees(self.gps_direction)
        #     self.get_logger().info(f"Direction of travel: {self.gps_direction}")
        # self.last_gps_point = (lat, lon)

        self.gps_direction = msg.track % 360
        self.get_logger().info(f"Direction of travel: {self.gps_direction}")

        # Emit the GPS data to the client
        data = {
            "latitude": msg.lat,
            "longitude": msg.lon,
            "direction": self.gps_direction,
        }
        self.socketio.emit('gps_data', json.dumps(data))

        # reset the current intersection and lane if GPS data is received
        self.current_intersection_id = None
        self.current_lane_id = None

        # Get the closest Lane and Intersection
        closest_intersection_obj, closest_lane, distance = self.get_closest_lane((msg.lat, msg.lon, self.gps_direction))
        
        if closest_intersection_obj is None:
            self.get_logger().info("No intersection found")
            return
        if closest_lane is None:
            self.get_logger().info(f"No lanes found in intersection {closest_intersection_obj.id}")
            return
        
        # Emit the closest lane and intersection to the client
        self.get_logger().info(f"Closest lane in intersection {closest_intersection_obj.id}: {closest_lane} distance: {distance}")
        self.current_intersection_id = closest_intersection_obj.id
        self.current_lane_id = closest_lane
        # self.socketio.emit('current_lane', json.dumps(
        # {
        #     "intersection_id": closest_intersection_obj.id,
        #     "lane_id": closest_lane,
        #     "distance": distance
        # }
        # ))
        self.socketio.emit('intersection', json.dumps(closest_intersection_obj.export(self.current_lane_id)))

    def mapem_callback(self, msg: v2xmsg.Mapem):
        for intersection in msg.map.intersections.intersectiongeometrylist:
            intersection: v2xmsg.Intersectiongeometry
            intersectionID: str = f"{msg.header.stationid.stationid}_{intersection.id.id.intersectionid}"
            

            if intersectionID in self.Map:
                self.Map[intersectionID].update(intersection)
            else:
                self.Map[intersectionID] = models.Intersection(intersection, intersectionID)
            
            intersection :models.Intersection = self.Map[intersectionID]
            current_lane = None
            if self.current_intersection_id == intersectionID:
                current_lane = self.current_lane_id

            self.socketio.emit('intersection', json.dumps(intersection.export(current_lane)))            


    def spatem_callback(self, msg: v2xmsg.Spatem):
        for intersection in msg.spat.intersections.intersectionstatelist:
            intersection: v2xmsg.Intersection
            intersectionID: str = f"{msg.header.stationid.stationid}_{intersection.id.id.intersectionid}"

            if intersectionID in self.Map:
                self.Map[intersectionID].update_spat(intersection)
                self.get_logger().info(f"SPaT data for Intersection ID: {intersectionID}")

                current_lane = None
                if self.current_intersection_id == intersectionID:
                    current_lane = self.current_lane_id
                self.socketio.emit('intersection', json.dumps(self.Map[intersectionID].export(current_lane)))
            else:
                self.get_logger().warning(f"SPaT data received for unknown intersection ID: {intersectionID}")

    def can_callback(self, msg: canmsg.EvitoState):
        # self.get_logger().info(f"Received CAN data")
        self.last_can_msg = msg
        

    def traffic_light_info_callback(self, request, response):
        self.get_logger().info(f"Received TrafficLightInfo request: {request}")
        # To trigger callback, run '''ros2 service call /TrafficLightInfo v2x_services/srv/TrafficLightInfo'''
        current_intersection: models.Intersection = self.Map.get(self.current_intersection_id, None)
        if current_intersection is None:
            self.get_logger().warning("No current intersection found")
            return response
        movement_phase_state = current_intersection.get_state_for_lane(self.current_lane_id)

        response.movementphasestate = str(movement_phase_state)
        return response

    def warning_info_display_callback(self, request, response):
        self.get_logger().info(f"Received WarningInfoDisplay request: {request}")
        type = request.type
        infomessage = request.infomessage

        if type == 1:  # Warning
            self.socketio.emit('warning', json.dumps({
                "type": "warning",
                "message": infomessage
            }))
        elif type == 0:
            self.socketio.emit('info', json.dumps({
                "type": "info",
                "message": infomessage
            }))

        return response


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
        closest_intersections = list()
        for intersectionID, intersection in self.Map.items():
            intersection: models.Intersection
            distance = intersection.get_distance_to_refpoint(gps_point)
            if distance < MAX_INTERSECTION_DISTANCE:
                closest_intersections.append(intersection)

        if len(closest_intersections) == 0:
            self.get_logger().info("No intersections found within the maximum distance")
            return None, None, None

        min_distance = float('inf')
        closest_lane = None
        closest_intersection = None
        for intersection in closest_intersections:
            intersection: models.Intersection
            lane, distance = intersection.get_closest_lane(gps_point)
            if distance < min_distance:
                min_distance = distance
                closest_intersection = intersection
                closest_lane = lane

        return closest_intersection, closest_lane, min_distance

    def run_flask(self):
        # Start the background task only after Flask is running
        self.socketio.start_background_task(rclpy.spin, self)
        self.socketio.run(self.app, host="0.0.0.0", port=5000, debug=False, use_reloader=False, allow_unsafe_werkzeug=True)



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