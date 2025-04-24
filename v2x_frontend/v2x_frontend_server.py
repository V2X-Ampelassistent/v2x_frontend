import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

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
        self.socketio.on_event('gps_data', self.handle_gps_data)
        
        # Start ROS 2 subscription
        self.GPS_subscription = self.create_subscription(
            NavSatFix,
            'Cohda_Signals/GPS',
            self.listener_callback,
            10
        )

        self.MAPEM_subscription = self.create_subscription(
            NavSatFix,
            'Cohda_Signals/MAPEM',
            self.listener_callback,
            10
        )

        self.SPATEM_subscription = self.create_subscription(
            NavSatFix,
            'Cohda_Signals/SPATEM',
            self.listener_callback,
            10
        )
        
        self.get_logger().info("ROS2 node and subscription initialized")

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

    def handle_gps_data(self, data):
        self.get_logger().info(f'Received GPS data from client: {data}')
        # Process the GPS data here if needed

    def listener_callback(self, msg):
        self.get_logger().info(f"Received GPS data from ROS: {msg.latitude}, {msg.longitude}")
        data = {
            "latitude": msg.latitude,
            "longitude": msg.longitude,
            "altitude": msg.altitude
        }
        self.socketio.emit('gps_data', json.dumps(data))
        


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