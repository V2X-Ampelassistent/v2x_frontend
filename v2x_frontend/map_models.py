import v2x_cohdainterfaces.msg as v2xmsg
import numpy as np
import math

MAX_DETECTION_RADIUS = 5  # meters
DEVIATION = 0.5  # Allowable deviation in radians

class SignalGroup:
    STATE_LOOKUP = {
        "unavailable": "UNAVAILABLE",
        "dark": "DARK",
        "stop-Then-Proceed": "STOP",
        "stop-And-Remain": "RED",
        "pre-Movement": "RED_YELLOW",
        "permissive-Movement-Allowed": "GREEN",
        "protected-Movement-Allowed": "GREEN",
        "permissive-Clearance": "YELLOW",
        "permissive-clearance": "YELLOW",
        "protected-Clearance": "YELLOW",
        "protected-clearance": "YELLOW",
        "caution-Conflicting-Traffic": "YELLOW",
    }

    def __init__(self, signal_group_data: v2xmsg.Signalgroupid):
        self.id = signal_group_data.signalgroupid
        self.state = None

    def update_state(self, state: v2xmsg.Movementphasestate):
        self.state = SignalGroup.STATE_LOOKUP.get(state.movementphasestate, None)
        if self.state is None:
            print(f"Unknown state: {state.movementphasestate}")


class Intersection:
    def __init__(self, intersection_data: v2xmsg.Intersectiongeometry, id: str):
        self.id = id

        self.refPoint = {
            "lon": intersection_data.refpoint.lon.longitude,
            "lat": intersection_data.refpoint.lat.latitude,
            "elevation": intersection_data.refpoint.elevation.elevation,
        }
        
        self.lanes = {lane.laneid.laneid: Lane(lane, self.refPoint) for lane in intersection_data.laneset.lanelist}

        self.signalGroups = {}

    def update_spat(self, data: v2xmsg.Intersectionstate):
        """Add SPaT data to the Intersection"""
        for movement in data.states.movementlist:
            movement: v2xmsg.Movementstate
            signalGroupID = movement.signalgroup.signalgroupid

            movement_event: v2xmsg.Movementphasestate = movement.state_time_speed.movementeventlist[0]
            if signalGroupID not in self.signalGroups:
                self.signalGroups[signalGroupID] = SignalGroup(movement.signalgroup)
            self.signalGroups[signalGroupID].update_state(movement_event.eventstate)


        pass

    def update(self, intersection_data: v2xmsg.Intersectiongeometry):
        """Update the Intersection with new data"""

        self.refPoint = {
            "lon": intersection_data.refpoint.lon.longitude,
            "lat": intersection_data.refpoint.lat.latitude,
            "elevation": intersection_data.refpoint.elevation.elevation,
        }

        for lane in intersection_data.laneset.lanelist:
            lane: v2xmsg.Lane
            lane_id = lane.laneid.laneid
            if lane_id not in self.lanes:
                self.lanes[lane_id] = Lane(lane, self.refPoint)
            else:
                self.lanes[lane_id].update(lane, self.refPoint)

    def export(self) -> dict:
        """Export the intersection as a leaflet object"""
        intersection_export = {
            "id": self.id,
        }
        refPoint_lat_lon = (self.refPoint["lat"] / 10000000, self.refPoint["lon"] / 10000000)
        intersection_export["ref_point"] = refPoint_lat_lon
        intersection_export["lanes"] = [{
            "path": [],
            "connections": []
        }]

        # export connections
        for lane in self.lanes.values():
            lane: Lane
            
            if not lane.laneType.vehicle: 
                continue

            color = "#FF0800"
            if lane.directionalUse_ingressPath:
                color = "#0000FF"
            elif lane.directionalUse_egressPath:
                color = "#FF7F00"

            lane_export = {
                "id": lane.LaneID,
                "path": lane.export_path(),
                "color": color,
                "connections": []
            }
                

            for connect in lane.connectsTo:
                connect: ConnectsTo
                connectingLaneID = connect.connectingLaneID
                if connectingLaneID not in self.lanes:
                    continue

                startpoint = (lane.nodes[0].lat / 10000000, lane.nodes[0].lon / 10000000)
                endpoint = (self.lanes[connectingLaneID].nodes[0].lat / 10000000, self.lanes[connectingLaneID].nodes[0].lon / 10000000)

                state = None
                if connect.signalGroup in self.signalGroups.keys():
                    signalGroup: SignalGroup = self.signalGroups[connect.signalGroup]
                    state = signalGroup.state
                
                lane_export["connections"].append({
                    "startpoint": startpoint,
                    "endpoint": endpoint,
                    "state": state,
                })

            intersection_export["lanes"].append(lane_export)

        return intersection_export

    def get_distance_to_refpoint(self, point: tuple) -> float:
        """Get the distance to the reference point"""
        ref_lat = self.refPoint["lat"] / 10000000
        ref_lon = self.refPoint["lon"] / 10000000

        x_diff = (point[1] - ref_lon) * 111320 * math.cos(math.radians(ref_lat))  # Approx. meters per degree longitude
        y_diff = (point[0] - ref_lat) * 111320  # Approx. meters per degree latitude 

        return math.sqrt(x_diff**2 + y_diff**2)


    def get_closest_lane(self, point: tuple):
        """Get the closest lane to the given point"""
        closest_lane = None
        min_distance = float('inf')
        for lane in self.lanes.values():
            lane: Lane
            if not lane.laneType.vehicle: 
                continue
            
            if lane.egressApproach:
                continue

            distance = lane.get_distance_to_point(point)
            if distance < min_distance:
                min_distance = distance
                closest_lane = lane

        if closest_lane is None:
            return None, min_distance
        
        if min_distance > MAX_DETECTION_RADIUS:
            return None, min_distance
        
        return closest_lane.LaneID, min_distance
    
class Lane:
    def __init__(self, lane_data: v2xmsg.Genericlane, refPoint: dict):
        self.refPoint = refPoint
        self.LaneID = lane_data.laneid.laneid

        self.ingressApproach = lane_data.ingressapproach.approachid
        self.egressApproach = lane_data.egressapproach.approachid

        self.directionalUse_egressPath = lane_data.laneattributes.directionaluse.egresspath
        self.directionalUse_ingressPath = lane_data.laneattributes.directionaluse.ingresspath
        
        self.maneuvers: v2xmsg.Allowedmaneuvers = lane_data.maneuvers
        self.laneType: v2xmsg.Laneattributes = lane_data.laneattributes.lanetype
        self.computed_lane: v2xmsg.Computedlane = lane_data.nodelist.computed
        self.nodes = self.create_node_(lane_data.nodelist.nodes)	
        self.connectsTo = [ConnectsTo(connect) for connect in lane_data.connectsto.connectstolist]

    def update(self, lane_data: v2xmsg.Genericlane, refPoint: dict):
        """Update the lane with new data"""
        self.laneType: v2xmsg.Laneattributes = lane_data.laneattributes.lanetype
        self.maneuvers: v2xmsg.Allowedmaneuvers = lane_data.maneuvers
        self.refPoint = refPoint

        # Update nodes and connectsTo
        self.nodes = self.create_node_(lane_data.nodelist.nodes)
        self.connectsTo = [ConnectsTo(connect) for connect in lane_data.connectsto.connectstolist]

    def create_node_(self, node_data: list) -> list:
        """
        Create a node from the node data
        param node_data: list of v2xmsg.Nodesetxy
        """
        
        computed_nodes = list()
        for node_set_xy in node_data:
            node_set_xy: v2xmsg.Nodesetxy
            node_list = node_set_xy.nodesetxy

            for node in node_list:
                node: v2xmsg.Nodexy
                computed_nodes.append(Node(node, self.refPoint))

        # Set the absolute position of the nodes
        prev_node = None
        for node in computed_nodes:
            node: Node
            if prev_node:
                node.set_absolute_position(prev_node.lon, prev_node.lat)
            else:
                node.set_absolute_position(self.refPoint["lon"], self.refPoint["lat"])
            prev_node = node
        
        return computed_nodes

    def export_path(self):
        """Get the path of the lane
        Returns a list of tuples (lat, lon) representing the path of the lane.
        """
        
        path = []
        for node in self.nodes:
            node: Node
            if node.lat is not None and node.lon is not None:
                lat = node.lat / 10000000
                lon = node.lon / 10000000
                
                direction = node.direction
                if self.directionalUse_egressPath:
                    direction = node.direction + math.pi

                path.append((lat, lon, direction))
        return path
    
    def get_distance_to_point(self, point: tuple) -> float:
        """Get the distance between a lanesegment and a point, return the distance"""
        min_distance = float('inf')
        
        path = self.export_path()
        
        prev_node = None
        for node in path:
            if prev_node is None:
                prev_node = node
                continue
            
            # Check if direction is valid
            if node[2] is not None and point[2] is not None:
                if abs(node[2] - point[2]) > DEVIATION:
                    prev_node = node
                    continue

            # Calculate the distance from the point to the line segment
            distance, is_on_segment = self.point_to_vector_distance(prev_node, node, point)
            if not is_on_segment:
                prev_node = node
                continue  # Skip if the projection is outside the segment
            if distance < min_distance:
                min_distance = distance

            prev_node = node
        return min_distance

    def point_to_vector_distance(self, start, end, point):
        """Calculate the distance from a point to a line segment defined by start and end points.
        Returns the distance and a boolean indicating if the point is on the segment.
        ChatGPT Wrote this, i don't have any idea how it works, but it works.
        """

        start = np.array(start)[:2]
        end = np.array(end)[:2]
        point = np.array(point)[:2]

        vec = (end - start) * 11320  # Approx. meters per degree latitude 
        vec[0] *= math.cos(math.radians(start[0]))  # Approx. meters per degree longitude

        point_vec = (point - start) * 11320  # Approx. meters per degree latitude
        point_vec[0] *= math.cos(math.radians(start[0]))  # Approx. meters per degree longitude

        vec_length_squared = np.dot(vec, vec)
        if vec_length_squared == 0:
            return np.linalg.norm(point - start), False  # Start and end are the same

        # Projection of point_vec onto vec
        t = np.dot(point_vec, vec) / vec_length_squared
        projection = start + t * vec

        # Clamp t to [0, 1] to check if projection lies on the segment
        is_on_segment = 0 <= t <= 1
        distance = np.linalg.norm(point - projection)

        return distance, is_on_segment

class Node:
    def __init__(self, node_data: v2xmsg.Nodexy, refPoint: dict):
        delta = node_data.delta
        self.deltaX = None
        self.deltaY = None
        self.lat = None
        self.lon = None
        self.direction = None # direction always from the refPoint outwards

        if delta.node_latlon:
            self.lat = delta.node_latlon[0].lat.latitude
            self.lon = delta.node_latlon[0].lon.longitude
        elif delta.node_xy1:
            self.deltaX = delta.node_xy1[0].x.offset_b10
            self.deltaY = delta.node_xy1[0].y.offset_b10
        elif delta.node_xy2:
            self.deltaX = delta.node_xy2[0].x.offset_b11
            self.deltaY = delta.node_xy2[0].y.offset_b11
        elif delta.node_xy3:
            self.deltaX = delta.node_xy3[0].x.offset_b12
            self.deltaY = delta.node_xy3[0].y.offset_b12
        elif delta.node_xy4:
            self.deltaX = delta.node_xy4[0].x.offset_b13
            self.deltaY = delta.node_xy4[0].y.offset_b13
        elif delta.node_xy5:
            self.deltaX = delta.node_xy5[0].x.offset_b14
            self.deltaY = delta.node_xy5[0].y.offset_b14
        elif delta.node_xy6:
            self.deltaX = delta.node_xy6[0].x.offset_b16
            self.deltaY = delta.node_xy6[0].y.offset_b16
        else:
            raise ValueError("Node data is not in the expected format")
        

        # Calculate the delta meters in delta degrees
        if self.deltaX is not None and self.deltaY is not None:
            ref_lat = refPoint["lat"] / 10000000  # Convert to degrees
            self.deltaLat = self.deltaY / 1.11320  # Approx. meters per degree latitude
            self.deltaLon = self.deltaX / (1.11320 * math.cos(math.radians(ref_lat)))

    def set_absolute_position(self, ref_lon: float, ref_lat: float):
        if self.lat is not None and self.lon is not None:
            self.direction = math.atan2(ref_lon - self.lon, ref_lat - self.lat)
            return
        
        if self.deltaX is not None and self.deltaY is not None:
            self.lon = ref_lon + self.deltaLon
            self.lat = ref_lat + self.deltaLat
            self.direction = math.atan2(-self.deltaX, -self.deltaY)

class ConnectsTo:   
    def __init__(self, connection_data: v2xmsg.Connection):
        self.connectingLaneID = connection_data.connectinglane.lane.laneid
        self.maneuver: v2xmsg.Allowedmaneuvers = connection_data.connectinglane.maneuver
        self.connectionID = connection_data.connectionid.laneconnectionid
        self.signalGroup = connection_data.signalgroup.signalgroupid
