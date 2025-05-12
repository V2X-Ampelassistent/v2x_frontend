import v2x_cohdainterfaces.msg as v2xmsg
import math


class Intersection:
    def __init__(self, intersection_data: v2xmsg.Intersectiongeometry):
        self.id = intersection_data.id.id.intersectionid

        self.refPoint = {
            "lon": intersection_data.refpoint.lon.longitude,
            "lat": intersection_data.refpoint.lat.latitude,
            "elevation": intersection_data.refpoint.elevation.elevation,
        }
        
        self.lanes = {lane.laneid.laneid: Lane(lane, self.refPoint) for lane in intersection_data.laneset.lanelist}

        self.signalGroups = {}

    def add_spat(self, data: dict):
        """Add SPaT data to the Intersection"""
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
        intersection_export["lanes"] = {
            "path": [lane.get_path() for lane in self.lanes.values()]
        }
        return intersection_export


class Lane:
    def __init__(self, lane_data: v2xmsg.Genericlane, refPoint: dict):
        self.LaneID = lane_data.laneid.laneid

        self.ingressApproach = lane_data.ingressapproach.approachid
        self.directionalUse_egressPath = lane_data.laneattributes.directionaluse.egresspath
        self.directionalUse_ingressPath = lane_data.laneattributes.directionaluse.ingresspath
        self.maneuvers: v2xmsg.Allowedmaneuvers = lane_data.maneuvers

        self.laneType: v2xmsg.Laneattributes = lane_data.laneattributes.lanetype
        self.computed_lane: v2xmsg.Computedlane = lane_data.nodelist.computed

        self.refPoint = refPoint

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

    def get_path(self):
        """Get the path of the lane
        Returns a list of tuples (lat, lon) representing the path of the lane.
        """
        
        path = []
        for node in self.nodes:
            node: Node
            if node.lat is not None and node.lon is not None:
                lat = node.lat / 10000000
                lon = node.lon / 10000000
                path.append((lat, lon))
        return path

class Node:
    def __init__(self, node_data: v2xmsg.Nodexy, refPoint: dict):
        delta = node_data.delta
        self.deltaX = None
        self.deltaY = None
        self.lat = None
        self.lon = None

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
            return
        
        if self.deltaX is not None and self.deltaY is not None:
            self.lon = ref_lon + self.deltaLon
            self.lat = ref_lat + self.deltaLat

class ConnectsTo:
    def __init__(self, connection_data: v2xmsg.Connection):
        self.connectingLaneID = connection_data.connectinglane.lane.laneid
        self.maneuver: v2xmsg.Allowedmaneuvers = connection_data.connectinglane.maneuver
        self.connectionID = connection_data.connectionid.laneconnectionid
        self.signalGroup = connection_data.signalgroup.signalgroupid

        self.state = None

    def update_state(self, state):
        self.state = state
