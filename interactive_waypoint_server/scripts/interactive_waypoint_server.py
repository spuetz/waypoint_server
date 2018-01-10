#!/usr/bin/python2
import rospy
import copy

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from interactive_waypoint_server_msgs.msg import Edge
from interactive_waypoint_server_msgs.srv import RemoveConnection, RemoveConnectionResponse, SaveWaypoints, SaveWaypointsResponse, GetWaypointGraph, GetWaypointGraphResponse
from diagnostic_msgs.msg import KeyValue
from visualization_msgs.msg import *
from geometry_msgs.msg import PointStamped, PoseStamped, Point
from networkx import Graph
from mongodb_store import message_store
import math
import yaml

MENU_CONNECT = 1
MENU_CLEAR = 2
MENU_REMOVE = 3

STATE_REGULAR = 0
STATE_CONNECT = 1
STATE_NONE = 2

def euclidean_distance(point1, point2):
    return math.sqrt((point2.x - point1.x)**2 + (point2.y - point1.y)**2 + (point2.y - point1.y)**2)


class InteractiveWaypointServer(object):
    def __init__(self):
        self.server = InteractiveMarkerServer("interactive_waypoint_server")
        self.waypoint_graph = Graph()
        self.next_waypoint_id = 0
        self.next_edge_id = 0
        self.state = STATE_REGULAR
        self.connect_first_marker = ""
        self.edge_line_publisher = rospy.Publisher("/interactive_waypoint_server/edges", MarkerArray, queue_size=10)

    def insertMarkerCallback(self, pos):
        rospy.logdebug("(Interactive_Waypoint_Server) Inserting new waypoint at position ({0},{1},{2}).".format(pos.point.x, pos.point.y, pos.point.z))
        self.insertMarker(pos.point)

    def clearAllMarkers(self):
        edges = MarkerArray()
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "map"
        marker.ns = "waypoint_edges"
        marker.id = 0
        marker.action = Marker.DELETEALL
        edges.markers.append(marker)
        self.edge_line_publisher.publish(edges)

    def _makeMarker(self, msg):
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = msg.scale * 0.45
        marker.scale.y = msg.scale * 0.45
        marker.scale.z = msg.scale * 0.45
        marker.color.r = 0
        marker.color.g = 1
        marker.color.b = 0
        marker.color.a = 1.0
        return marker

    def _makeEdge(self, scale, begin, end):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "waypoint_edges"
        marker.id = self.next_edge_id
        self.next_edge_id += 1
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = scale * 0.45
        marker.scale.y = scale * 0.45
        marker.scale.z = scale * 0.45
        marker.color.r = 0
        marker.color.g = 0
        marker.color.b = 1
        marker.color.a = 1.0
        marker.points.append(begin)
        marker.points.append(end)
        return marker

    def _removeMarker(self, name):
        self._clearMarker(name)
        self.waypoint_graph.remove_node(name)
        self.server.erase(name)
        self.server.applyChanges()

    def _clearMarker(self, name):
        # remove all edges to a waypoint
        edges = MarkerArray()
        to_remove = []
        for u, v, data in self.waypoint_graph.edges(name, data=True):
            marker = data["marker"]
            marker.action = Marker.DELETE
            to_remove.append((u,v,marker))

        # update knowledge database to remove all edges
        for u,v,marker in to_remove:
            edges.markers.append(marker)
            self.waypoint_graph.remove_edge(u, v)
        self.edge_line_publisher.publish(edges) # publish deletion

    def _connectMarkers(self, first, second):
        if self.waypoint_graph.has_edge(first, second):
            # remove edge
            edges = MarkerArray()
            marker = self.waypoint_graph.get_edge_data(first, second)["marker"]
            marker.action = Marker.DELETE
            edges.markers.append(marker)
            self.waypoint_graph.remove_edge(first, second)
            self.edge_line_publisher.publish(edges)  # publish deletion
        else:
            firstpos = self.server.get(first).pose.position
            secondpos = self.server.get(second).pose.position
            #  insert edge
            marker = self._makeEdge(0.2, firstpos, secondpos)
            # insert edge into graph
            self.waypoint_graph.add_edge(first, second, {"first": first, "marker": marker})
            self.updateEdges()

    def updateEdges(self):
        edges = MarkerArray()
        for u,v,data in self.waypoint_graph.edges_iter(data=True):
            edges.markers.append(data["marker"])
        self.edge_line_publisher.publish(edges)

    def processFeedback(self, feedback):

        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            handle = feedback.menu_entry_id
            if handle == MENU_CONNECT:
                self.state = STATE_CONNECT
                self.connect_first_marker = feedback.marker_name

            elif handle == MENU_CLEAR:
                self._clearMarker(feedback.marker_name)
            elif handle == MENU_REMOVE:
                self._removeMarker(feedback.marker_name)

        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            if self.state == STATE_CONNECT:
                self.state = STATE_NONE
                self._connectMarkers(self.connect_first_marker, feedback.marker_name)
            elif self.state == STATE_NONE:
                pass #ignore
            else:
                pos = feedback.pose.position
                rospy.logdebug("(Interactive_Waypoint_Server) Updateing pose of marker {3} to ({0},{1},{2})".format(pos.x, pos.y, pos.z, feedback.marker_name))
                # update database
                # push to scene database
                pstamped = PoseStamped()
                pstamped.header.frame_id = "map"
                pstamped.pose = feedback.pose

        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            if self.state == STATE_NONE:
                self.state = STATE_REGULAR

        self.server.applyChanges()

    def moveFeedback(self, feedback):
        pose = feedback.pose

        self.server.setPose(feedback.marker_name, pose)

        # correct all edges
        for u, v, data in self.waypoint_graph.edges([feedback.marker_name], data=True):
            if feedback.marker_name == data["first"]:
                data["marker"].points[0] = pose.position
            else:
                data["marker"].points[1] = pose.position

        self.updateEdges()
        self.server.applyChanges()

    def insertMarker(self, position, name=None, frame_id="map"):

        if name is None:
            name = "wp{0}".format(self.next_waypoint_id)
        self.next_waypoint_id += 1

        # insert waypoint
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = frame_id
        int_marker.pose.position = position
        int_marker.pose.orientation.w = 1
        int_marker.pose.orientation.z = 1
        int_marker.scale = 0.5

        int_marker.name = name
        int_marker.description = "Waypoint: {0}".format(name)


        # make markers moveable in the plane
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE

        # interactive menu for each marker
        menu_handler = MenuHandler()
        menu_handler.insert("Connect/Disconnect", callback=self.processFeedback)
        menu_handler.insert("Clear", callback=self.processFeedback)
        menu_handler.insert("Remove", callback=self.processFeedback)

        # make a box which also moves in the plane
        control.markers.append(self._makeMarker(int_marker))
        control.always_visible = True
        int_marker.controls.append(control)

        # we want to use our special callback function
        self.server.insert(int_marker, self.processFeedback)
        menu_handler.apply(self.server, int_marker.name)
        # set different callback for POSE_UPDATE feedback
        self.server.setCallback(int_marker.name, self.moveFeedback, InteractiveMarkerFeedback.POSE_UPDATE)
        self.server.applyChanges()

        # insert into graph
        self.waypoint_graph.add_node(int_marker.name)

    def saveWaypointsServicecall(self, request):
        filename = request.file_name
        self._saveWaypointsToFile(filename)
        rospy.loginfo("(Interactive_Waypoint_Server) Saved waypoints to {0}".format(filename))
        return SaveWaypointsResponse()

    def _saveWaypointsToFile(self, filename):
        data = {"waypoints": {}, "edges": []}
        for node in self.waypoint_graph.nodes_iter():
            pos = self.server.get(node).pose.position
            data["waypoints"].update({node : {"x": pos.x, "y": pos.y, "z" : pos.z}})
        for u,v in self.waypoint_graph.edges_iter():
            data["edges"].append([u,v])

        with open(filename, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)

    def loadWaypointsFromFile(self, filename):
        with open(filename, 'r') as f:
            data = yaml.load(f)

        for name, pos in data["waypoints"].items():
            point = Point(pos["x"], pos["y"], pos["z"])
            self.insertMarker(point, name)

        for edge in data["edges"]:
            self._connectMarkers(edge[0], edge[1])

    def removeConnectionServiceCall(self, request):
        first = request.first
        second = request.second
        if self.waypoint_graph.has_edge(first, second):
            self._connectMarkers(first, second)

        return RemoveConnectionResponse()

    def getWaypointGraphServiceCall(self, request):
        ret = GetWaypointGraphResponse()
        for r in request.waypoints:
            if not self.waypoint_graph.has_node(str(r)):
                raise rospy.ServiceException("invalid waypoint {:}".format(r))

        if len(request.waypoints) == 0:
            graph = self.waypoint_graph
        else:
            graph = self.waypoint_graph.subgraph(request.waypoints)

        for node in graph.nodes_iter():
            pose = self.server.get(node).pose
            ret.names.append(node)
            ret.positions.append(pose)

        for u,v in graph.edges_iter():
            e = Edge()
            e.source = u
            e.target = v
            ret.edges.append(e)

        return ret


if __name__ == "__main__":

    rospy.init_node("interactive_waypoint_server")
    rospy.loginfo("(Interactive_Waypoint_Server) Initializing.")

    load_file = rospy.get_param("~waypoint_file", "") # yaml file with waypoints to load
    server = InteractiveWaypointServer()
    rospy.Subscriber("/clicked_point", PointStamped, server.insertMarkerCallback)
    rospy.on_shutdown(server.clearAllMarkers)
    rospy.loginfo("(Interactive_Waypoint_Server) Waiting for rviz.")
    while server.edge_line_publisher.get_num_connections() == 0:
        rospy.sleep(0.5)

    server.clearAllMarkers()

    if len(load_file) != 0:
        rospy.loginfo("(Interactive_Waypoint_Server) Loading initial waypoint file {0}.".format(load_file))
        server.loadWaypointsFromFile(load_file)

    removeService = rospy.Service('/interactive_waypoint_server/remove_connection', RemoveConnection, server.removeConnectionServiceCall)
    saveService = rospy.Service('/interactive_waypoint_server/save_waypoints', SaveWaypoints, server.saveWaypointsServicecall)
    getWaypointGraphService = rospy.Service('/interactive_waypoint_server/get_waypoint_graph', GetWaypointGraph, server.getWaypointGraphServiceCall)

    rospy.loginfo("(Interactive_Waypoint_Server) Ready.")

    rospy.spin()

