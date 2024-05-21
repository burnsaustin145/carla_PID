import numpy as np
import carla as c


class Node:
    def __init__(self, waypoint):
        self.waypoint = waypoint
        self.edges = []

    def add_edge(self, edge):
        self.edges.append(edge)


# class for creating a graph of waypoints on the current map
class WaypointGraph:
    nodes = []
    visited_locations = []

    def __init__(self, world_map, starting_way, search_rad):
        self.world_map = world_map
        self.starting_way = starting_way
        self.search_rad = search_rad
        self.starting_node = None
        self.build_graph(self.starting_way)

    def _build_graph_helper(self, curr_way):
        dist_start = np.sqrt((curr_way.transform.location.x - self.starting_way.transform.location.x) ** 2 +
                             (curr_way.transform.location.y - self.starting_way.transform.location.y) ** 2)

        if dist_start < 10 < len(self.nodes):
            curr = Node(curr_way)
            self.nodes.append(curr)
            return curr
        elif (curr_way.transform.location.x, curr_way.transform.location.y) in self.visited_locations:
            curr = Node(curr_way)
            self.nodes.append(curr)
            return curr

        curr = Node(curr_way)
        self.nodes.append(curr)
        self.visited_locations.append((curr.waypoint.transform.location.x, curr.waypoint.transform.location.y))

        next_way = (self.world_map.get_waypoint(curr.waypoint.transform.location, project_to_road=True,
                    lane_type=c.LaneType.Driving)).next(self.search_rad)[0]
        next_node = self._build_graph_helper(next_way)
        curr.add_edge(next_node)
        return curr

    # recursively build road graph
    def build_graph(self, start):
        # in: starting waypoint
        # out: starting node
        self.starting_node = self._build_graph_helper(start)


class Planner:
    path = []

    def __init__(self, world_map, car, search_dist):
        self.start = (world_map.get_waypoint(car.get_location(), project_to_road=True,
                                             lane_type=c.LaneType.Driving)).next(search_dist)[0]
        self.road_graph = WaypointGraph(world_map, self.start, search_dist)
        self.build_path()

    # build a path from the waypoint graph
    def build_path(self):
        curr = self.road_graph.starting_node
        self.path.append(curr)
        while curr:
            print(f"curr edges: {curr.edges}")
            if len(curr.edges) > 0:
                curr = curr.edges[0]
            else:
                break
            if curr in self.path:
                break
            if curr == self.start:
                break
            self.path.append(curr)

