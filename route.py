#!/usr/local/bin/python3
# route.py : Find routes through maps
#
# Code by: Sripal reddy Nomula: srnomula
#
# Based on skeleton code by V. Mathur and D. Crandall, Fall 2022
#


# !/usr/bin/env python3
import sys
import csv

"""
Find shortest driving route between start city and end city
based on a cost function.

1. Your function should return a dictionary having the following keys:
    -"route-taken" : a list of pairs of the form (next-stop, segment-info), where
       next-stop is a string giving the next stop in the route, and segment-info is a free-form
       string containing information about the segment that will be displayed to the user.
       (segment-info is not inspected by the automatic testing program).
    -"total-segments": an integer indicating number of segments in the route-taken
    -"total-miles": a float indicating total number of miles in the route-taken
    -"total-hours": a float indicating total amount of time in the route-taken
    -"total-delivery-hours": a float indicating the expected (average) time 
                               it will take a delivery driver who may need to return to get a new package
2. Do not add any extra parameters to the get_route() function, or it will break our grading and testing code.
3. Please do not use any global variables, as it may cause the testing code to fail.
4. You can assume that all test cases will be solvable.
5. The current code just returns a dummy solution.
"""

# step1: As we need lot of heuristics for different parameter in the successor function we will define seperate functions
#       to simplify the code

import pandas as pd
from math import radians, sin, cos, asin, sqrt, tanh
import heapq as hq
from collections import defaultdict


# initially we are given a city_gps data in a text file and we need to access it.

class AstarSearch:
    def __init__(self, start_node, end_node):
        self.city_gps_dict = self.city_gps()
        self.road_seg_dict, self.max_speed = self.road_segments()
        self.start_node, self.end_node = start_node, end_node
        self.initialise_state_list()
        self.total_states = len(self.state_list)
        self.visited_state_list = []
        self.state_visited = 0

    def initialise_state_list(self):
        self.state_list = set()
        for keys in self.city_gps_dict.keys():
            self.state_list.add(keys.split(",")[1])

    def city_gps(self):
        city_gps_dict = {}
        data = pd.read_csv('city-gps.txt', delimiter=' ', header=None, names=['city', 'lat', 'long']).drop_duplicates()
        for _, row in data.iterrows():
            try:

                city_gps_dict[row['city']] = (row['lat'], row['long'])
            except:
                pass

        return city_gps_dict

    #
    def road_segments(self):
        road_seg_dict = defaultdict(list)
        data = pd.read_csv("road-segments.txt", delimiter=' ', header=None)

        max_speed = data[3].max()

        for _, row in data.iterrows():
            try:

                road_seg_dict[row[0]].append((row[1], row[2], row[3], row[4]))
                road_seg_dict[row[1]].append((row[0], row[2], row[3], row[4]))
            except:
                print("Missing entry for one of the fields: ", row)

        return road_seg_dict, max_speed

    def time_heuristic(self, coord1):
        return self.haversine(coord1, self.end_node) / self.max_speed

    # As given from the problem description of time taken by delivery time with the probability of package destroyed.
    def delivery_heuristic(self, l_road, speed_limit, t_road, t_trip):
        # formula generated from the question
        if speed_limit >= 50:
            prob = tanh((l_road) / 1000)

            d_time = 2 * (prob * (t_road + t_trip)) + t_road

        else:
            d_time = t_road

        return d_time

    #
    # # we need to find the distance between given two gps co-ordiantes of city(from latitudes & longitudes)
    # # this can be achieved by using the Haversine distance formula is referred from the site
    # # https://stackoverflow.com/questions/4913349/haversine-formula-in-python-bearing-and-distance-between-two-gps-points
    # # as shown below:

    def haversine(self, coord1):

        # convert decimal degrees to radians

        coord2 = self.city_gps_dict[self.end_node]
        lon1, lat1, lon2, lat2 = map(radians, [coord1[0], coord1[1], coord2[0], coord2[1]])

        # haversine formula
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
        b = 2 * asin(sqrt(a))
        c = 3956  # radius of earth= 3956 in miles
        return b * c
        # reference end

    def get_neighbours(self, current_node):
        # print(self.road_seg_dict)

        return self.road_seg_dict[current_node]

    def get_heuristic(self, cost, segments, seg_distance, speed_limit, node_lat, node_long, t_road, t_trip,
                      state_value):

        if cost == "distance":
            return self.haversine((node_lat, node_long))
        elif cost == "segments":
            return segments + 1
        elif cost == "time":
            return self.haversine((node_lat, node_long)) / self.max_speed
        elif cost == "statetour":
            return self.remaining_states(state_value)
        else:
            # delivery:
            return self.delivery_heuristic(seg_distance, speed_limit, t_road, t_trip)

    def remaining_states(self, state_value):
        if state_value in self.state_list and (state_value != self.start_node.split(",")[1]):
            self.state_visited += 1
            self.state_list.remove(state_value)
            self.visited_state_list.append(state_value)
        return len(self.state_list)

    def calculate_cost_f(self, cost, node_info, h_value):
        time_till_now, segments, miles, delivery_time = node_info
        if cost == "distance":
            return miles + h_value
        elif cost == "segments":
            return segments + 1
        elif cost == "time":
            return time_till_now + h_value
        elif cost == "statetour":
            return h_value + self.state_visited
        else:
            # delivery
            return delivery_time + h_value


#
#
def get_route(start_node, end_node, cost):
    astar = AstarSearch(start_node, end_node)

    open_list = []
    visited_nodes = defaultdict(dict)

    route_taken = [start_node]

    seg_distance, speed_limit = 0, 0
    t_trip = float('inf')  # initial time set to infinity
    time_till_now, segments, miles, delivery_time, t_road = 0, 0, 0, 0, 0
    node_lat, node_long = astar.city_gps_dict[start_node]
    state_value = start_node.split(",")[1]
    h_value = astar.get_heuristic(cost, segments, seg_distance, speed_limit, node_lat, node_long, t_road, t_trip,
                                  state_value)

    hq.heappush(open_list, (0 + h_value, (start_node, route_taken, time_till_now, segments, miles, delivery_time)))

    while len(open_list) > 0:

        # print("statevisited", astar.state_visited)
        curr_node = hq.heappop(open_list)

        prev_cost, (node, route_taken, time_till_now, segments_prev, miles_prev, delivery_time_prev) = curr_node

        if cost != "statetour":
            if node not in route_taken:
                route_taken.append(node)
            else:

                # the current node is already present so remove the nodes from the previous node to end.
                pos_index = route_taken.index(node)
                route_taken = route_taken[:pos_index + 1]
        else:
            if node != astar.end_node and astar.state_visited != astar.total_states:
                route_taken.append(node)

        if node == astar.end_node:
            # goal found
            final_route = []

            for i in range(len(route_taken) - 1):
                next_destination_list = astar.road_seg_dict[route_taken[i]]
                next_destination = [n[3] for n in next_destination_list if n[0] == route_taken[i + 1]]

                final_route.append((route_taken[i + 1], next_destination[0]))

            route_taken.remove(start_node)  # remove starting node
            print(astar.visited_state_list, len(astar.visited_state_list))

            return {"total-segments": len(route_taken),
                    "total-miles": miles_prev,
                    "total-hours": time_till_now,
                    "total-delivery-hours": delivery_time_prev,
                    "route-taken": final_route}



        else:
            visited_nodes[node] = (False, prev_cost)

            neighbours = astar.get_neighbours(node)

            for temp_node in neighbours:
                city_name, seg_distance, speed_limit, us_route = temp_node
                state_value = city_name.split(",")[1]
                time_till_current = time_till_now + seg_distance / speed_limit
                delivery_time_current = delivery_time_prev + astar.delivery_heuristic(seg_distance, speed_limit,
                                                                                      seg_distance / speed_limit,
                                                                                      time_till_now)
                if city_name not in astar.city_gps_dict.keys():
                    curr_h = 0
                else:
                    node_lat, node_long = astar.city_gps_dict[city_name]

                    curr_h = astar.get_heuristic(cost, segments, seg_distance, speed_limit, node_lat, node_long,
                                                 seg_distance / speed_limit, delivery_time_prev, state_value)
                    # print(curr_h)

                curr_f = astar.calculate_cost_f(cost, (
                    time_till_current, segments_prev, miles_prev + seg_distance, delivery_time_current), curr_h)

                if city_name in visited_nodes and curr_f < visited_nodes[city_name][1] and cost != "segments":
                    visited_nodes[city_name] = (False, curr_f)
                    hq.heappush(open_list, (
                        round(curr_f, 4),
                        (city_name, route_taken + [city_name], round(time_till_current, 2), segments_prev + 1,
                         miles_prev + seg_distance,
                         round(delivery_time_current, 2))))

                elif city_name not in visited_nodes:

                    hq.heappush(open_list, (
                        round(curr_f, 4),
                        (city_name, route_taken + [city_name], round(time_till_current, 2), segments_prev + 1,
                         miles_prev + seg_distance,
                         round(delivery_time_current, 2))))


# Please don't modify anything below this line
#
if __name__ == "__main__":
    pass
    if len(sys.argv) != 4:
        raise (Exception("Error: expected 3 arguments"))

    (_, start_city, end_city, cost_function) = sys.argv
    if cost_function not in ("segments", "distance", "time", "delivery"):
        raise (Exception("Error: invalid cost function"))

    result = get_route(start_city, end_city, cost_function)

    # result = get_route('Bloomington,_Indiana', 'Indianapolis,_Indiana', 'distance')
    # Pretty print the route
    print("Start in %s" % start_city)
    # print(result)
    for step in result["route-taken"]:
        print("   Then go to %s via %s" % step)

    print("\n          Total segments: %4d" % result["total-segments"])
    print("             Total miles: %8.3f" % result["total-miles"])
    print("             Total hours: %8.3f" % result["total-hours"])
    print("Total hours for delivery: %8.3f" % result["total-delivery-hours"])
