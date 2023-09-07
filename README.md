# Route-planner

Algorithm: As we have large set of data and inorder to traverse and find an efficient,  quick solution and optimum solution, A* algorithm would be the best for path finding with different cost functions included.
  

In this part we need to find the route between start-city and end-city.
we are given with 2 datasets:
city-gps: This is a dataset of cities with their respective latitude and longitudes.	
         in the format: [city,_state lati long]

road-segments: This is a dataset of raod segments that are connecting 2 cities.
	   first city, second city, state distance(in miles) max_speed highway

cost-function: It gives the total cost below parameter when called with repective name as below

-segments: This gives a route minimum number of road segments

-distance: This gives a route with minimum distance

-time    : This gives a route which can be covered in less amount time

-delivery: This gives a route  where delivery driver can reach the goal in less time even with the probability of falling off package atmost once.

-statetour: This gives the shortest route between cities but passes through atleast once city of all US states.

As we proceed further we need following 3 parameter:

f = g + h

g is the cost of the path

h is the Heuristic function(this gives us the least cost from curr node to goal node)

where f is total cost

In order to find the heuristic we used 'haversine distance' which gives distance between 2 given coordinates though which is lesser than the actual distance when covered in actual manner.
