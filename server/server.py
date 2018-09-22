"""
Mehrab Mehdi Islam
Student Id: 1545664

Alexandar Chan
Student Id: 1544806

"""
from serial import Serial
from time import sleep
from graph import Graph
import math
from binary_heap import BinaryHeap


def least_cost_path(graph, start, dest, cost):
    """
    Find and return a least cost path in graph from start
    vertex to dest vertex.

    Efficiency: If E is the number of edges, the run-time is
    O( E log(E) ).

    Args:
        graph (Graph): The digraph defining the edges between the
            vertices.

        start: The vertex where the path starts. It is assumed
            that start is a vertex of graph.

        dest: The vertex where the path ends. It is assumed
            that dest is a vertex of graph.

        cost: A class with a method called "distance" that takes
            as input an edge (a pair of vertices) and returns the cost
            of the edge. For more details, see the CostDistance class
            description below.

    Returns:
        list: A potentially empty list (if no path can be found) of
            the vertices in the graph. If there was a path, the first
            vertex is always start, the last is always dest in the list.
            Any two consecutive vertices correspond to some
            edge in graph.
    """

    reached = {}  # dictionary to stored read_city_graph_undirected
    events = BinaryHeap()  # a heap called events is called
    events.insert([start, start], 0)  # the start point is inserted

    while events:
        edge, time = events.popmin()  # get edge and time from events
        if edge[1] not in reached:  # if v is not in the events dictionary
            reached[edge[1]] = edge[0]  # v in reached is assigned u
            for nbr in graph.neighbours(edge[1]):  # for the variable in the neighbours of v
                events.insert(((edge[1]), nbr), (time + cost.distance((edge[1], nbr)))) # insert into heap

    if dest not in reached:  # if the dest in not in the reached dict
        return []  # return an empty list

    path = [dest]  # path is a list containing dest

    while dest != start:  # if start not equal to dest
        dest = reached[dest]  # dest is the value assigned to the destination in the reached dictionary
        path.append(dest)  # add the dest to the path dictionary

    path.reverse()  # reverse the order of the list

    return path  # return the list


def load_edmonton_graph(filename):

    g = Graph()  # g is the graph class
    locations = {}  # empty dict to store the locations

    with open(filename, "r") as file:
        for line in file:
            line = line.strip().split(",")
            # splits the data

            if line[0] == "V":
                g.add_vertex(int(line[1]))
                # appends  the vertices
                locations[int(line[1])] = [int(float(line[2])*100000),int(float(line[3])*100000)]
                # adds location
            elif line[0] == "E":
                g.add_edge((int(line[1]), int(line[2])))
                # appends the edges

    return g, locations


def euclid_dist(coord1, coord2):
    Euclidean_dist = math.sqrt((coord1[0]-coord2[0])**2+(coord1[1]-coord2[1])**2)  #calculates euclidian distance between 2 points
    return Euclidean_dist



class CostDistance:
    """
    A class with a method called distance that will return the Euclidean
    between two given vertices.
    """
    def __init__(self, location):
        """
        Creates an instance of the CostDistance class and stores the
        dictionary "location" as a member of this class.
        """
        self.location = location

    def distance(self, e):
        """
        Here e is a pair (u,v) of vertices.
        Returns the Euclidean distance between the two vertices u and v
        """

        coord1, coord2 = e

        return euclid_dist(self.location[coord1], self.location[coord2])  # calculates euclidian distance between 2 coordinates

# The TA help me with the creation of this function
# It finds the closest vertices to the current vertce


def closest_vertices(location, coordinates):
    if coordinates in list(location.values()):
        return list({k: v for k, v in location.items()
                     if v == coordinates}.keys())[0]
    check = float("inf")
    ver = (0, 0)
    for v in location:
        xsum = math.fabs(int(location[v][0]) - int(coordinates[0]))
        ysum = math.fabs(int(location[v][1]) - int(coordinates[1]))
        if check > (xsum + ysum):
            check = (xsum + ysum)
            ver = v
    return ver


# Communicates the server side with the arduino
def arduino_talk():
    with Serial("/dev/ttyACM0", baudrate=9600, timeout=0.1) as ser:
        while True:
            # Infinite loop that echoes all messages from
            # the arduino to the terminal
            line = ser.readline()

            # If there is no input, restart
            if not line:
                print("timeout, restarting...")
                continue

            # Decode the line from ASCII
            line_string = line.decode("ASCII")

            # Split the string of carriage returns and new lines
            stripped = line_string.rstrip("\r\n")

            # Print out the start and end coordinates
            print(stripped)

            # If the line starts witht the chracter 'R', save te subsequent
            # coordinates as the starting and ending point
            if stripped[0] == 'R':
                request = stripped.split()
                start = closest_vertices(location, (request[1], request[2]))
                end = closest_vertices(location, (request[3], request[4]))

                # create an empty path list and fill it with the
                # least cost pathway
                path = []
                path = least_cost_path(edmonton, start, end, cost)

                # If path is empty (there is no possible route), print 0 pathways
                # Otherwise, print the number of pathways to the arduino
                if not path:
                    waypoints = "N 0\n"
                    encoded = waypoints.encode("ASCII")
                    ser.write(encoded)
                else:
                    waypoints = "N " + str(len(path)) + "\n"
                    encoded = waypoints.encode("ASCII")
                    ser.write(encoded)
                    continue

            # If the arduino is asking for the cooridnates of the waypoint
            # and there are still waypints in the path, return the character
            # 'W' along with the coordinates followed by a newline. Otherwise,
            # if the path is empty, return the character 'E'
            # Then encode everything in ASCII and write it to the arduino
            elif stripped[0] == 'A':
                if path:
                    coords = location[path.pop(0)]
                    waypoints = "W " + str(coords[0]) + " " + str(coords[1]) + "\n"
                else:
                    waypoints = "E \n"   # path is now empty

                encoded = waypoints.encode("ASCII")
                ser.write(encoded)
                continue
            else:
                out_line = "%"
                encoded = out_line.encode("ASCII")
                ser.write(encoded)

            sleep(2)
    return 0


# if running this specific script, await user command
if __name__ == "__main__":
    edmonton, location = load_edmonton_graph("edmonton-roads-2.0.1.txt")
    cost = CostDistance(location)
    arduino_talk()
