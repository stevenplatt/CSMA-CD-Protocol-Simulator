# CSMA/CD Algorithm
import random
import math
import collections
import csv 
from statistics import mean

maxSimulationTime = 3600 # 1 hour simulated time

class Node:
    def __init__(self, location, A):
        self.queue = collections.deque(self.generate_queue(A))
        self.location = location  # Defined as a multiple of D
        self.collisions = 0
        self.wait_collisions = 0
        self.MAX_COLLISIONS = 10

    def collision_occured(self, R):
        self.collisions += 1
        if self.collisions > self.MAX_COLLISIONS:
            # Drop packet and reset collisions
            return self.pop_packet()

        # Add the exponential backoff time to waiting time
        backoff_time = self.queue[0] + self.exponential_backoff_time(R, self.collisions)

        for i in range(len(self.queue)):
            if backoff_time >= self.queue[i]:
                self.queue[i] = backoff_time
            else:
                break

    def successful_transmission(self):
        self.collisions = 0
        self.wait_collisions = 0

    def generate_queue(self, A):
        packets = []
        arrival_time_sum = 0

        while arrival_time_sum <= maxSimulationTime:
            arrival_time_sum += get_exponential_random_variable(A)
            packets.append(arrival_time_sum)
        return sorted(packets)

    def exponential_backoff_time(self, R, general_collisions):
        rand_num = random.random() * (pow(2, general_collisions) - 1)
        return rand_num * 4096/float(R)  # 4096 bit-times

    def pop_packet(self):
        self.queue.popleft()
        self.collisions = 0
        self.wait_collisions = 0

    def non_persistent_bus_busy(self, R):
        self.wait_collisions += 1
        if self.wait_collisions > self.MAX_COLLISIONS:
            # Drop packet and reset collisions
            return self.pop_packet()

        # Add the exponential backoff time to waiting time
        backoff_time = self.queue[0] + self.exponential_backoff_time(R, self.wait_collisions)

        for i in range(len(self.queue)):
            if backoff_time >= self.queue[i]:
                self.queue[i] = backoff_time
            else:
                break


def get_exponential_random_variable(param):
    # Get random value between 0 (exclusive) and 1 (inclusive)
    uniform_random_value = 1 - random.uniform(0, 1)
    exponential_random_value = (-math.log(1 - uniform_random_value) / float(param))

    return exponential_random_value

def build_nodes(N, A, D):
    nodes = []
    for i in range(0, N):
        nodes.append(Node(i*D, A))
    return nodes

def csma_cd(N, A, R, L, D, S, is_persistent):
    curr_time = 0
    transmitted_packets = 0
    successfuly_transmitted_packets = 0
    nodes = build_nodes(N, A, D)
    
    while True:

    # Step 1: Pick the smallest time out of all the nodes
        min_node = Node(None, A)  # Some random temporary node
        min_node.queue = [float("infinity")]
        for node in nodes:
            if len(node.queue) > 0:
                min_node = min_node if min_node.queue[0] < node.queue[0] else node

        if min_node.location is None:  # Terminate if no more packets to be delivered
            break

        curr_time = min_node.queue[0]
        transmitted_packets += 1

        # Step 2: Check if collision will happen
        # Check if all other nodes except the min node will collide
        collsion_occurred_once = False
        for node in nodes:
            if node.location != min_node.location and len(node.queue) > 0:
                delta_location = abs(min_node.location - node.location)
                t_prop = delta_location / float(S)
                t_trans = L/float(R)

                # Check collision
                will_collide = True if node.queue[0] <= (curr_time + t_prop) else False

                # Sense bus busy
                if (curr_time + t_prop) < node.queue[0] < (curr_time + t_prop + t_trans):
                    if is_persistent is True:
                        for i in range(len(node.queue)):
                            if (curr_time + t_prop) < node.queue[i] < (curr_time + t_prop + t_trans):
                                node.queue[i] = (curr_time + t_prop + t_trans)
                            else:
                                break
                    else:
                        node.non_persistent_bus_busy(R)

                if will_collide:
                    collsion_occurred_once = True
                    transmitted_packets += 1
                    node.collision_occured(R)

        # Step 3: If a collision occured then retry
        # otherwise update all nodes latest packet arrival times and proceed to the next packet
        if collsion_occurred_once is not True:  # If no collision happened
            successfuly_transmitted_packets += 1
            min_node.pop_packet()
        else:    # If a collision occurred
            min_node.collision_occured(R)

#    print("Efficiency", successfuly_transmitted_packets/float(transmitted_packets))
#    print("Throughput", (L * successfuly_transmitted_packets) / float(curr_time + (L/R)) * pow(10, -6), "Mbps")
#    print("Block Delay: ", (t_prop + t_trans) * 3)
#    print("Block Throughput:", int(successfuly_transmitted_packets / 3))

    link_efficiency = int(float(successfuly_transmitted_packets / transmitted_packets)*100)
    block_throughput = int(successfuly_transmitted_packets / 3)

    return link_efficiency # change between "block_throughput" and "link_efficiency" simulations

# Run Algorithm
# N = The number of nodes/computers connected to the LAN
# A = Average packet arrival rate (packets per second)
# R = The speed of the LAN/channel/bus (in bps)
# L = Packet length (in bits)
# D = Distance between adjacent nodes on the bus/channel
# S = Propagation speed (meters/sec)

D = 1500 * 1000 # conversion to kilometers
C = 3 * pow(10, 8) # speed of light
S = (2/float(3)) * C

rate_a = 1 / float(60*60) # 1 block per hour
rate_b = 1 / float(60) # 1 block per minute
rate_c = 1 # 1 block per second

def simulation():    # Show the system efficiency, block delay, and block throughput (CSMA/CD Persistent)
    for N in range(2, 11, 1):

        node_throughput = []
        rate_throughput = []
        csma_result = []
        
        for A in [rate_a, rate_b, rate_c]:    

            print("Peer Nodes:",N,"; Blocks Per Hour:",A * (60*60))
            for _ in range(3): # run simulation 10 times

                R = 1 * pow(10, 9)
                L = 1500
                data = csma_cd(N, A, R, L, D, S, True)
                csma_result.append(data)
                csma_ave = mean(csma_result)
        
            rate_throughput.append(int(csma_ave))

        rate_throughput.insert(0, N)
        print(rate_throughput)
        print("")

        # append results to csv file
        row = rate_throughput

        with open(filename_1, 'a+', newline='') as csvfile:   
            csvwriter = csv.writer(csvfile)  # creating a csv writer object  
            csvwriter.writerow(row) # writing the data rows

# create CSV files to store outputs
fields_1 = ['Node Count', 'Block Per Hour', 'Block Per Minute', 'Block Per Second'] # header fields
filename_1 = "block_simulation.csv"

# writing to csv file  
with open(filename_1, 'w') as csvfile:  
    csvwriter = csv.writer(csvfile)  # creating a csv writer object 
    csvwriter.writerow(fields_1)  # writing the fields 

simulation()
