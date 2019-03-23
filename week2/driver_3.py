
from collections import deque
import heapq
import time
import sys

if sys.platform == "win32":
    import psutil
    #print("psutil", psutil.Process().memory_info().rss)
else:
    # Note: if you execute Python from cygwin,
    # the sys.platform is "cygwin"
    # the grading system's sys.platform is "linux2"
    import resource
    #print("resource", resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
import math

#### SKELETON CODE ####

## The Class that Represents the Puzzle

class PuzzleState(object):

    """docstring for PuzzleState"""

    def __init__(self, config, n, parent=None, action="Initial", cost=0):

        if n*n != len(config) or n < 2:

            raise Exception("the length of config is not correct!")

        self.n = n

        self.cost = cost

        self.parent = parent

        self.action = action

        self.dimension = n

        self.config = config

        self.children = []

        for i, item in enumerate(self.config):

            if item == 0:

                self.blank_row = i // self.n

                self.blank_col = i % self.n

                break

    def display(self):

        for i in range(self.n):

            line = []

            offset = i * self.n

            for j in range(self.n):

                line.append(self.config[offset + j])

            print (line)



    def move_left(self):

        if self.blank_col == 0:

            return None

        else:

            blank_index = self.blank_row * self.n + self.blank_col

            target = blank_index - 1

            new_config = list(self.config)

            new_config[blank_index], new_config[target] = new_config[target], new_config[blank_index]

            return PuzzleState(tuple(new_config), self.n, parent=self, action="Left", cost=self.cost + 1)

    def move_right(self):

        if self.blank_col == self.n - 1:

            return None

        else:

            blank_index = self.blank_row * self.n + self.blank_col

            target = blank_index + 1

            new_config = list(self.config)

            new_config[blank_index], new_config[target] = new_config[target], new_config[blank_index]

            return PuzzleState(tuple(new_config), self.n, parent=self, action="Right", cost=self.cost + 1)

    def move_up(self):

        if self.blank_row == 0:

            return None

        else:

            blank_index = self.blank_row * self.n + self.blank_col

            target = blank_index - self.n

            new_config = list(self.config)

            new_config[blank_index], new_config[target] = new_config[target], new_config[blank_index]

            return PuzzleState(tuple(new_config), self.n, parent=self, action="Up", cost=self.cost + 1)

    def move_down(self):

        if self.blank_row == self.n - 1:

            return None

        else:

            blank_index = self.blank_row * self.n + self.blank_col

            target = blank_index + self.n

            new_config = list(self.config)

            new_config[blank_index], new_config[target] = new_config[target], new_config[blank_index]

            return PuzzleState(tuple(new_config), self.n, parent=self, action="Down", cost=self.cost + 1)

    def expand(self):

        """expand the node"""

        # add child nodes in order of UDLR

        if len(self.children) == 0:

            up_child = self.move_up()

            if up_child is not None:

                self.children.append(up_child)

            down_child = self.move_down()

            if down_child is not None:

                self.children.append(down_child)

            left_child = self.move_left()

            if left_child is not None:

                self.children.append(left_child)

            right_child = self.move_right()

            if right_child is not None:

                self.children.append(right_child)

        return self.children


def find_path(state, path,depth):
    if state.parent != None:
        depth += 1
        path.append(state.action)
        path, depth = find_path(state.parent, path,depth)
    return path, depth

class Frontier:
    def __init__(self):
        self.data = deque()
        self.set_config = set()

    def enqueue(self,item):
        self.data.append(item)
        self.set_config.add(item.config)

    def dequeue(self):
        item = self.data.popleft()
        self.set_config.remove(item.config)
        return item

    def push(self,item):
        self.data.append(item)
        self.set_config.add(item.config)

    def pop(self):
        item = self.data.pop()
        self.set_config.remove(item.config)
        return item

    def empty(self):
        return len(self.data)==0

    def in_frontier(self,item_config):
        return item_config in self.set_config


class Priority_Queue:
    def __init__(self):
        self.data = []
        self.entry_finder = {}
        self.order = 0

    def empty(self):
        return len(self.entry_finder)==0

    def in_queue(self,item_config):
        return item_config in self.entry_finder

    def add_item(self, priority,item, level):
        self.order +=1
        entry = [priority,self.order,item,level]
        heapq.heappush(self.data,entry)
        self.entry_finder[item.config] = entry

    def decrease_key(self, priority, item,level):
        item_in_data = self.entry_finder[item.config]
        if priority < item_in_data[0]:
            item_in_data[-2] = None
            self.order +=1
            new_item_in_data = [priority, self.order, item,level]
            heapq.heappush(self.data,new_item_in_data)
            self.entry_finder[item.config] = new_item_in_data

    def pop_item(self):
        while self.data:
            priority, order, item, level = heapq.heappop(self.data)
            if item != None:
                del self.entry_finder[item.config]
                return item,level
        raise KeyError('pop from an empty priority queue')



# Function that Writes to output.txt
### Students need to change the method to have the corresponding parameters
def writeOutput(path, cost, nodes, search_depth, max_search_depth, time, memory):
    ### Student Code Goes here ###
    f = open("output.txt", "w")
    f.write("path_to_goal: {} \n".format(path))
    f.write("cost_of_path: {}\n".format(cost))
    f.write("nodes_expanded: {}\n".format(nodes))
    f.write("search_depth: {}\n".format(search_depth))
    f.write("max_search_depth: {}\n".format(max_search_depth))
    f.write("running_time: {:1.8f}\n".format(time))
    f.write("max_ram_usage: {:1.8f}\n".format(memory))
    f.close()


def bfs_search(initial_state):
    """BFS search"""
    ### STUDENT CODE GOES HERE ###
    start_time = time.time()

    frontier = Frontier()
    frontier.enqueue(initial_state)
    explored = set()

    nodes_in_current_level = 1 #number of nodes in expading level
    nodes_in_next_level = 0 # total number of children nodes of nodes in expanding level
    level = 0

    while not frontier.empty():
        state = frontier.dequeue()

        nodes_in_current_level -=1
        if nodes_in_current_level < 0:
            nodes_in_current_level = nodes_in_next_level -1
            nodes_in_next_level = 0
            level +=1

        explored.add(state.config)

        if test_goal(state):
            stop_time = time.time()
            running_time = stop_time - start_time

            max_ram_usage = psutil.Process().memory_info().rss/1048576 # bytes-megabytes

            path,search_depth = find_path(state,[],0)
            path_to_goal = str(path[::-1])
            cost_of_path = len(path)
            node_expanded = len(explored)-1  # if this state is goal state, don't need to expand it
            if nodes_in_next_level !=0:
                max_search_depth = level +1
            else:
                max_search_depth = level



            writeOutput(path_to_goal, cost_of_path, node_expanded, search_depth, max_search_depth, running_time, max_ram_usage)
            return state.display()

        children = state.expand()

        for child in children:
            if child.config not in explored:
                if not frontier.in_frontier(child.config):
                    frontier.enqueue(child)
                    if nodes_in_current_level >= 0:
                        nodes_in_next_level +=1
    return None


def dfs_search(initial_state):
	"""DFS search"""
    ### STUDENT CODE GOES HERE ###
	start_time = time.time()

	frontier = Frontier()
	frontier.push(initial_state)
	explored = set()

	max_search_depth = 0

	level = 0
	nodes_in_current_level = 1 # number of nodes in expading level
	num_of_nodes ={} # list number of nodes in each level
	path = deque()

	while not frontier.empty():
		state = frontier.pop()
		explored.add(state.config)

		nodes_in_current_level -=1

		if nodes_in_current_level >= 0:
			num_of_nodes[level] = nodes_in_current_level
		else:
			while num_of_nodes[level] == 0:  # go back to previos level, if all of nodes in previos level expanded -> go back to upper level
				path.pop()
				level -= 1
			num_of_nodes[level] -= 1
			nodes_in_current_level = num_of_nodes[level]


		if test_goal(state):
			stop_time = time.time()
			running_time = stop_time - start_time
			max_ram_usage = psutil.Process().memory_info().rss/1048576
			node_expanded = len(explored)-1  # if this state is goal state, don't need to expand it

			if state.action != "Initial":
				path.append(state.action)
			path_to_goal = str(list(path))
			cost_of_path = len(path)
			search_depth = len(path)
			if max_search_depth < level:
				max_search_depth = level
			writeOutput(path_to_goal, cost_of_path, node_expanded, search_depth, max_search_depth, running_time, max_ram_usage)
			return state.display()

		children = state.expand()
		children.reverse()

		next_nodes = 0
		for child in children:
			if child.config not in explored:
				if not frontier.in_frontier(child.config):
					frontier.push(child)
					next_nodes +=1

		if next_nodes !=0:
			nodes_in_current_level = next_nodes
			level +=1
			if state.action != "Initial":
				path.append(state.action)
		else:
			if max_search_depth < level:
				max_search_depth = level

	return None



def A_star_search(initial_state):
    """A * search"""
    ### STUDENT CODE GOES HERE ###
    start_time = time.time()

    frontier = Priority_Queue()
    level =0
    priority = calculate_total_cost(initial_state) + level
    frontier.add_item(priority,initial_state,level)
    explored = set()
    max_search_depth = 0

    while not frontier.empty():
        state,level = frontier.pop_item()
        explored.add(state.config)

        if test_goal(state):

            stop_time = time.time()
            running_time = stop_time - start_time

            max_ram_usage = psutil.Process().memory_info().rss/1048576
            if level > max_search_depth:
                max_search_depth = level

            path,search_depth = find_path(state,[],0)
            path_to_goal = str(path[::-1])
            cost_of_path = len(path)
            node_expanded = len(explored)-1

            writeOutput(path_to_goal, cost_of_path, node_expanded, search_depth, max_search_depth, running_time, max_ram_usage)
            return state.display()

        children = state.expand()
        level +=1



        if level > max_search_depth:
            max_search_depth = level

        for child in children:
            if child.config not in explored:
                priority = calculate_total_cost(child) + level
                if not frontier.in_queue(child.config):
                    frontier.add_item(priority,child,level)
                else:
                    frontier.decrease_key(priority,child,level)



def calculate_total_cost(state):
    """calculate the total estimated cost of a state"""
    ### STUDENT CODE GOES HERE ###
    total_cost = 0
    for idx, value in enumerate(state.config):
        if value !=0:
            total_cost += calculate_manhattan_dist(idx, value, state.n)
    return total_cost
def calculate_manhattan_dist(idx, value, n):
    """calculate the manhattan distance of a tile"""
    ### STUDENT CODE GOES HERE ###
    idx_row = idx // n
    idx_col = idx % n
    value_row = value // n
    value_col = value % n

    return abs(idx_row - value_row) + abs(idx_col - value_col)

def test_goal(puzzle_state):
    """test the state is the goal state or not"""
    ### STUDENT CODE GOES HERE ###
    return puzzle_state.config == (0,1,2,3,4,5,6,7,8)

# Main Function that reads in Input and Runs corresponding Algorithm

def main():

    sm = sys.argv[1].lower()

    begin_state = sys.argv[2].split(",")

    begin_state = tuple(map(int, begin_state))

    size = int(math.sqrt(len(begin_state)))

    hard_state = PuzzleState(begin_state, size)

    if sm == "bfs":

        bfs_search(hard_state)

    elif sm == "dfs":

        dfs_search(hard_state)

    elif sm == "ast":

        A_star_search(hard_state)

    else:

        print("Enter valid command arguments !")

if __name__ == '__main__':

    main()
