import heapq

class PriorityQueue:
    def __init__(self):
        self.elements = []
    
    def empty(self):
        return len(self.elements) == 0
    
    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))
    
    def get(self):
        return heapq.heappop(self.elements)[1]

possibleDir = ["North","East","South","West"];
possibleMoves = ["Left","Right","Forward"]
dirVectors = {"North":(0,-1),"East":(1,0),"South":(0,1),"West":(-1,0)}

def makeMove(state,action,grid):
    (x,y,d) = state
    lenX = len(grid[0])
    lenY = len(grid)

    if action == "Left":
        ind = possibleDir.index(d)
        d = possibleDir[(ind-1)%4]
        return (x,y,d)

    if action == "Right":
        ind = possibleDir.index(d)
        d = possibleDir[(ind+1)%4]
        return (x,y,d)

    if action == "Forward":
        vec = dirVectors[d]
        (newX,newY) = (x+vec[0],y+vec[1])
        if 0 <= newX and newX < lenX and 0 <= newY and newY < lenY and grid[newY][newX] != 1:
            return (newX,newY,d)
    return None


def expandNode(grid,state):
    states = []
    for action in possibleMoves:
        nextState = makeMove(state,action,grid)  
        if nextState != None:
            states += [(nextState,action)]
    return states

def heuristic(cur,final):
    (x,y,d) = cur
    (x1,y1,d1) = final
    return abs(x-x1) + abs(y-y1)

def a_star_search(grid, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    
    while not frontier.empty():
        current = frontier.get()
        if current == goal:
            return make_path(came_from,goal,start)
        for (next,action) in expandNode(grid,current):
            new_cost = cost_so_far[current] + 1
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = (current,action)
    print("No path found")
    return None

def make_path(came_from,goal,start):
    path = []
    curr = goal
    while came_from[curr] != None:
        (prev,action) = came_from[curr]
        path.insert(0,action)
        curr = prev
    return path
