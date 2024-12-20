import util
from util import Queue, PriorityQueue

class SearchProblem:
    """
    Abstract class defining the structure of a search problem.
    Subclasses must implement the outlined methods.
    """
    def getStartState(self):
        raise NotImplementedError("This method should be overridden by subclasses.")
    
    def isGoalState(self, state):
        raise NotImplementedError("This method should be overridden by subclasses.")
    
    def getSuccessors(self, state):
        raise NotImplementedError("This method should be overridden by subclasses.")
    
    def getCostOfActions(self, actions):
        raise NotImplementedError("This method should be overridden by subclasses.")

# Tiny maze-specific search
def tinyMazeSearch(problem):
    """
    Solves a predefined tiny maze using a fixed sequence of moves.
    Note: This is specific to the tiny maze and will not work generically.
    """
    from game import Directions
    moves = [Directions.SOUTH, Directions.SOUTH, Directions.WEST,
             Directions.SOUTH, Directions.WEST, Directions.WEST,
             Directions.SOUTH, Directions.WEST]
    return moves

# Depth-First Search (DFS) implementation
def depthFirstSearch(problem):
    """
    Explores the deepest nodes in the search tree using a stack (LIFO).
    Avoids cycles using an explored set.
    """
    stack = util.Stack()
    stack.push((problem.getStartState(), []))  # (current state, path to state)
    visited = set()  # Keeps track of visited states
    
    while not stack.isEmpty():
        current_state, actions = stack.pop()
        if current_state in visited:
            continue
        visited.add(current_state)

        # Check if the goal is reached
        if problem.isGoalState(current_state):
            return actions

        # Add successors to the stack
        for next_state, action, _ in problem.getSuccessors(current_state):
            if next_state not in visited:
                stack.push((next_state, actions + [action]))
    
    return []

# Breadth-First Search (BFS) implementation
def breadthFirstSearch(problem):
    """
    Explores the shallowest nodes in the search tree using a queue (FIFO).
    """
    queue = Queue()
    queue.push((problem.getStartState(), []))
    visited = set()
    
    while not queue.isEmpty():
        current_state, actions = queue.pop()
        if current_state in visited:
            continue
        visited.add(current_state)

        if problem.isGoalState(current_state):
            return actions

        for next_state, action, _ in problem.getSuccessors(current_state):
            if next_state not in visited:
                queue.push((next_state, actions + [action]))
    
    return []

# Uniform Cost Search (UCS) implementation
def uniformCostSearch(problem):
    """
    Explores nodes with the least total cost using a priority queue.
    """
    priority_queue = PriorityQueue()
    priority_queue.push((problem.getStartState(), []), 0)
    visited = set()
    
    while not priority_queue.isEmpty():
        current_state, actions = priority_queue.pop()
        if current_state in visited:
            continue
        visited.add(current_state)

        if problem.isGoalState(current_state):
            return actions

        for next_state, action, cost in problem.getSuccessors(current_state):
            if next_state not in visited:
                total_cost = problem.getCostOfActions(actions + [action])
                priority_queue.push((next_state, actions + [action]), total_cost)
    
    return []

# A* Search (A* search with heuristic)
def aStarSearch(problem, heuristic=lambda s, p: 0):
    """
    Combines UCS with a heuristic to prioritize paths that are likely to lead to the goal.
    """
    priority_queue = PriorityQueue()
    start_state = problem.getStartState()
    priority_queue.push((start_state, []), heuristic(start_state, problem))
    visited = set()

    while not priority_queue.isEmpty():
        current_state, actions = priority_queue.pop()
        if current_state in visited:
            continue
        visited.add(current_state)

        if problem.isGoalState(current_state):
            return actions

        for next_state, action, cost in problem.getSuccessors(current_state):
            if next_state not in visited:
                new_actions = actions + [action]
                total_cost = problem.getCostOfActions(new_actions) + heuristic(next_state, problem)
                priority_queue.push((next_state, new_actions), total_cost)
    
    return []

# Aliases for easy access
dfs = depthFirstSearch
bfs = breadthFirstSearch
ucs = uniformCostSearch
astar = aStarSearch
