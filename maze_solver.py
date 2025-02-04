import random
import copy
import heapq
import time

# Function to generate a maze using a randomized algorithm
def generate_maze(height, width):
    # Function to count the number of surrounding cells
    def surroundingCells(rand_wall):
        s_cells = 0
        if (maze[rand_wall[0]-1][rand_wall[1]] == 'c'):
            s_cells += 1
        if (maze[rand_wall[0]+1][rand_wall[1]] == 'c'):
            s_cells += 1
        if (maze[rand_wall[0]][rand_wall[1]-1] == 'c'):
            s_cells +=1
        if (maze[rand_wall[0]][rand_wall[1]+1] == 'c'):
            s_cells += 1
        return s_cells

    wall = 'w'
    cell = 'c'
    unvisited = 'u'
    maze = []

    # Create an empty maze grid
    for i in range(0, height):
        line = []
        for j in range(0, width):
            line.append(unvisited)
        maze.append(line)

    # Randomly choose starting point for the maze
    starting_height = int(random.random()*height)
    starting_width = int(random.random()*width)
    if (starting_height == 0):
        starting_height += 1
    if (starting_height == height-1):
        starting_height -= 1
    if (starting_width == 0):
        starting_width += 1
    if (starting_width == width-1):
        starting_width -= 1

    # Set the starting point as a cell
    maze[starting_height][starting_width] = cell

    # Add surrounding walls to the starting point
    walls = []
    walls.append([starting_height - 1, starting_width])
    walls.append([starting_height, starting_width - 1])
    walls.append([starting_height, starting_width + 1])
    walls.append([starting_height + 1, starting_width])

    maze[starting_height-1][starting_width] = 'w'
    maze[starting_height][starting_width - 1] = 'w'
    maze[starting_height][starting_width + 1] = 'w'
    maze[starting_height + 1][starting_width] = 'w'

    # While there are walls in the list
    while (walls):
        rand_wall = walls[int(random.random()*len(walls))-1]

        # Check surrounding cells of the randomly chosen wall
        if (rand_wall[1] != 0):
            if (maze[rand_wall[0]][rand_wall[1]-1] == 'u' and maze[rand_wall[0]][rand_wall[1]+1] == 'c'):
                s_cells = surroundingCells(rand_wall)

                # If there are less than 2 surrounding cells, convert wall to cell
                if (s_cells < 2):
                    maze[rand_wall[0]][rand_wall[1]] = 'c'

                    # Add adjacent walls to the list
                    if (rand_wall[0] != 0):
                        if (maze[rand_wall[0]-1][rand_wall[1]] != 'c'):
                            maze[rand_wall[0]-1][rand_wall[1]] = 'w'
                        if ([rand_wall[0]-1, rand_wall[1]] not in walls):
                            walls.append([rand_wall[0]-1, rand_wall[1]])

                    if (rand_wall[0] != height-1):
                        if (maze[rand_wall[0]+1][rand_wall[1]] != 'c'):
                            maze[rand_wall[0]+1][rand_wall[1]] = 'w'
                        if ([rand_wall[0]+1, rand_wall[1]] not in walls):
                            walls.append([rand_wall[0]+1, rand_wall[1]])

                    if (rand_wall[1] != 0):
                        if (maze[rand_wall[0]][rand_wall[1]-1] != 'c'):
                            maze[rand_wall[0]][rand_wall[1]-1] = 'w'
                        if ([rand_wall[0], rand_wall[1]-1] not in walls):
                            walls.append([rand_wall[0], rand_wall[1]-1])

                # Remove the current wall from the list
                for wall in walls:
                    if (wall[0] == rand_wall[0] and wall[1] == rand_wall[1]):
                        walls.remove(wall)

                continue

        # Repeat the process for other directions
        if (rand_wall[0] != 0):
            if (maze[rand_wall[0]-1][rand_wall[1]] == 'u' and maze[rand_wall[0]+1][rand_wall[1]] == 'c'):
                s_cells = surroundingCells(rand_wall)
                if (s_cells < 2):
                    maze[rand_wall[0]][rand_wall[1]] = 'c'

                    if (rand_wall[0] != 0):
                        if (maze[rand_wall[0]-1][rand_wall[1]] != 'c'):
                            maze[rand_wall[0]-1][rand_wall[1]] = 'w'
                        if ([rand_wall[0]-1, rand_wall[1]] not in walls):
                            walls.append([rand_wall[0]-1, rand_wall[1]])

                    if (rand_wall[1] != 0):
                        if (maze[rand_wall[0]][rand_wall[1]-1] != 'c'):
                            maze[rand_wall[0]][rand_wall[1]-1] = 'w'
                        if ([rand_wall[0], rand_wall[1]-1] not in walls):
                            walls.append([rand_wall[0], rand_wall[1]-1])

                    if (rand_wall[1] != width-1):
                        if (maze[rand_wall[0]][rand_wall[1]+1] != 'c'):
                            maze[rand_wall[0]][rand_wall[1]+1] = 'w'
                        if ([rand_wall[0], rand_wall[1]+1] not in walls):
                            walls.append([rand_wall[0], rand_wall[1]+1])

                for wall in walls:
                    if (wall[0] == rand_wall[0] and wall[1] == rand_wall[1]):
                        walls.remove(wall)

                continue

        if (rand_wall[0] != height-1):
            if (maze[rand_wall[0]+1][rand_wall[1]] == 'u' and maze[rand_wall[0]-1][rand_wall[1]] == 'c'):
                s_cells = surroundingCells(rand_wall)
                if (s_cells < 2):
                    maze[rand_wall[0]][rand_wall[1]] = 'c'

                    if (rand_wall[0] != height-1):
                        if (maze[rand_wall[0]+1][rand_wall[1]] != 'c'):
                            maze[rand_wall[0]+1][rand_wall[1]] = 'w'
                        if ([rand_wall[0]+1, rand_wall[1]] not in walls):
                            walls.append([rand_wall[0]+1, rand_wall[1]])
                    if (rand_wall[1] != 0):
                        if (maze[rand_wall[0]][rand_wall[1]-1] != 'c'):
                            maze[rand_wall[0]][rand_wall[1]-1] = 'w'
                        if ([rand_wall[0], rand_wall[1]-1] not in walls):
                            walls.append([rand_wall[0], rand_wall[1]-1])
                    if (rand_wall[1] != width-1):
                        if (maze[rand_wall[0]][rand_wall[1]+1] != 'c'):
                            maze[rand_wall[0]][rand_wall[1]+1] = 'w'
                        if ([rand_wall[0], rand_wall[1]+1] not in walls):
                            walls.append([rand_wall[0], rand_wall[1]+1])

                for wall in walls:
                    if (wall[0] == rand_wall[0] and wall[1] == rand_wall[1]):
                        walls.remove(wall)

                continue

        if (rand_wall[1] != width-1):
            if (maze[rand_wall[0]][rand_wall[1]+1] == 'u' and maze[rand_wall[0]][rand_wall[1]-1] == 'c'):
                s_cells = surroundingCells(rand_wall)
                if (s_cells < 2):
                    maze[rand_wall[0]][rand_wall[1]] = 'c'

                    if (rand_wall[1] != width-1):
                        if (maze[rand_wall[0]][rand_wall[1]+1] != 'c'):
                            maze[rand_wall[0]][rand_wall[1]+1] = 'w'
                        if ([rand_wall[0], rand_wall[1]+1] not in walls):
                            walls.append([rand_wall[0], rand_wall[1]+1])
                    if (rand_wall[0] != height-1):
                        if (maze[rand_wall[0]+1][rand_wall[1]] != 'c'):
                            maze[rand_wall[0]+1][rand_wall[1]] = 'w'
                        if ([rand_wall[0]+1, rand_wall[1]] not in walls):
                            walls.append([rand_wall[0]+1, rand_wall[1]])
                    if (rand_wall[0] != 0):
                        if (maze[rand_wall[0]-1][rand_wall[1]] != 'c'):
                            maze[rand_wall[0]-1][rand_wall[1]] = 'w'
                        if ([rand_wall[0]-1, rand_wall[1]] not in walls):
                            walls.append([rand_wall[0]-1, rand_wall[1]])

                for wall in walls:
                    if (wall[0] == rand_wall[0] and wall[1] == rand_wall[1]):
                        walls.remove(wall)

                continue

        for wall in walls:
            if (wall[0] == rand_wall[0] and wall[1] == rand_wall[1]):
                walls.remove(wall)

    # Convert remaining unvisited cells to walls
    for i in range(0, height):
        for j in range(0, width):
            if (maze[i][j] == 'u'):
                maze[i][j] = 'w'

    # Connect the entrance and exit of the maze
    for i in range(0, width):
        if (maze[1][i] == 'c'):
            maze[0][i] = 'c'
            break

    for i in range(width-1, 0, -1):
        if (maze[height-2][i] == 'c'):
            maze[height-1][i] = 'c'
            break

    return maze

# A* search algorithm
def astar_search(maze, start, end):
    # Heuristic function for A*
    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    open_list = []
    heapq.heappush(open_list, (0, start))
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0

    while open_list:
        current_cost, current_node = heapq.heappop(open_list)

        if current_node == end:
            break

        for next_node in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            new_node = (current_node[0] + next_node[0], current_node[1] + next_node[1])
            new_cost = cost_so_far[current_node] + 1

            if new_node[0] < 0 or new_node[0] >= len(maze) or new_node[1] < 0 or new_node[1] >= len(maze[0]) or maze[new_node[0]][new_node[1]] == 'w':
                continue

            if new_node not in cost_so_far or new_cost < cost_so_far[new_node]:
                cost_so_far[new_node] = new_cost
                priority = new_cost + heuristic(end, new_node)
                heapq.heappush(open_list, (priority, new_node))
                came_from[new_node] = current_node

    path = []
    current = end
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()
    return path, cost_so_far[end]

# Breadth-First Search algorithm
def bfs(maze, start, end):
    # Function to check if a state is the goal state
    def is_goal(state):
        return state == end

    # Function to generate next possible states
    def next_states(state):
        result = []
        for move in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            new_pos = (state[0] + move[0], state[1] + move[1])
            if 0 <= new_pos[0] < len(maze) and 0 <= new_pos[1] < len(maze[0]) and maze[new_pos[0]][new_pos[1]] != 'w':
                result.append(new_pos)
        return result

    start_time = time.time()
    result = bfs_search(start, is_goal, next_states)
    end_time = time.time()
    return result, end_time - start_time

# Depth-First Search algorithm
def dfs(maze, start, end):
    # Function to check if a state is the goal state
    def is_goal(state):
        return state == end

    # Function to generate next possible states
    def next_states(state):
        result = []
        for move in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            new_pos = (state[0] + move[0], state[1] + move[1])
            if 0 <= new_pos[0] < len(maze) and 0 <= new_pos[1] < len(maze[0]) and maze[new_pos[0]][new_pos[1]] != 'w':
                result.append(new_pos)
        return result

    start_time = time.time()
    result = dfs_search(start, is_goal, next_states)
    end_time = time.time()
    return result, end_time - start_time

# Breadth-First Search algorithm implementation
def bfs_search(start_state, is_goal, next_states):
    visited = set()
    queue = [[start_state]]

    while queue:
        path = queue.pop(0)
        state = path[-1]
        if state in visited:
            continue
        visited.add(state)
        if is_goal(state):
            return path
        for next_state in next_states(state):
            new_path = list(path)
            new_path.append(next_state)
            queue.append(new_path)
    return None

# Depth-First Search algorithm implementation
def dfs_search(start_state, is_goal, next_states):
    stack = [start_state]
    visited = set()

    while stack:
        state = stack.pop()
        if state in visited:
            continue
        if is_goal(state):
            return True
        visited.add(state)
        for next_state in next_states(state):
            stack.append(next_state)

    return False

# Main function to run the program
def main():
    height = int(input("Enter maze height: "))
    width = int(input("Enter maze width: "))
    difficulty = float(input("Enter difficulty (0-1): "))

    # Generate the maze
    maze = generate_maze(height, width)
    print("Original Maze:")
    for row in maze:
        print(' '.join(row))
    print()

    start = (1, 1)
    end = (height - 2, width - 2)

    # Perform A* search
    path, cost = astar_search(maze, start, end)
    print("A* Search:")
    print("Maze:")
    for row in maze:
        print(' '.join(row))
    print("Path cost:", cost)
    print("Path:", path)
    print()

    # Perform Breadth-First Search
    path, time_taken = bfs(maze, start, end)
    print("Breath-First Search:")
    print("Maze:")
    for row in maze:
        print(' '.join(row))
    print("Time taken:", time_taken)
    print("Path:", path)
    print()

    # Perform Depth-First Search
    path, time_taken = dfs(maze, start, end)
    print("Depth-First Search:")
    print("Maze:")
    for row in maze:
        print(' '.join(row))
    print("Time taken:", time_taken)
    print("Path:", path)

if __name__ == "__main__":
    main()
