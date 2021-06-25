import heapq
from collections import deque

import numpy as np


class Puzzle:
    state = None
    zero = None
    operator = None
    depth = 0
    cost = 0
    parent = None

    def __init__(self, pos, parent=None, operator=None, depth=0):
        self.state = pos
        self.zero = self.find_zero()
        self.depth = depth
        self.operator = operator
        self.cost = self.depth
        self.parent = parent

    def __lt__(self, other):
        self.cost = self.cost + self.manhattan()
        other.cost = other.cost + other.manhattan()
        if self.cost != other.cost:
            return self.cost < other.cost
        else:
            precedence = {None: -1, 'up': 0, 'down': 1, 'left': 2, 'right': 3}
            return precedence[self.operator] < precedence[other.operator]

    def find_zero(self):
        for i in range(9):
            if self.state[i] == 0:
                zero = i
                return zero

    def swap(self, dir):
        new_state = np.array(self.state)
        zero = self.zero
        if dir == 'up':
            temp = new_state[zero]
            new_state[zero] = new_state[zero - 3]
            new_state[zero - 3] = temp
            return new_state
        elif dir == 'down':
            temp = new_state[zero]
            new_state[zero] = new_state[zero + 3]
            new_state[zero + 3] = temp
            return new_state
        elif dir == 'left':
            temp = new_state[zero]
            new_state[zero] = new_state[zero - 1]
            new_state[zero - 1] = temp
            return new_state
        elif dir == 'right':
            temp = new_state[zero]
            new_state[zero] = new_state[zero + 1]
            new_state[zero + 1] = temp
            return new_state

    def up(self):
        if self.zero < 3:
            return self
        else:
            return Puzzle(np.array(self.swap('up')), self, "up", self.depth + 1)

    def down(self):
        if self.zero > 5:
            return self
        else:
            return Puzzle(np.array(self.swap('down')), self, "down", self.depth + 1)

    def left(self):
        if (self.zero % 3) == 0:
            return self
        else:
            return Puzzle(np.array(self.swap('left')), self, "left", self.depth + 1)

    def right(self):
        if (self.zero + 1) % 3 == 0:
            return self
        else:
            return Puzzle(np.array(self.swap('right')), self, "right", self.depth + 1)

    def index(self, pos):
        index = np.array(range(9))
        for x, y in enumerate(pos):
            index[y] = x
        return index

    def children(self):
        children = []
        children.append(self.up())
        children.append(self.down())
        children.append(self.left())
        children.append(self.right())
        return children

    def goal(self):
        if np.array_equal(self.state, np.arange(9)):
            return True
        else:
            return False

    def sol_steps(self):
        current = self
        parents = []
        while current.parent is not None:
            parents.append(current)
            current = current.parent
        parents.append(current)
        return self.show_steps(parents)

    def show_steps(self, steps):
        for i in reversed(steps):
            print(i.state, end="  ")
            print(i.operator)
        cost = len(steps) - 1
        print("cost = ", cost)
        print("depth = ", self.depth)

    def manhattan(self):
        pos = self.index(self.state)
        goal = self.index(np.arange(9))
        total = sum((abs(pos // 3 - goal // 3) + abs(pos % 3 - goal % 3))[1:])
        return total


class Solution:
    sol = None

    def bfs(self, pos):
        frontier = deque()
        frontier.append(pos)
        froxplored = set()
        while frontier:
            board = frontier.popleft()
            froxplored.add(board)
            if board.goal():
                self.soln = board
                return self.soln.sol_steps()
            for neighbor in board.children():
                if tuple(neighbor.state) not in froxplored:
                    frontier.append(neighbor)
                    froxplored.add(tuple(neighbor.state))
        return None

    def dfs(self, pos):
        frontier = []
        frontier.append(pos)
        froxplored = set()
        while frontier:
            board = frontier.pop()
            froxplored.add(board)
            if board.goal():
                self.soln = board
                return self.soln.sol_steps()
            for neighbor in board.children():
                if tuple(neighbor.state) not in froxplored:
                    frontier.append(neighbor)
                    froxplored.add(tuple(neighbor.state))
        return None

    def ast(self, puzzle):
        frontier = []
        heapq.heappush(frontier, puzzle)
        froxplored = set()
        while frontier:
            board = heapq.heappop(frontier)
            froxplored.add(tuple(board.state))
            if board.goal():
                self.soln = board
                return self.soln.sol_steps()
            for neighbor in board.children():
                if tuple(neighbor.state) not in froxplored:
                    heapq.heappush(frontier, neighbor)
                    froxplored.add(tuple(neighbor.state))
        return None


list1 = [8, 4, 6, 3, 2, 1, 7, 0, 5]
state = np.array(list1)
puzzle1 = Puzzle(state)
s = Solution()
print('in AST:')
s.ast(puzzle1)
