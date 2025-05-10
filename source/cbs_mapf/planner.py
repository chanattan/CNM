#!/usr/bin/env python3
'''
Author: Haoran Peng
Email: gavinsweden@gmail.com

An implementation of multi-agent path finding using conflict-based search
[Sharon et al., 2015]
'''
from typing import List, Tuple, Dict, Callable, Set
import multiprocessing as mp
from heapq import heappush, heappop
from itertools import combinations
from copy import deepcopy
import numpy as np

# The low level planner for CBS is the Space-Time A* planner
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from pathfinding.core.diagonal_movement import DiagonalMovement

from .constraint_tree import CTNode
from .constraints import Constraints
from .agent import Agent
from .assigner import *
class Planner:

    """
    def __init__(self, grid_size: int,
                       robot_radius: int,
                       static_obstacles: List[Tuple[int, int]]):

        self.robot_radius = robot_radius
        self.st_planner = STPlanner(grid_size, robot_radius, static_obstacles)
    """
    def __init__(self, grid: List[List[int]]):
        '''
            In this version, the grid already contains the static obstacles (1).
            The agents have their starter position (2),
            and the goals (3) are assigned with a farthest-goal algorithm.
        '''
        def transform_grid(card):
            """
                Short utility function to only consider free (1) and obstacles (0) in the map.
                Used to collaborate with the pathfinding library.
            """
            for y in range(len(card)):
                for x in range(len(card[y])):
                    obj = card[y][x]
                    if obj == 2: # Obstacle: 2 -> 0
                        card[y][x] = 0
                    else: # Anything else is an agent: x -> 1
                        card[y][x] = 1
        self.robot_radius = 10
        transform_grid(grid)
        self.grid = grid

    '''
    You can use your own assignment function, the default algorithm greedily assigns
    the closest goal to each start.
    '''
    def plan(self, starts: List[Tuple[int, int]],
                   goals: List[Tuple[int, int]],
                   assign:Callable = min_cost,
                   max_iter:int = 200,
                   max_process:int = 10,
                   debug:bool = False) -> np.ndarray:
        self.debug = debug

        # Do goal assignment
        self.agents = assign(starts, goals)

        constraints = Constraints()

        # Compute path for each agent using low level planner
        solution = dict((agent, self.calculate_path(agent, constraints)) for agent in self.agents)

        open = []
        if all(len(path) != 0 for path in solution.values()):
            # Make root node
            node = CTNode(constraints, solution)
            # Min heap for quick extraction
            open.append(node)

        manager = mp.Manager()
        iter_ = 0
        while open and iter_ < max_iter:
            iter_ += 1

            results = manager.list([])

            processes = []

            # Default to 10 processes maximum
            for _ in range(max_process if len(open) > max_process else len(open)):
                p = mp.Process(target=self.search_node, args=[heappop(open), results])
                processes.append(p)
                p.start()

            for p in processes:
                p.join()

            for result in results:
                if len(result) == 1:
                    if debug:
                        print('CBS_MAPF: Paths found after about {0} iterations'.format(4 * iter_))
                    return result[0]
                if result[0]:
                    heappush(open, result[0])
                if result[1]:
                    heappush(open, result[1])

        if debug:
            print('CBS-MAPF: Open set is empty, no paths found.')
        return np.array([])

    '''
    Abstracted away the cbs search for multiprocessing.
    The parameters open and results MUST BE of type ListProxy to ensure synchronization.
    '''
    def search_node(self, best: CTNode, results):
        agent_i, agent_j, time_of_conflict = self.validate_paths(self.agents, best)

        # If there is not conflict, validate_paths returns (None, None, -1)
        if agent_i is None:
            results.append((self.reformat(self.agents, best.solution),))
            return
        # Calculate new constraints
        if self.debug:
            print("agent i:", agent_i)
            print("agent j:", agent_j)
            print("time of conflict:", time_of_conflict)
        agent_i_constraint = self.calculate_constraints(best, agent_i, agent_j, time_of_conflict)
        agent_j_constraint = self.calculate_constraints(best, agent_j, agent_i, time_of_conflict)

        if self.debug:
            print("agent 1 constraint:", agent_i_constraint)
            print("agent 2 constraint:", agent_j_constraint)

        # Calculate new paths
        agent_i_path = self.calculate_path(agent_i,
                                           agent_i_constraint)
        agent_i_path2 = self.calculate_path(agent_i, agent_j_constraint)
        agent_j_path = self.calculate_path(agent_j,
                                           agent_j_constraint)
        agent_j_path2 = self.calculate_path(agent_j, agent_i_constraint)
        
        if self.debug:
            print("calculated path 1 for agent 1:", agent_i_path)
            print("calculated path 1 for agent 2:", agent_i_path)
            print("calculated path 2 for agent 1:", agent_i_path2)
            print("calculated path 2 for agent 2:", agent_j_path2)

        # Replace old paths with new ones in solution
        solution_i = best.solution
        solution_j = deepcopy(best.solution)
        fst = True
        if len(agent_i_path) > 0:
            solution_j[agent_i] = agent_i_path
            solution_j[agent_j] = agent_j_path2
        else:
            solution_j[agent_i] = agent_i_path2 if len(agent_i_path2) > 0 else np.array([agent_i.start])
            solution_j[agent_j] = agent_j_path if len(agent_j_path) > 0 else np.concatenate((np.array([agent_j_path2[0]]) if len(agent_j_path2) > 0 else np.array(agent_j.start), agent_j_path2), axis=0)
            fst = False
        #if len(agent_j_path) > 0:
        #    solution_j[agent_j] = agent_j_path
        #    solution_j[agent_i] = agent_i_path2
        #else:
        # Replace old paths with new ones in solution
        #solution_i = best.solution
        #solution_j = deepcopy(best.solution)
        #solution_i[agent_i] = agent_i_path
        #solution_j[agent_j] = agent_j_path

        node_i = None
        if all(len(path) != 0 for path in solution_i.values()):
            node_i = CTNode(agent_i_constraint if fst else agent_j_constraint, solution_i)

        node_j = None
        if all(len(path) != 0 for path in solution_j.values()):
            node_j = CTNode(agent_j_constraint if fst else agent_i_constraint, solution_j)

        results.append((node_i, node_j))


    '''
    Pair of agent, point of conflict
    '''
    def validate_paths(self, agents, node: CTNode):
        # Check collision pair-wise
        for agent_i, agent_j in combinations(agents, 2):
            time_of_conflict = self.collides(node.solution, agent_i, agent_j)
            # time_of_conflict=1 if there is not conflict
            if time_of_conflict == -1:
                continue
            return agent_i, agent_j, time_of_conflict
        return None, None, -1

    def collides(self, solution: Dict[Agent, np.ndarray], agent_i: Agent, agent_j: Agent) -> int:
        #for idx, (point_i, point_j) in enumerate(zip(solution[agent_i], solution[agent_j])):
            #if self.dist(point_i, point_j) > 2*self.robot_radius:
                #continue
            #return idx
        path1, path2 = solution[agent_i], solution[agent_j]
        if len(path2) < len(path1):
            path1, path2 = path2, path1
        # Bias of collision: the crossing is considered more important as an imminent collision
        # Bias of selection: the shortest path is considered with the colliding point, i.e., the agent with the longest path is the obstacle   
        # Common path
        p1, p1_n, p2, p2_n = None, None, None, None
        if np.ndim(path1) == 1:
            p1 = path1[0]
            p1_n = path1[1]
        else:
            p1 = path1[0][0]
            p1_n = path1[0][1]
        if np.ndim(path2) == 1:
            p2 = path2[0]
            p2_n = path2[1]
        else:
            p2 = path2[0][0]
            p2_n = path2[0][1]
        if np.ndim(path1) == 1 or np.ndim(path2) == 1:
            # Virtual obstacle
            if p1 == p2 and p1_n == p2_n:
                return 0 # (p1[0], p1[1])
        else:
            for i in range(len(path1)-1):
                p1 = path1[i]
                p1_n = path1[i+1]
                p2 = path2[i]
                # Virtual obstacle
                if p1[0] == p2[0] and p1[1] == p2[1]:
                    return i # (p1[0], p1[1])
                # Crossing paths
                if p1_n[0] == p2[0] and p1_n[1] == p2[1]:
                    return i # (p2[0], p2[1])
            # Check last cell which cannot be crossing path as it has already been checked
            if path1[len(path1)-1][0] == path2[len(path1)-1][0] and path1[len(path1)-1][1] == path2[len(path1)-1][1]:
                return len(path1) - 1 # (path1[len(path1)-1][0], path1[len(path1)-1][1])
        return -1

    @staticmethod
    def dist(point1: np.ndarray, point2: np.ndarray) -> int:
        return int(np.linalg.norm(point1-point2, 2))  # L2 norm

    def calculate_constraints(self, node: CTNode,
                                    constrained_agent: Agent,
                                    unchanged_agent: Agent,
                                    time_of_conflict: int) -> Constraints:
        contrained_path = node.solution[constrained_agent]
        unchanged_path = node.solution[unchanged_agent]

        pivot = unchanged_path[time_of_conflict]
        conflict_end_time = time_of_conflict
        try:
            while self.dist(contrained_path[conflict_end_time], pivot) < 1:
                conflict_end_time += 1
        except IndexError:
            pass
        return node.constraints.fork(constrained_agent, tuple(pivot.tolist()), time_of_conflict, conflict_end_time)

    def calculate_goal_times(self, node: CTNode, agent: Agent, agents: List[Agent]):
        solution = node.solution
        goal_times = dict()
        for other_agent in agents:
            if other_agent == agent:
                continue
            time = len(solution[other_agent]) - 1
            goal_times.setdefault(time, set()).add(tuple(solution[other_agent][time]))
        return goal_times

    '''
        Add constraints to the grid, physically.
    '''
    def translate_constraints(self, constraints: Dict[int, Set[Tuple[int, int]]], grid):
        '''
            The modeled constraints in CBS are time-based. In this translation, conflicts
            are considered as virtual static obstacles present on the grid.
        '''
        v_grid = deepcopy(grid)
        for s in constraints.values():
            for (x, y) in s:
                v_grid[y][x] = 0 # Obstacle
        return v_grid

    '''
    Calculate the paths for all agents with space-time constraints
    '''
    def calculate_path(self, agent: Agent, 
                       constraints: Constraints) -> np.ndarray:
        grid = Grid(height=len(self.grid),
                    width=len(self.grid[0]),
                    matrix = self.translate_constraints(constraints.setdefault(agent, dict()), self.grid),
                    inverse=False, grid_id=1)
        finder = AStarFinder(diagonal_movement = DiagonalMovement.if_at_most_one_obstacle)
        ss = list(agent.start)
        ee = list(agent.goal)
        start = grid.node(ss[0], ss[1])
        end = grid.node(ee[0], ee[1])
        path, runs = finder.find_path(start,
                                      end,
                                      grid)
        cell = lambda c : (c.x, c.y)
        path = list(map(cell, path))
        #print("----- path", path, "\n")
        return np.array(path) # np array of dimension (len(path), 2)
        #return self.st_planner.plan(agent.start, 
        #                            agent.goal, 
        #                            constraints.setdefault(agent, dict()), 
        #                            semi_dynamic_obstacles=goal_times,
        #                            max_iter=self.low_level_max_iter, 
        #                            debug=self.debug)

    '''
    Reformat the solution to a numpy array
    '''
    @staticmethod
    def reformat(agents: List[Agent], solution: Dict[Agent, np.ndarray]):
        solution = Planner.pad(solution)
        reformatted_solution = []
        for agent in agents:
            reformatted_solution.append(solution[agent])
        return np.array(reformatted_solution)

    '''
    Pad paths to equal length, inefficient but well..
    '''
    @staticmethod
    def pad(solution: Dict[Agent, np.ndarray]):
        max_ = max(len(path) for path in solution.values())
        for agent, path in solution.items():
            if len(path) == max_:
                continue
            padded = np.concatenate([path, np.array(list([path[-1]])*(max_-len(path)))])
            solution[agent] = padded
        return solution

