#!/usr/bin/env python
# -*- coding: utf-8 -*-
# solver16.py : Solve 16 puzzle problem!
# Augustine Joseph
# Course: CS-B551-Fall2017
# Based on following video
# https://www.youtube.com/watch?v=V1i7EcynYKk
# https://www.youtube.com/watch?v=sAoBeujec74
#
#
'''

Problem description: This program will solve a 15-puzzle problem and will ﬁnd a short sequence of moves that restores 
the canonical conﬁguration, given an initial board conﬁguration.

State Space: Movement of 1 to 3 tiles on a 4X4 board with one empty tile and 15 numbered tiles. All configuration of 
the board from initial state to the canonical conﬁguration.

Successor function: The successor function will move 1 to 3 tiles in such a way that the board will reach the final 
configuration.  

Goal state: Canonical conﬁguration 

Heuristic function: The number of misplaced tiles

Why they are admissible: The number of misplaced tiles, never over estimates the cost to reach the final configuration. 
So this is admissible. 

How the search algorithm works: A* search is implemented using priority que. 

Assumptions: 1 cost per move. 

'''

import sys
import numpy as np



class node:
    def __init__ (self, state, parent, move, m_index, g_of_n, h_of_n):
        self.state = state
        self.parent = parent
        self.move = move  # movements up, left, down, right
        self.m_index = m_index # Index of the moving tile
        self.g_of_n = g_of_n  # cumulative path cost to reach the current node
        self.h_of_n = h_of_n  # h(n), heuristic value of the current node

        # store node expansions

        self.child_down = None
        self.child_up = None
        self.child_left = None
        self.child_right = None

    # Checking down movement

    def chk_move_down(self):
        # position of the empty tile
        empty_position = [i[0] for i in np.where(self.state == 0)]
        if empty_position[0] == 0:
            return False
        else:
            m_idx = empty_position[1] # location of the upper tile

            next_state = self.state.copy()
            next_state[empty_position[0], empty_position[1]], next_state[empty_position[0] - 1, empty_position[1]] = next_state[empty_position[0] - 1, empty_position[1]], next_state[empty_position[0], empty_position[1]]

            return next_state, m_idx +1



    # Checking up movement

    def chk_move_up(self):

        # position of the empty tile
        empty_position = [i[0] for i in np.where(self.state == 0)]
        if empty_position[0] == 3:  # change this to 3
            return False

        else:
            m_idx = empty_position[1]  # location of the lower tile

            next_state = self.state.copy()
            next_state[empty_position[0], empty_position[1]] , next_state[empty_position[0] + 1, empty_position[1]] = next_state[empty_position[0] + 1, empty_position[1]] , next_state[empty_position[0], empty_position[1]]
            return next_state, m_idx +1


    # Checking left movement

    def chk_move_left(self):
        # position of the empty tile
        empty_position = [i[0] for i in np.where(self.state == 0)]
        if empty_position[1] == 3:  # change this to 3
            return False
        else:
            m_idx = empty_position[0]  # location of the right tile

            next_state = self.state.copy()
            next_state[empty_position[0], empty_position[1]], next_state[empty_position[0], empty_position[1] + 1] = next_state[empty_position[0], empty_position[1] + 1], next_state[empty_position[0], empty_position[1]]
            return next_state, m_idx +1


    # Checking right movement

    def chk_move_right(self):
        # position of the empty tile
        empty_position = [i[0] for i in np.where(self.state == 0)]
        if empty_position[1] == 0:
            return False
        else:
            m_idx = empty_position[0] # location of the left tile

            next_state = self.state.copy()
            next_state[empty_position[0], empty_position[1]], next_state[empty_position[0], empty_position[1] - 1] = next_state[empty_position[0], empty_position[1] - 1], next_state[empty_position[0], empty_position[1]]

            return next_state, m_idx+1

    # heuristic function: number of misplaced tiles


    def h_misplaced_cost(self, goal_state):
        num_misplaced = np.sum(self.state != goal_state) - 1  # The empty tile is not considered
        if num_misplaced > 0:
            self.h_of_n = num_misplaced
            return num_misplaced
        else:
            return 0  # if  goal state

    # If the goal node is reached, trace back to the root node and print out the results

    def print_solution_path(self):
        # Set stacks to place the trace
        state_trace = [self.state]
        move_trace = [self.move]
        m_index_trace = [self.m_index]
        g_of_n_trace = [self.g_of_n]


        # populate node information while going back up the tree

        while self.parent:
            self = self.parent
            state_trace.append(self.state)
            move_trace.append(self.move)
            m_index_trace.append(self.m_index)
            g_of_n_trace.append(self.g_of_n)


        tile_counter = 1

        current_mtr = move_trace.pop()
        current_midx = m_index_trace.pop()

        result = ''

        print(state_trace.pop())
        print('move=', current_mtr, ', total_cost=', str(g_of_n_trace.pop()), '\n')

        i=0
        while move_trace:
            current_mtr = move_trace.pop()
            if move_trace:
                next_mtr = move_trace[-1]


            else:

                 current_midx = m_index_trace.pop()
                 print(state_trace.pop())
                 i +=1
                 print('move=', current_mtr, ', total_cost=', str(i), '\n')

                 r_str = current_mtr+str(tile_counter)+str(current_midx)+" "
                 result += r_str
                 print(result)
                 break



            if (current_mtr == next_mtr):

                    state_trace.pop()
                    g_of_n_trace.pop()
                    current_midx = m_index_trace.pop()

                    tile_counter += 1
            else:

                    current_midx = m_index_trace.pop()
                    print(state_trace.pop())
                    i += 1
                    print('move=', current_mtr, ', total_cost=', str(i), '\n')

                    r_str = current_mtr + str(tile_counter) + str(current_midx)+ " "
                    result += r_str
                    tile_counter = 1



    def a_star_search(self, goal_state):

        queue = [(self, 0)]
        eval_func_queue = [self.h_of_n]  # store evaluation function
        h_cost = 0 # Initial heuristic value

        expanded = set([]) # Set of nodes once checked or expanded

        while queue:
            # sort the queue based on eval function

            queue = sorted(queue, key=lambda x: x[1])
            eval_func_queue = sorted(eval_func_queue, key = lambda x: x)
            current_node = queue.pop(0)[0]
            current_eval_func = eval_func_queue.pop(0)
            expanded.add(tuple(current_node.state.reshape(1, 16)[0]))

            if np.array_equal(current_node.state, goal_state):
                current_node.print_solution_path()

                return True


            else:

                #moving upper tile down

                if current_node.chk_move_down():
                    next_state, m_idx = current_node.chk_move_down()

                    if tuple(next_state.reshape(1, 16)[0]) not in expanded:


                        current_node.child_down = node(state=next_state, parent=current_node, move='D',
                                                     m_index = m_idx,
                                                     g_of_n=current_node.g_of_n + 1, h_of_n=h_cost)

                        h_cost = current_node.child_down.h_misplaced_cost(goal_state)

                        queue.append((current_node.child_down, current_node.g_of_n + h_cost + 1))

                        eval_func_queue.append(current_node.g_of_n + h_cost + 1)



                #moving left tile right
                if current_node.chk_move_right():
                    next_state, m_idx = current_node.chk_move_right()

                    if tuple(next_state.reshape(1, 16)[0]) not in expanded:


                        current_node.child_right = node(state=next_state, parent=current_node, move='R',
                                                       m_index=m_idx,
                                                       g_of_n=current_node.g_of_n + 1, h_of_n=h_cost)

                        h_cost = current_node.child_right.h_misplaced_cost(goal_state)


                        queue.append((current_node.child_right, current_node.g_of_n + h_cost + 1))

                        eval_func_queue.append(current_node.g_of_n + h_cost + 1)



                #moving lower tile up
                if current_node.chk_move_up():
                    next_state, m_idx = current_node.chk_move_up()

                    if tuple(next_state.reshape(1, 16)[0]) not in expanded:


                        current_node.child_up = node(state=next_state, parent=current_node, move='U',
                                                    m_index=m_idx,
                                                    g_of_n=current_node.g_of_n + 1, h_of_n=h_cost)

                        h_cost = current_node.child_up.h_misplaced_cost(goal_state)



                        queue.append((current_node.child_up, current_node.g_of_n + h_cost + 1))

                        eval_func_queue.append(current_node.g_of_n + h_cost + 1)



                #moving right tile left
                if current_node.chk_move_left():
                    next_state, m_idx = current_node.chk_move_left()

                    if tuple(next_state.reshape(1, 16)[0]) not in expanded:



                        current_node.child_left = node(state=next_state, parent=current_node, move='L',
                                                      m_index=m_idx,
                                                      g_of_n=current_node.g_of_n + 1, h_of_n=h_cost)

                        h_cost = current_node.child_left.h_misplaced_cost(goal_state)

                        queue.append((current_node.child_left, current_node.g_of_n + h_cost + 1))

                        eval_func_queue.append(current_node.g_of_n + h_cost + 1)



goal_state =np.array([1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,0]).reshape(4,4)

input_file = sys.argv[1]

#Load data from the text file
initial_state = np.loadtxt(input_file, dtype=int, )

print(initial_state[1][1])

# Initialize the root node



root_node = node(state=initial_state,parent=None,move=None,m_index=None,g_of_n=0,h_of_n=0)

#
root_node.a_star_search(goal_state)


