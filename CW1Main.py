# -*- coding: utf-8 -*-
"""
Created on Thu Apr  8 13:43:38 2021

@author: alext
"""

import numpy as np
import pygame
import pygame.freetype
import time
import sys
import math
from queue import PriorityQueue
from collections import deque

pygame.init()

BLACK = (0,0,0) #Background
YELLOW = (255,0,0) #End
RED = (255,255,0) #Start
GRAY = (200,200,200) #Grid
WHITE = (255,255,255) #Wall
GREEN = (0, 255, 0) #Search
PURPLE = (128, 0, 128) #Boundary
BLUE = (0, 0, 255) #Path

#Dimension of game window
WIDTH = 800
HEIGHT = 1000

row_count = 20
col_count = 20

Screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Path planning")
    
class Node:
    def __init__(self, row, col, width, total_rows): #Node parameters
        self.row = row
        self.col = col
        self.x = row * width
        self.y = col * width
        self.color = BLACK
        self.neighbours = []
        self.width = width
        self.total_rows = total_rows
        self.prev = None
        self.visited = False
        self.wall = False
    
    def get_pos(self):
        return self.row, self.col
    
    #Defining colours for each node
    def is_closed(self):
        return self.color == GREEN
    
    def is_open(self):
        return self.color == PURPLE
    
    def is_barrier(self):
        return self.color == WHITE
    
    def is_start(self):
        return self.color == RED
        
    def is_end(self):
        return self.color == YELLOW
    
    def reset(self):
        self.color = BLACK
    
    def make_start(self):
        self.color = RED
    
    def make_closed(self):
        self.color = GREEN
        
    def make_open(self):
        self.color = PURPLE
        
    def make_barrier(self):
        self.color = WHITE

    def make_end(self):
        self.color = YELLOW

    def make_path(self):
        self.color = BLUE
    
    #Draw function for each node
    def draw(self, win):
        pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.width))
    
    #Checks to see if surrounding nodes are walls
    def update_neighbours(self, grid):
        self.neighbours = []
        if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].is_barrier(): # Down
            self.neighbours.append(grid[self.row + 1][self.col])
            
        if self.row > 0 and not grid[self.row - 1][self.col].is_barrier(): # Up
            self.neighbours.append(grid[self.row - 1][self.col])
            
        if self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_barrier(): # Right
            self.neighbours.append(grid[self.row][self.col + 1])
            
        if self.col > 0 and not grid[self.row][self.col - 1].is_barrier(): # Left
            self.neighbours.append(grid[self.row][self.col - 1]) 
        
        
    def __It__(self, other):
        return False

    
def heuristic(p1, p2): #'h' sum for the A* algorithm
	x1, y1 = p1
	x2, y2 = p2
	return abs(x1 - x2) + abs(y1 - y2)


#Functions for creating the grid
def make_grid(rows, width):
    grid = []
    gap = width // rows
    for i in range(rows):
    	grid.append([])
    	for j in range(rows):
    		node = Node(i, j, gap, rows)
    		grid[i].append(node)
            
    return grid


def draw_grid(win, rows, width):
	gap = width // rows
	for i in range(rows):
		pygame.draw.line(win, GRAY, (0, i * gap), (width, i * gap))
		for j in range(rows):
			pygame.draw.line(win, GRAY, (j * gap, 0), (j * gap, width))
 

def draw(win, grid, rows, width):
	win.fill(BLACK)
    
	for row in grid:
		for spot in row:
			spot.draw(win)

	draw_grid(win, rows, width)
	pygame.display.update()

#Handles mouse inputs    
def get_clicked_pos(pos, rows, width):
	gap = width // rows
	y, x = pos

	row = y // gap
	col = x // gap

	return row, col

#Displays the text 
def text_handler(text, font):
    textSurf = font.render(text, True, WHITE)
    return textSurf, textSurf.get_rect()


def text_display(text, x, y):
    myfont = pygame.font.Font('freesansbold.ttf', 20)    
    textSurface, textRect = text_handler(text, myfont)
    textRect.center = ((x),(y))
    Screen.blit(textSurface,textRect)
    
    pygame.display.update()  

#A* Algorithm   
def aStar(draw, grid, start, end):
    count = 0
    open_set = PriorityQueue()
    open_set.put((0, count, start))
    came_from = {}
    g_score = {spot: float("inf") for row in grid for spot in row}
    g_score[start] = 0
    f_score = {spot: float("inf") for row in grid for spot in row}
    f_score[start] = heuristic(start.get_pos(), end.get_pos())

    open_set_hash = {start}

    while not open_set.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.init()

        current = open_set.get()[2]
        open_set_hash.remove(current)
        
        if current == end: #builds the path
            reconstruct_path(came_from, start, end, draw)     
            end.make_end()
            start.make_start()
            return True

        for neighbour in current.neighbours: #searches the grid with hueristic
            temp_g_score = g_score[current] + 1

            if temp_g_score < g_score[neighbour]:
                came_from[neighbour] = current
                g_score[neighbour] = temp_g_score
                f_score[neighbour] = temp_g_score + heuristic(neighbour.get_pos(), end.get_pos())
                if neighbour not in open_set_hash:
                    count += 1
                    open_set.put((f_score[neighbour], count, neighbour))
                    open_set_hash.add(neighbour)
                    neighbour.make_open()

        draw()

        if current != start:
            current.make_closed()

    return False

def reconstruct_path(came_from, start, current, draw):
	while current in came_from:
		current = came_from[current]
		current.make_path()
		draw()

#Dijkstra algorithm    
def dijkstra(draw, grid, start, end):
    count = 0
    open_set = PriorityQueue()
    open_set.put((0, count, start))
    came_from = {}
    g_score = {spot: float("inf") for row in grid for spot in row}
    g_score[start] = 0
    f_score = {spot: float("inf") for row in grid for spot in row}
    f_score[start] = 0

    open_set_hash = {start}

    while not open_set.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.init()

        current = open_set.get()[2]
        open_set_hash.remove(current)
        
        if current == end: #builds the path
            reconstruct_path(came_from, start, end, draw)     
            end.make_end()
            start.make_start()
            return True

        for neighbour in current.neighbours: #seaches the grid with no hueristic 
            temp_g_score = g_score[current] + 1

            if temp_g_score < g_score[neighbour]:
                came_from[neighbour] = current
                g_score[neighbour] = temp_g_score
                f_score[neighbour] = temp_g_score
                if neighbour not in open_set_hash:
                    count += 1
                    open_set.put((f_score[neighbour], count, neighbour))
                    open_set_hash.add(neighbour)
                    neighbour.make_open()

        draw()

        if current != start:
            current.make_closed()

    return False

def main(win, width):
    
    ROWS = 50 #size of gried
    grid = make_grid(ROWS, width)
    

    start = None
    end = None

    run = True
    while run:
        draw(win, grid, ROWS, width)
        text_display('A* Algorithm: Press 1               Dijkstra Alogorithm: Press 2', 400, 850)
        text_display('Left Click to add nodes    Right cick to delete nodes    Press SPACEBAR to clear', 400, 920)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
                
            if pygame.mouse.get_pressed()[0]: #add node to grid
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, width)            
                node = grid[row][col]
                if not start and node != end: #Prioritise start node, then end node and after wall nodes
                    start = node
                    start.make_start()

                elif not end and node != start:
                    end = node
                    end.make_end()
                    
                elif node != end and node != start:
                    node.make_barrier()
                    
            elif pygame.mouse.get_pressed()[2]: #delete node from grid
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, width)
                node = grid[row][col]
                node.reset()
                if node == start:
                    start = None
                elif node == end:
                    end = None
                    
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_1 and start and end: #start A* algorithm
                    for row in grid:
                        for node in row:
                            node.update_neighbours(grid)
                            
                    aStar(lambda: draw(win, grid, ROWS, width), grid, start, end)
                                        
                
                if event.key == pygame.K_2 and start and end: #Start Dijkstra algorithm
                    for row in grid:
                        for node in row:
                            node.update_neighbours(grid)
                            
                    dijkstra(lambda: draw(win, grid, ROWS, width), grid, start, end)
        
                if event.key == pygame.K_SPACE: #Clear the grid
                    start = None
                    end = None
                    grid = make_grid(ROWS, width)
                
    pygame.quit()

main(Screen, WIDTH)
    
