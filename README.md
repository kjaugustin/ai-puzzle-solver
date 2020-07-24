# 15-puzzle solver

This is a variant of the 15-puzzle, but with the following important change. Instead of sliding a
single tile from one cell into an empty cell, in this variant, either one, two, or three tiles may be slid
left, right, up or down in a single move. 


The goal is to find a short sequence of moves that restores the canonical configuration, given an initial board configuration. 
The Python program called solver16.py finds a solution to this problem efficiently using A* search. To run program on the command line enter:
```
./solver16.py [input-board-filename]
```
where input-board-filename is a text file containing a board configuration in a format like:
```
1 2 3 4
5 6 7 8
9 0 10 12
13 14 15 11
```
