# Rubik's Cube Solver

Solver for a 3x3x3 **Rubik's Cube** implementing an A* algorithm with the Manhattan Distance as heuristics.

Code for the cube adopted from Miguel Hernando: <https://github.com/mhernando/pyRubikSim>

<br>

### - Miguel Granero Ramos. 2021

<br>

## Requirements: 
As well as in Miguel Hernando's library:
```
pip install numpy
pip install matplotlib
```

or simply execute:
```
pip install -r requirements.txt
```

## How to use:
The solver is wrapped around a ``Astar`` class. It need the cube to solve as parameter when instatiating. Example code can be found inside ``rb_solver.py``.
```py
# Create a cube
cube = rb.RubCube(3)

# Random movements
m=getRandomMoves(6)
print("Initial movements")
for x in m:
    print(x)
    cube.rotate_90(x[0],x[1],x[2])
print("Close the figure to start!!")
cube.plot()

solver = Astar(cube=cube, verbose=False)
movements = solver.solve()

print("Necessary moves {}: ".format(len(movements)))
for i in movements:
    print(i.movement)
    cube.rotate_90(i.movement[0],i.movement[1],i.movement[2])
cube.plot()
```