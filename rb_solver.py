#####################
#
#           
#         -- RUBIKS CUBE SOLVER --
#           ---------------------
#
#       A* solver for a Rubik's Cube
#
#####
#
#### Author
#
#       Miguel Granero Ramos - https://github.com/Migran99
#
#       - Rubik's cube library by Miguel Hernando (UPM). https://github.com/mhernando/PyRubikSim
#
#######################

import rub_cube as rb
import math
import random

'''
Rubik's cube solver.
Instantiate a solver with the class Astar(cube) passing as argument the rubik's cube you want to solve.
Then call the function solver.solve() and it will be solved automatically.
- The time needed to solve it increases exponentially with the number of movements done to unorganize the cube. 
- Only tried with a max. of 5 moves. (6 and 7 were also tried and solved but they took a considerably amount of time)
'''

class Node:
    '''
    Node for the A* solver. It holds the position, parent and costs. As well as the movement that led to this state.
    '''
    def __init__(self, cube, parent, g,h, movement):
        self.cube = cube
        self.position = cube.get_State()
        self.parent = parent
        self.movement = movement
        self.g = g # Distance to start node
        self.h = h # Heuristic distance to goal
        self.f = 0 # Total cost
        self.update()
    
    # Compare nodes
    def __eq__(self, other):
        return self.position == other.position
    def __lt__(self, other):
         return self.f < other.f
    def __gt__(self, other):
         return self.f > other.f
    
    # Representation and print of the node
    # the Node is represented just by the position, not the costs.
    # It is possible to check for the node in a list even if it has different costs
    def __repr__(self):
        return self.position
    def __str__(self):
        return str(self.position)

    # Calculat the costs given a parent and a given heuristic cost
    def calcCosts(self, parent, h):
        try:
            g = parent.g + 1
        except:
            g = 1
        
        f = g + h
        return g,h,f

    # Update values
    def update(self):
        self.g, self.h, self.f = self.calcCosts(self.parent,self.h)

    # Solve a new parent situation. If the new parent grant the node with a lower cost, update it.
    def newParent(self,parent):
        _,_,f = self.calcCosts(parent,self.h)
        if(f < self.f):
            print("New parent!!")
            self.parent = parent
            self.update()



class Astar:
    '''
    A* solver for the rubiks cube.
    '''
    
    # Variables
    visitedNodes = []
    Q = []
    cube = rb.RubCube(3)

    ###########
    # Aux functions
    ###########
    def totuple(self,a):
        try:
            return tuple(self.totuple(i) for i in a)
        except:
            return a
    def flatten(self,data):
        if isinstance(data, tuple):
            for x in data:
                yield from self.flatten(x)
        else:
            yield data

    ###########
    # --------------
    ###########

    ###########
    # Heuristic functions
    ###########
    def euclidianDist(self,a,b):
        # Do not use, it is not a valid heuristic function
        A = tuple(self.flatten(a))
        B = tuple(self.flatten(b))
        print(A)
        sume = 0.0
        for i,(x,y) in enumerate(zip(A,B)):
            if(i % 5 != 0):
                sume += math.pow(1- int(x==y),2) 
        
        return math.floor(math.sqrt(sume)/8)

    def manhattanDist(self,a,b):
        A = tuple(self.flatten(a))
        B = tuple(self.flatten(b))
        sume = 0.0
        for i,(x,y) in enumerate(zip(A,B)):
            if(i % 5 != 0): # only the borders and corners, not centers
                #print("{}: {} - {}".format(i,x,y))
                sume += abs(1- int(x==y))  

        return math.floor(sume / 8) # Divide by 8 because one movement only moves 8 pieces of the rubik's cube.

    ###########
    # --------------
    ###########

    ###########
    # Solver functions
    ###########
    def getNextPositions(self, cube,parent, goal):
        '''
        Get the next valid positions according to the possible movements.
        - Movements: 3 (axis) * 3 (layers) * 2 (ways) = 18
        Only contemplated sigle moves, since the more complex moves can be formed by single moves.
        '''
        positions = []
        movements = ['x','y','z']
        ways = [-1,1]
        for a in movements:
            for n in range(3):
                for r in ways:
                    aux = rb.RubCube(3)
                    aux.set_State(cube.get_State())
                    aux.rotate_90(a,n,r)
                    # Node for the position. The G cost is always 1 greater than the parent cost.
                    N = Node(aux,parent,parent.g+1,self.Hfunction(self,aux.get_State(), goal.position),[a,n,r])
                    positions.append(N)
        return positions
    
    def getBestInQueue(self):
        '''
        Get the best node in the queue according to the F cost.
        '''
        #print("Q size: ",len(self.Q))
        best = self.Q[0]
        best_index = 0
        if(len(self.Q) > 1):
            for i,n in enumerate(self.Q):
                if n < best:
                    best = n
                    best_index = i
                #print("{} : g({}), h({}), f({})".format(i,n.g,n.h,n.f))
        self.Q.pop(best_index) # Remove the selected node from the queue
        #print("Best found on {}".format(best_index, best))
        return best

    def isNodeVisited(self, node):
        return node in self.visitedNodes

    
    ###########
    # --------------
    ###########

    ###########
    # Init
    ###########
    HfunctionDict = {'euclidian': euclidianDist, 'manhattan': manhattanDist}
    def __init__(self,cube,h='manhattan', g=0, verbose = False):
        self.Hfunction = self.HfunctionDict[h] # It is possible to change between heuristic functions, but only the implemented Manhattan is valid.
        self.gFunction = g
        self.cube.set_State(cube.get_State())
        self.verbose = verbose # Bool ->  Print more info about the iterations

    ###########
    # --------------
    ###########

    ###########
    # Solver function
    ###########
    def solve(self, goal = Node(rb.RubCube(3), None, 0,0,None)):
        '''
        A* main solver function.
        '''

        # Initialization 
        first = Node(self.cube,None,0,self.Hfunction(self,self.cube.get_State(),goal.position),['x',0,0])
        if(self.verbose):
            print("Starting in:", first)
        self.visitedNodes.append(first)
        self.Q.append(first)
        moves = [] 
        finished = False

        # Main loop
        while len(self.Q) > 0 and not finished:
            x = self.getBestInQueue()
            self.cube.set_State(x.position) # Update the cube to get next positions
            if(self.verbose):
                print("Checking: ", x)

            # Goal is found    
            if(x == goal):
                print("FINISHED")
                finished = True
                p = x.parent
                while not (p is None): # Until we reach the first Node
                    moves.insert(0,x) # Add the movement at the beginning
                    x = p
                    p = x.parent
                    
            else:
                U = self.getNextPositions(self.cube,x,goal) # Get the next movements.
                #  U in literature represents the possible actions. Here the next nodes (states)
                for u in U:
                    if(not self.isNodeVisited(u)):
                        self.Q.append(u)
                        self.visitedNodes.append(u)
                    else: # It was already in the list, solve the problem
                        u.newParent(x)
            if(self.verbose):
                print("\n\n")
        
        return moves

    ###########
    # --------------
    ###########

def getRandomMoves(N):
    '''
    Function to get a set of random (single) movements.
    '''
    m = []
    for i in range(N):
        a = random.choice( ['x','y','z'])
        n = random.randint(0,2)
        r = random.choice([-1,1])
        m.append([a,n,r])
    return m
    


if(__name__=="__main__"):
    # Create a cube
    cube = rb.RubCube(3)

    # Random movements
    m=getRandomMoves(5)
    print("Initial movements")
    for x in m:
        print(x)
        cube.rotate_90(x[0],x[1],-x[2])
    print("Close the figure to start!!")
    cube.plot()

    solver = Astar(cube=cube, verbose=False)
    movements = solver.solve()

    print("Necessary moves {}: ".format(len(movements)))
    for i in movements:
        print(i.movement)
        cube.rotate_90(i.movement[0],i.movement[1],i.movement[2])
    cube.plot()