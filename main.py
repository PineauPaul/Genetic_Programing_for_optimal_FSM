from roblib import *
import numpy as np
import matplotlib.pyplot as plt
from fsm import fsm
from riptide import Riptide
from terrain import *
from datetime import datetime


class Fsm(object):
    def __init__(self, heading_list,submarine):

        self.heading_list = heading_list
        self.isobath = -18
        self.depth = -submarine.getEchosounder()
        self.cap = 0
        self.heading_order = heading_list[0]


        self.fs = fsm([
        ("Start","0", True),
        ("0","0", self.outside, self.Continue),
        ("0","1_in", self.inside, self.DoHeading1),
        ("1_in","1_in", self.inside, self.Continue),

        ("1_in","2_out", self.outside, self.DoHeading2),
        ("2_out","2_out", self.outside, self.Continue),

		("2_out","2_in", self.inside, self.Continue),
        ("2_in","2_in", self.inside, self.Continue),


		("2_in","3_out",self.outside,self.DoHeading3),
        ("3_out","3_out", self.outside, self.Continue),

		("3_out","3_in", self.inside, self.Continue),
        ("3_in","3_in", self.inside, self.Continue),

	    ("3_in","1_out", self.outside, self.DoHeading1),
        ("1_out","1_out", self.outside, self.Continue),
        ("1_out", "1_in", self.inside, self.Continue)])



    def outside (self,fss):
        #print("dans la fonction outside")
        return (self.depth <= self.isobath)

    def inside (self,fss):
        #print("dans la fonction inside")
        return (self.depth > self.isobath)



    def DoHeading1(self,fss,value):

        self.heading_order = self.heading_list[0]

    def DoHeading2(self, fss, value):

        self.heading_order = self.heading_list[1]

    def DoHeading3(self, fss, value):

        self.heading_order = self.heading_list[2]


    def Continue (self,fss,value):
        self.heading_order = self.heading_order


def crossover(l1,l2):
    """l1 and l2 are sorted """
    l = []
    for k in range(len(l1)):
        l.append((l1[k] + l2[k])/2)
    return l

def mutation(l1):
    k = np.random.randint(0,len(l1))#choose a random heading to change
    #Add condition for stability ?
    l1[k] = l1[k] + np.random.uniform(-0.5,0.5)
    return l1

def random_headings(n,length=3):
    L = []
    for k in range(n):
        temp = []
        for j in range(length):
            temp.append(np.random.uniform(0,1)*2*np.pi)

        temp.sort()
        L.append(temp)

    print('taille population:',len(L))
    return L


def is_stable(heading):
    length = len(heading)
    return (sum(heading) <= np.pi*length)


def valid_line(w1,w2,submarine):
    # global Gpos

    xs, ys, zs = submarine.X[0], submarine.X[1], submarine.X[2]
    pos = np.array((xs,ys,zs)).flatten()
    p = np.vdot((np.array(w2) - np.array(w1)),(np.array(pos) - np.array(w2)))

    return p>=0

def valid_waypoint(mat):
    for k in range(len(mat)):
        if sum(mat[k]) < 2 :
            return False
    for k in range(len(mat[0])):
        if sum(mat[:,k]) < 2:
            return False
    return True


def compute_time(fsm,submarine,waypoints,waypoint_init):
    t_tot = 0
    is_valid = False

    dt = 0.1
    tmax = 50

    mat_deja_vu = np.zeros((len(waypoints) + 1, len(waypoints) + 1))  # point initial a prendre en compte
    mat_deja_vu += np.diag([1 for k in range(len(waypoints) + 1)])

    speed = 3
    depth = 2

    fsm.fs.start("Start")
    wtot = waypoints
    wtot.append(waypoint_init)
    n_way = len(wtot)
    x = 0
    y = 0
    while t < tmax:
        euler_angles = submarine.getMagnetometer()
        auv_depth = submarine.getDepthmeter()

        fsm.fs.currentState = fsm.fs.event("")[0]


        u = submarine.control(speed, fsm.heading_order, depth, euler_angles, auv_depth)
        dX = submarine.evolX(u)
        submarine.X = submarine.X + dt * dX

        #erreur car terrain trop petit
        x = submarine.X[0]
        y = submarine.X[1]
        if x < 0 or x > 50 or y < 0 or y > 50 :
            return np.float('inf')

        fsm.depth = -submarine.getEchosounder()

        t_tot += dt

        #check validity
        for k in range(n_way):
            for l in range(n_way):
                if valid_line(wtot[k],wtot[l],submarine):
                    mat_deja_vu[k,l] = 1

        if valid_waypoint(mat_deja_vu):
            is_valid = True
            return t_tot

    if not is_valid:
        return float('inf')
    else:
        return t_tot

def compute_optimal_time(waypoints,waypoint_init,speed):
    """"estimation grossière du temps parfait pour parcourir tous les waypoints"""
    d_tot =  np.sqrt((waypoints[0][0] - waypoint_init[0])**2 + (waypoints[0][1] - waypoint_init[1])**2 + (waypoints[0][2] - waypoint_init[2])**2)
    n = len(waypoints) - 1
    for k in range(1,n):
        d_tot += np.sqrt((waypoints[k+1][0] - waypoints[k][0])**2 + (waypoints[k+1][1] - waypoints[k][1])**2+(waypoints[k+1][2] - waypoints[k][2])**2 )
    d_tot += np.sqrt((waypoints[n][0] - waypoint_init[0])**2 + (waypoints[n][1] - waypoint_init[1])**2 + (waypoints[n][2] - waypoint_init[2])**2)
    return d_tot/speed


Evolution_size = 5

def get_random_parent(population, fitness,n):
    # randomly select population members for the tournament
    tournament_members = [
        np.random.randint(0, n - 1) for k in range(Evolution_size)]
    # select tournament member with best fitness
    member_fitness = [(fitness[i], population[i]) for i in tournament_members]
    return min(member_fitness, key=lambda x: x[0])[1]

prob_crossover = 0.5#propability of crossover else mutation


def get_offspring(population, fitness,n):
    parent1 = get_random_parent(population, fitness,n)
    if np.random.random() > prob_crossover:
        parent2 = get_random_parent(population, fitness,n)
        return crossover(parent1, parent2)
    else:
        return mutation(parent1)


def compute_optimal_fsm(n,submarine,waypoint,waypoint_init):
    # np.random.seed(datetime.now())
    population = random_headings(n)
    MAX_GENERATIONS = 50  # we stop after max 10 generations

    global_best = float("inf")  # infinite fitness for the first value
    target = compute_optimal_time(waypoint, waypoint_init, 3)
    best_head = [float('inf'),float('inf'),float('inf')]
    # print('optimal time : ', target)
    for gen in range(MAX_GENERATIONS):
        print('generation numéro :',gen)
        #print('taille population : ', len(population))
        fitness = []
        for headings in population:

            fsm = Fsm(headings, submarine)
            fsm.fs.start("Start")
            score = compute_time(fsm,submarine,waypoint,waypoint_init)

            fitness.append(score)
            if score < global_best:
                global_best = score

                best_head = headings

        population = [get_offspring(population, fitness,n) for i in range(n)]

    print("Best score: %f" % global_best)
    print("Best program: %s" % best_head)

    return best_head




if __name__=='__main__':

    waypoints = [[10,20,-2],[20,20,-2],[20,10,-2]]

    mat_deja_vu = np.zeros((len(waypoints)+1,len(waypoints)+1),dtype=bool) #point initial a prendre en compte
    mat_deja_vu += np.diag([True for k in range(len(waypoints)+1)])

    heading_list_test = [0,0,0]
    # Init Riptide
    waypoint_init = [10, 10, -3.5]
    speed = 5
    depth = -2

    Xinit_riptide = array([[waypoint_init[0], waypoint_init[1], waypoint_init[2],  # x, y, z
                            0.2, -0.1, -0.3,  # phi, theta, psi
                            0.5]]).T  # v




    Xt, Yt, Zt, realSize = generate_terrain_1(centre = (25,25),largeur=1500, longueur=1500,penteX=0.07,penteY=0.07,offset=-20,pas=1)
    submarine = Riptide(Xinit_riptide,(Zt,realSize))
    ### Figure initialisation
    ax = plt.axes(projection='3d')


    # fsm = Fsm(heading_list_test,submarine)
    #
    #
    # fsm.fs.start("Start")

    dt = 0.1
    t=0

    print(compute_optimal_fsm(200,submarine,waypoints,waypoint_init))
    while t < 0:


        ax.clear()
        ax.relim()

        euler_angles = submarine.getMagnetometer()
        auv_depth = submarine.getDepthmeter()

        fsm.fs.currentState = fsm.fs.event("")[0]

        # print(auv_depth)
        u = submarine.control(speed,fsm.heading_order,depth,euler_angles,auv_depth)
        dX = submarine.evolX(u)
        submarine.X = submarine.X + dt * dX
        fsm.depth = -submarine.getEchosounder()


        draw_riptide(ax, submarine.X)
        ax.plot_surface(Xt, Yt, Zt, cmap=cm.viridis, linewidth=0, antialiased=False)
        # print('depth:',fsm.depth)
        # print('heading',fsm.heading_order)
        ax.autoscale_view(True, True, True)
        ax.set_xlim(0, 50)
        ax.set_ylim(0, 50)
        ax.set_zlim(-30, 20)
        ax.set_xlabel('X axis, meters')
        ax.set_ylabel('Y axis, meters')
        ax.set_zlabel('Z axis, meters')

        # print(fsm.fs.currentState)
        t += dt
        plt.draw()
        plt.pause(0.01)
