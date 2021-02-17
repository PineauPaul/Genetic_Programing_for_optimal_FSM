import pandas as pd
import operator
from random import randint, random, seed
from copy import deepcopy
import numpy as np
import matplotlib.pyplot as plt

# Read data
#add 1...10 value to the possibility of leafs
data['1'] = 1
data['2'] = 2
data['3'] = 3
# data['4'] = 4
# data['5'] = 5
# data['6'] = 6
# data['7'] = 7
# data['8'] = 8
# data['9'] = 9
# data['10'] = 10


def list_prog(node,value):
    if "leaf" not in node:
        if node["feature_name"] != "windspeed":
            return float(node["feature_name"])
        else:
            return value
    return node["func"](*[list_prog(c,value) for c in node["leaf"]])




def print_prog(node):
    #if there is no leaf
    if "leaf" not in node:
        # print("ok")
        # print(node["feature_name"])
        # print("ok2")
        return node["feature_name"]
    return node["format_str"].format(*[print_prog(c) for c in node["leaf"]])


def evaluate(node, row):
    if "leaf" not in node:
        #row contain all leaf from the tree here
        return row[node["feature_name"]]
    return node["func"](*[evaluate(c, row) for c in node["leaf"]] )


def safe_div(a, b):
    return a / b if b else a

def cosinus(a):
    return np.cos(a)

def sinus(a):
    return np.sin(a)

operations = (
    #creations of the basic operations

    {"func": operator.add, "arg_count": 2, "format_str": "({} + {})"},
    {"func": operator.sub, "arg_count": 2, "format_str": "({} - {})"},
    {"func": operator.mul, "arg_count": 2, "format_str": "({} * {})"},
    {"func": safe_div, "arg_count": 2, "format_str": "({} / {})"},
    {"func": operator.neg, "arg_count": 1, "format_str": "-({})"},

    {"func" :sinus,"arg_count" : 1,"format_str" : "sin({})"},
    {"func" :cosinus,"arg_count" : 1,"format_str" : "cos({})"},

)


def random_prog(depth):
    #create random program recursively
    if randint(0, 10) >= depth * 2:
        op = operations[randint(0, len(operations) - 1)]
        return {
            "func": op["func"],
            "leaf": [random_prog(depth + 1) for _ in range(op["arg_count"])],
            "format_str": op["format_str"],
        }
    else:
        return {"feature_name": data.columns[randint(0, data.shape[1] - 1)]}


POP_SIZE = 15
population = [random_prog(0) for i in range(POP_SIZE)]



def select_random_node(selected, parent, depth):
    if "leaf" not in selected:
        return parent
    if randint(0, 10) < 2*depth:
        return selected
    child_count = len(selected["leaf"])
    return select_random_node(
        selected["leaf"][randint(0, child_count - 1)],
        selected, depth+1)




def mutation(selected):
    #mutation
    offspring = deepcopy(selected)
    mutate_point = select_random_node(offspring, None, 0)
    child_count = len(mutate_point["leaf"])
    mutate_point["leaf"][randint(0, child_count - 1)] = random_prog(0)
    return offspring


def crossover(selected1, selected2):
    #crossover
    offspring = deepcopy(selected1)
    #select two nodes
    xover_point1 = select_random_node(offspring, None, 0)
    xover_point2 = select_random_node(selected2, None, 0)
    #get size
    child_count = len(xover_point1["leaf"])
    #replace part of tree by another
    xover_point1["leaf"][randint(0, child_count - 1)] = xover_point2
    return offspring




Evolution_size = 5

def get_random_parent(population, fitness):
    # randomly select population members for the tournament
    tournament_members = [
        randint(0, POP_SIZE - 1) for k in range(Evolution_size)]
    # select tournament member with best fitness
    member_fitness = [(fitness[i], population[i]) for i in tournament_members]
    return min(member_fitness, key=lambda x: x[0])[1]

prob_crossover = 0.5#propability of crossover else mutation


def get_offspring(population, fitness):
    parent1 = get_random_parent(population, fitness)
    if random() > prob_crossover:
        parent2 = get_random_parent(population, fitness)
        return crossover(parent1, parent2)
    else:
        return mutation(parent1)


def node_count(x):
    if "leaf" not in x:
        return 1
    return sum([node_count(c) for c in x["leaf"]])


def compute_fitness(prediction):
    mse = ((pd.Series(prediction) - target) ** 2).mean()
    return mse

MAX_GENERATIONS = 3 #we stop after max 10 generations

global_best = float("inf") #infinite fitness for the first value

for gen in range(MAX_GENERATIONS):
    fitness = []
    for prog in population:
        prediction = [evaluate(prog, row) for i, row in data.iterrows()]
        score = compute_fitness(prediction)
        fitness.append(score)
        if score < global_best:
            global_best = score
            best_pred = prediction
            best_prog = prog
    print("Generation: %d\nBest Score: %.2f\nMedian score: %.2f\nBest program: %s\n" % (gen,global_best,pd.Series(fitness).median(),print_prog(best_prog),))

    population = [get_offspring(population, fitness) for i in range(POP_SIZE)]

print("Best score: %f" % global_best)
print("Best program: %s" % print_prog(best_prog))


L=[]
for k in range(len(wind)):
    L.append(list_prog(best_prog,wind[k]))


plt.plot(L)
plt.show()