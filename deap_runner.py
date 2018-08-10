#!/usr/bin/env python2.7
#    This file is part of DEAP.
#
#    DEAP is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as
#    published by the Free Software Foundation, either version 3 of
#    the License, or (at your option) any later version.
#
#    DEAP is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public
#    License along with DEAP. If not, see <http://www.gnu.org/licenses/>.

import array
import multiprocessing
import random
import sys
import time
import json 
import numpy as np
import os

import algorithms
from deap import base
from deap import creator
from deap import tools

from simulation_master import evaluate, ports_queue


creator.create("FitnessMax", base.Fitness, weights=(1.0,))
creator.create("Individual", array.array, typecode="f", fitness=creator.FitnessMax)

toolbox = base.Toolbox()

# Attribute generator
toolbox.register("rand_float", random.random)

# Structure initializers
toolbox.register("individual", tools.initRepeat, creator.Individual, toolbox.rand_float, 312)
toolbox.register("population", tools.initRepeat, list, toolbox.individual)

crossover = tools.cxUniform
mutation = tools.mutGaussian
select = tools.selTournament

toolbox.register("evaluate", evaluate, ports_queue=ports_queue)
toolbox.register("mate", crossover, indpb=0.1)
toolbox.register("mutate", mutation, mu=0.0, sigma=0.2, indpb=0.2)
toolbox.register("select", select, tournsize=3)

if __name__ == "__main__":

    seed = 64
    pop_size = 40
    hof_elem = 1
    ngen = 30
    cxpb=0.5
    mutpb=0.2

    random.seed(seed)

    proc_num = 8
    try:
        proc_num = int(sys.argv[1])
    except:
        pass
    
    # Process Pool of 4 workers
    pool = multiprocessing.Pool(processes=proc_num)
    toolbox.register("map", pool.map)
    
    pop = toolbox.population(n=pop_size)
    hof = tools.HallOfFame(hof_elem)
    stats = tools.Statistics(lambda ind: ind.fitness.values)
    stats.register("avg", np.mean)
    stats.register("std", np.std)
    stats.register("min", np.min)
    stats.register("max", np.max)

    simulation_name = time.time()
    algorithms.eaSimple(simulation_name, pop, toolbox, cxpb=cxpb, mutpb=mutpb, ngen=ngen, 
                        stats=stats, halloffame=hof)
    duration = (time.time() - simulation_name) / 1000.0
    try:
        cp = dict(seed=seed, pop_size=pop_size, hof_elem=hof_elem, ngen=ngen, cxpb=cxpb, mutpb=mutpb, time=duration,
            crossover=str(crossover), mutation=str(mutation), select=str(select))
        json_elem = json.dumps(cp)
        path = "/home/gazebo/Scrivania/output/{0}".format(simulation_name)
        with open(os.path.join(path, "README"), "w") as cp_file:
            cp_file.writelines(json_elem)
    except Exception as ex:
        print(ex)

    pool.close()
