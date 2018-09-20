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

import pickle
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
toolbox.register("individual", tools.initRepeat, creator.Individual, toolbox.rand_float, 192)
toolbox.register("population", tools.initRepeat, list, toolbox.individual)

crossover = tools.cxUniform
mutation = tools.mutGaussian
select = tools.selBest

toolbox.register("evaluate", evaluate, ports_queue=ports_queue)
toolbox.register("mate", crossover, indpb=0.6) #0.6
toolbox.register("mutate", mutation, mu=0.0, sigma=0.3, indpb=0.3)#0.0 0.2 0.3
toolbox.register("select", select)

def main():

    MU = 100
    LAMBDA = 200 

    seed = 64
    pop_size = MU
    hof_elem = 1
    ngen = 300
    cxpb=0.6
    mutpb=0.4

    random.seed(seed)

    proc_num = 16
    try:
        proc_num = int(sys.argv[1])
    except:
        pass
    
    try:
    	gen_res = sys.argv[2]
    	with open(gen_res, "rb") as snap_file:
    		snap = pickle.load(snap_file)
    		pop = snap["population"]
    		hof = snap["halloffame"]
    		random.setstate(snap["rndstate"])
    		print("loaded checkpoint {0}".format(gen_res))
    except Exception as ex:
    	print(ex)
    	pop = toolbox.population(n=pop_size)
    	hof = tools.HallOfFame(hof_elem)
    	print("generating new population...")

    # Process Pool of 4 workers
    pool = multiprocessing.Pool(processes=proc_num)
    toolbox.register("map", pool.map)
    
    stats = tools.Statistics(lambda ind: ind.fitness.values)
    stats.register("avg", np.mean)
    stats.register("std", np.std)
    stats.register("min", np.min)
    stats.register("max", np.max)

    simulation_name = time.time()
    
    algorithms.eaMuPlusLambda(simulation_name, pop, toolbox, mu=MU, lambda_=LAMBDA, 
                              cxpb=cxpb, mutpb=mutpb, ngen=ngen, 
                              stats=stats, halloffame=hof)

    duration = (time.time() - simulation_name) / 1000.0
    try:
        cp = dict(seed=seed, pop_size=pop_size, hof_elem=hof_elem, ngen=ngen, cxpb=cxpb, mutpb=mutpb, time=duration,
            crossover=str(crossover), mutation=str(mutation), select=str(select))
        json_elem = json.dumps(cp)
        path = "./output/{0}".format(simulation_name)
        with open(os.path.join(path, "README"), "w") as cp_file:
            cp_file.writelines(json_elem)
    except Exception as ex:
        print(ex)

    pool.close()

if __name__ == '__main__':
    main()
