import pickle
import array
import sys
import os

from deap import base
from deap import creator

import pygal

creator.create("FitnessMax", base.Fitness, weights=(1.0,))
creator.create("Individual", array.array, typecode="f", fitness=creator.FitnessMax)

PLOT_TEMPLATE = """
<!DOCTYPE html>
<html>
  <head>
  <link rel="stylesheet" href="https://unpkg.com/spectre.css/dist/spectre.min.css">
  <style>
    body, html {
        width: 95%;
        height: 95%;
        margin: 10px;
    }
    svg {
        max-height: 100%;
        margin-left: 15%;
    }
  </style>
  <script type="text/javascript" src="http://kozea.github.com/pygal.js/latest/pygal-tooltips.min.js"></script>
    <!-- ... -->
  </head>
  <body>
    {plot}
  </body>
</html>
"""

def get_last_gen(sim_out_dir):
    max_gen = -1
    gens = os.listdir(sim_out_dir)
    for g in gens:
        try:
            gi = int(g)
            max_gen = max(gi, max_gen)
        except:
            pass
    return max_gen


def get_best_indiv(sim_out_dir):
    max_gen = get_last_gen(sim_out_dir)

    with open(os.path.join(sim_out_dir, str(max_gen)), "rb") as gen_file:       
        p = pickle.load(gen_file)
        w = "weights = " + str(p["halloffame"].items[0]).replace("array('f', ", "").replace(")","") 
        return w

    return "error!"


def load_logbook(sim_out_dir):
    max_gen = get_last_gen(sim_out_dir)

    with open(os.path.join(sim_out_dir, str(max_gen)), "rb") as gen_file:       
        p = pickle.load(gen_file)
        return p["logbook"]

    return []

def plot_logbook(logbook):
    y1 = list(map(lambda g: g['max'], logbook))
    y2 = list(map(lambda g: g['avg'], logbook))
    y3 = list(map(lambda g: g['min'], logbook))
    y4 = list(map(lambda g: g['std'], logbook))
    x = list(map(lambda g: g['gen'], logbook))

    line_chart = pygal.XY(show_dots=False)
    line_chart.title = 'Simulation Plot'
    
    line_chart.add('max', list(zip(x,y1)))
    line_chart.add('avg', zip(x,y2))
    line_chart.add('min', zip(x,y3))
    line_chart.add('std', zip(x,y4))
    
    return PLOT_TEMPLATE.replace("{plot}", line_chart.render().decode("utf-8"))
