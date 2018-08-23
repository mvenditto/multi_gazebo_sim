import pickle
import array
import sys
import os

from deap import base
from deap import creator

import pygal
import shutil

from pygal.style import CleanStyle

sparkline_style = CleanStyle()
sparkline_style.background = '#ffffff'
sparkline_style.plot_background = '#ffffff'

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
        border: 1.5px solid lightsteelblue;
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
        return max_gen, w

    return "error!"


def load_logbook(sim_out_dir):
    max_gen = get_last_gen(sim_out_dir)

    with open(os.path.join(sim_out_dir, str(max_gen)), "rb") as gen_file:       
        p = pickle.load(gen_file)
        return p["logbook"]

    return []

MIN_VAL = -100.0

def map_neg_infinity(values):
    return map(lambda x: x if x != float("-inf") else MIN_VAL, values)

def sparkline_logbook(logbook, ngens, last_n_gen=80):
	values = list(map(lambda g: str(g['max']), logbook))#[-min(80, int(ngens)):]
	max_ = round(float(values[-1]),4)

	chart = pygal.Line(style=sparkline_style)
	chart.add('', list(map(lambda x: int(float(x)), values)))
	chart.render_sparkline()
	sparkline = chart.render_sparkline().decode("utf-8")

	return sparkline, max_


def plot_logbook(logbook):
    y1 = list(map(lambda g: g['max'], logbook))
    y2 = list(map(lambda g: g['avg'], logbook))
    y3 = list(map(lambda g: g['min'], logbook))
    y4 = list(map(lambda g: g['std'], logbook))
    x = list(map(lambda g: g['gen'], logbook))

    line_chart = pygal.XY(show_dots=False, xrange=(0, int(x[-1])))
    line_chart.title = 'Simulation Plot'
    

    line_chart.add('max', list(zip(x, map_neg_infinity(y1))))
    line_chart.add('avg', zip(x, map_neg_infinity(y2)))
    line_chart.add('min', zip(x, map_neg_infinity(y3)))
    line_chart.add('std', zip(x, map_neg_infinity(y4)))
    
    return PLOT_TEMPLATE.replace("{plot}", line_chart.render().decode("utf-8"))
