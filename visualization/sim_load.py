import pickle
import array
import sys
import os

from deap import base
from deap import creator

import pygal
import shutil

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


def plot_logbook(logbook):
    y1 = list(map(lambda g: g['max'], logbook))
    y2 = list(map(lambda g: g['avg'], logbook))
    y3 = list(map(lambda g: g['min'], logbook))
    y4 = list(map(lambda g: g['std'], logbook))
    x = list(map(lambda g: g['gen'], logbook))

    line_chart = pygal.XY(show_dots=False)
    line_chart.title = 'Simulation Plot'
    

    line_chart.add('max', list(zip(x, map_neg_infinity(y1))))
    line_chart.add('avg', zip(x, map_neg_infinity(y2)))
    line_chart.add('min', zip(x, map_neg_infinity(y3)))
    line_chart.add('std', zip(x, map_neg_infinity(y4)))
    
    return PLOT_TEMPLATE.replace("{plot}", line_chart.render().decode("utf-8"))

def term_plot_logbook(x, y):
    tw,th = shutil.get_terminal_size()
    print(tw,th)
    x = x[0: tw - 2]
    y = y[0: th - 2]

    y = list(map(lambda yi: int(round(yi)),y))
    rows = []
    max_y = max(y)
    histos = list(map(lambda j: ''.join(reversed((u'\u2588' * j) + (' ' * (max_y - j)))), y))

    x_labels = list(map(str, x))
    max_x_label_len = max(map(len, x_labels))

    y_labels = list(map(str, y))
    max_y_label_len = max(map(len, y_labels))

    plot = ''
    for i in range(0, max_y):
        d = map(lambda h: h[i], histos)
        plot += '\n'
        y_l = str(max_y - i)
        plot += y_l + (' ' * (max_y_label_len - len(y_l)))
        plot += ''.join(d)
        
    plot += '\n'

    for i in range(0, max_x_label_len):
        labels_ = map(lambda l: l[i] if len(l) > i else ' ', x_labels)
        plot += (' ' * max_y_label_len) + ''.join(labels_)
        plot += '\n'

    print(plot)

if __name__ == '__main__':
    outdir = "/home/mvend/Scrivania/simulations/output/1534262608.3884723/"
    logbook = load_logbook(outdir)
    x = list(map(lambda g: g['gen'], logbook))
    y = list(map(lambda g: g['max'], logbook))
    histos = term_plot_logbook(x,y)
