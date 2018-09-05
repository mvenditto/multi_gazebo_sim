import pickle
import array
import sys
import os

from deap import base
from deap import creator

import shutil
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import base64
from io import BytesIO


from plotly.offline import plot
from plotly.graph_objs import Scatter


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
    /*
    svg {
        max-height: 100%;
        margin-left: 15%;
        border: 1.5px solid lightsteelblue;
    }*/
  </style>
  </head>
  <body>
  	<h2>{title}</h2>
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
    print(sim_out_dir, max_gen)
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

def sparkline_logbook(logbook, ngens, figsize=(4, 0.3)):
    y_values = list(map(lambda g: str(g['max']), logbook))
    x_values = list(range(0, ngens))
    max_ = round(float(y_values[-1]),4)

    data = list(y_values)

    fig, ax = plt.subplots(1, 1, figsize=figsize)
    ax.plot(data)
    for k,v in ax.spines.items():
        v.set_visible(False)
    ax.set_xticks([])
    ax.set_yticks([])

    plt.plot(len(data) - 1, data[len(data) - 1], 'r')

    ax.fill_between(range(len(data)), data, len(data)*[min(data)], alpha=0.1)

    img = BytesIO()
    plt.savefig(img, transparent=True, bbox_inches='tight')
    img.seek(0)
    plt.close()

    chart = base64.b64encode(img.read()).decode("UTF-8")
    return '<div><img src="data:image/png;base64,{}"/></div>'.format(chart), max_


def get_logbook_trace(id_, logbook):

    y1 = list(map(lambda g: g['max'], logbook))
    x = list(map(lambda g: g['gen'], logbook))

    return Scatter(
        x = x,
        y = y1,
        mode = 'lines',
        name = str(id_)
    )

def plot_multi_logbook(traces):

    chart = plot(traces, output_type='div')
    chart = '<div style="height:100%;">' + chart[5:]
    
    return PLOT_TEMPLATE.replace("{plot}", chart)
   

def plot_logbook(logbook):

    y1 = list(map(lambda g: g['max'], logbook))
    y2 = list(map(lambda g: g['avg'], logbook))
    y3 = list(map(lambda g: g['min'], logbook))
    y4 = list(map(lambda g: g['std'], logbook))
    x = list(map(lambda g: g['gen'], logbook))

    trace_max = Scatter(
        x = x,
        y = y1,
        mode = 'lines',
        name = 'max'
    )
    trace_avg = Scatter(
        x = x,
        y = y2,
        mode = 'lines',
        name = 'avg'
    )
    trace_min = Scatter(
        x = x,
        y = y3,
        mode = 'lines',
        name = 'min'
    )
    trace_std = Scatter(
        x = x,
        y = y4,
        mode = 'lines',
        name = 'std'
    )

    chart = plot([trace_max, trace_avg, trace_min, trace_std], output_type='div')
    chart = '<div style="height:100%;">' + chart[5:]
    
    return PLOT_TEMPLATE.replace("{plot}", chart)
