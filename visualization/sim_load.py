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
from plotly.graph_objs import Scatter, Scatter3d

colors = "#DA2808, #9E9D22, #D9B41E, #EBBF1D, #FBCD1D, #4DDA8B, #360097, #EA41F2, #F7BCE9, #EA9411, #E95EBD, #E1EBE9, #EC8400, #EF3D1A, #ECFF70, #71CF70, #E3D50B, #F7A227".split(", ")

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


def plot_3d_traces(traces):
    import json

    data = []
    idx = 0

    for trace_ in traces:
        
        if isinstance(trace_, str):
            with open(trace_, "r") as trace_json:
                trace = json.loads(trace_json.read())
        else:
            trace = trace_


        name = trace["name"]
        trace = trace["trace"]

        x = list(map(lambda p: p[0], trace))
        y = list(map(lambda p: p[1], trace))
        z = list(map(lambda p: p[2], trace))

        t = Scatter3d(
            name=name,
            x=x, y=y, z=z,
            mode = "lines",
            line=dict(
                color= colors[idx],
                width=4
            )
        )

        end = Scatter3d(
            name="reached_p_" + name,
            x=[x[-1]], y=[y[-1]], z=[z[-1]],
            mode = "markers",
            marker=dict(
                color="green",
                size=4
            )
        )

        data.append(t)
        data.append(end)
        idx = idx + 1

    o = Scatter3d(
        name="start_point(origin)",
        x=[0], y=[0], z=[0],
        mode = "markers",
        marker=dict(
            color="red",
            size=4
        )
    )

    data.append(o)
    
    layout = dict(
        autosize=True,
        title='Simulations Trace',
        margin=dict(
            l=1,
            r=1,
            b=1,
            t=50,
        ),
        plot_bgcolor='#000',
        scene=dict(
            xaxis=dict(
                range=[-5,5],
                showbackground=True,
                backgroundcolor='rgb(65, 65, 65, .65)'
            ),
            yaxis=dict(
                range=[-5,5],
                showbackground=True,
                backgroundcolor='rgb(65, 65, 65, .65)'
            ),
            zaxis=dict(
                #range=[0,0.5],
                gridcolor='rgb(255, 255, 255)',
                zerolinecolor='rgb(255, 0, 0)',
                showbackground=True,
                backgroundcolor='rgb(85, 85, 85)'
            ),
            camera = dict(
                up=dict(x=0, y=0, z=1),
                center=dict(x=0, y=0, z=0),
                eye=dict(x=0.0, y=1.2, z=0.1)
            ),
            aspectratio = dict( x=2.0, y=1.5, z=0.5),
            aspectmode = 'manual'
        ),
    )

    fig = dict(data=data, layout=layout)

    chart = plot(fig, output_type='div')
    chart = '<div style="height:100%;">' + chart[5:]
    return chart