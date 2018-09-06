import sim_load
import os
import json

with open("traces_plot.html", "w") as html:
    traces_dir = "/home/mvend/Scrivania/base_sims/traces/gz3/"
    traces = os.listdir(traces_dir)

    def prepare_trace(t):
        try:
        	with(open(t, "r")) as tj:
	            trace = json.loads(tj.read())
	            trace["name"] = t.strip().split("/")[-1].split(".")[0]
	            return trace
        except:
            print(t)

    traces = map(lambda t: prepare_trace(os.path.join(traces_dir, t)), traces)

    html.write(sim_load.plot_3d_traces(list(traces)))
