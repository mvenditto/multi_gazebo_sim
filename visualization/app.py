from flask import Flask, send_file, url_for
import os
import datetime
import itertools
import time
from sim_load import *
from templates import *
from io import BytesIO
import json
import re

app = Flask(__name__)
app.config["CACHE_TYPE"] = "null"

SIM_OUT_DIRS = [ ]
OUT_DIR_MAP = dict()

with open("root", "r") as root_dir:
	SIM_OUT_DIRS = list(filter(lambda x: not x[0].startswith('#'),[p.strip().split(':') for p in root_dir.readlines()]))

for di in SIM_OUT_DIRS:
	OUT_DIR_MAP[di[0].split('|')[0]] = di[1]

print(SIM_OUT_DIRS)
print(OUT_DIR_MAP)

print(" * {0}".format(SIM_OUT_DIRS))

def readable_date(ms):
	return datetime.datetime.fromtimestamp(ms).strftime("%d/%m/%Y %H:%M:%S")

def date_diff(d2, d1):
        print(d1, d2)
        d1 = datetime.datetime.strptime(d1, "%d/%m/%Y %H:%M:%S")
        d2 = datetime.datetime.strptime(d2, "%d/%m/%Y %H:%M:%S")
        return abs((d2 - d1).seconds) / 3600.

def list_simulations():
	return [(p, os.listdir(p[1])) for p in SIM_OUT_DIRS]

def render_math(x, color="#626262"):
	return f"`color({color})({x})`"
	

def summary_plot():
	sims_ = []
	traces = []
	sims = list_simulations()
	for sim_dir in sims:
		for sim in sim_dir[1]:
			sims_.append(os.path.join(sim_dir[0][1], sim))

	for out_path in sims_:
		print(out_path)
		logbook = load_logbook(out_path)
		logbook_trace = get_logbook_trace(out_path.split('/')[-1], logbook)
		traces.append(logbook_trace)

	plot_page = plot_multi_logbook(traces)
	plot_page = plot_page.replace("{title}", "summary")
	return plot_page

def simulations_info():
	sims = list_simulations()
	content = ""
	idx = 0
	for sim in sims:
		for s in sim[1]:
			name = s
			path = os.path.join(sim[0][1], name)
			readme = os.path.join(path, 'README')
			net = os.path.join(path, 'net')
			loc,_ = sim[0][0].split('|')
			deap_base = "https://deap.readthedocs.io/en/master/api/tools.html#deap.tools."
			sqrt_ = r"sqrt\((.*)\)"
			 
			if not os.path.exists(readme) or not os.path.exists(net):
				continue
			
			try:
				with open(readme, "r") as readme_:
					readme = json.loads(readme_.read()) 
				with open(net, "r") as net_:
					net = json.loads(net_.read()) 

				rowc = "#e3e3e3" if idx % 2 == 0 else "#f0f1f4"

				row = """
				<tr class="active" style="background-color:{0}">
					<td><a href="/sim/{14}/{1}">{1}</a></td>
					<td>{2}</td>
					<td>{3}</td>
					<td>{4}</td>
					<td>{5}</td>
					<td>{6}</td>
					<td><a href="{15}{7}">{7}</a></td>
					<td><a href="{15}{8}">{8}</a></td>
					<td><a href="{15}{9}">{9}</a></td>
					<td>{10}</td>
					<td>{11}</td>
					<td>{12}</td>
					<td>{13}</td>
				</tr>
				""".format(
					    rowc, 
						s, # name 
						readme["pop_size"],
						readme["seed"],
						readme["ngen"],
						readme["cxpb"],
						readme["mutpb"],
						readme["crossover"].split()[1],
						readme["mutation"].split()[1],
						readme["select"].split()[1],
						net["fitness"] if "fitness" in net else "x",
						render_math(net["topology"]),
						render_math(net["input_norm"]["range"]) if "range" in net["input_norm"] else render_math("false"),
						render_math(net["weights_init"]),
						loc,
						deap_base
					)

				content = content + row + "\n"

			except Exception as e:
				print(e)
				continue

			idx += 1

	return content


def simulations_format():
	sims = list_simulations()
	content = ""
	idx = 0
	for sim in sims:
		for s in sim[1]:
			try:
				rowc = "#e3e3e3" if idx % 2 == 0 else "#f0f1f4"
				name = s
				path = os.path.join(sim[0][1], name)
				start = readable_date(float(name))
				ngens, last_gen_t = get_gen_num(path)
				last_update = date_diff(readable_date(time.time()), readable_date(last_gen_t))
				loc,col = sim[0][0].split('|')
				readme = os.path.join(path, 'README')
				status, scol = ("check", "green") if os.path.exists(readme) else ("time", "gray")
				end = "" if status == "time" else " - " + readable_date(os.stat(readme).st_mtime)

				if status == "check":
					end_time = readable_date(os.stat(readme).st_mtime)
				if status == "time" and last_update > 5:
					status,scol = ("cross", "red")

				sparkline, max_ = sparkline_logbook(load_logbook(path), int(ngens))

				row = """
				<tr class="active" style="background-color:{9}">
				   <td><strong style="color:{4};">{3}<strong></td>
				   <td><a href="/sim/{3}/{0}">{0}</a></td>
				   <td>{1}</td>
				   <td>{7}</td>
				   <td><i class="icon icon-{5}" style="color:{8};"></i>{6}</td>
				   <td>{10}</td>
				   <td>{11}</td>
				   <td>{2}</td>
				   <td style="text-align:center;"><button onclick="location.href='/sim/{3}/best_{0}'" class="btn btn-primary btn-sm"><i class="icon icon-download"></i></button></td>
				</tr>
				""".format(name, start, ngens, loc, col, status, end, readable_date(last_gen_t), scol, rowc, sparkline, max_)
				content = content + row + "\n"
			except Exception as ex:
				print(ex)
				continue
			idx += 1
	return content

def get_gen_num(sim_path):
	last_gen_n = get_last_gen(sim_path)
	last_gen = os.path.join(sim_path, str(last_gen_n))
	last_gen_time = os.stat(last_gen).st_mtime
	return last_gen_n, last_gen_time

@app.route('/sim/<loc>/<sim_id>')
def sim_plot(loc, sim_id):
	plot_page = plot_logbook(load_logbook(os.path.join(OUT_DIR_MAP[loc], sim_id)))
	plot_page = plot_page.replace("{title}", f"{loc} > {sim_id}")
	return plot_page

@app.route('/sim/summary')
def sim_splot():
	return summary_plot()

@app.route('/sim/<loc>/best_<sim_id>')
def sim_get_best(loc, sim_id):
	gen, best =  get_best_indiv(os.path.join(OUT_DIR_MAP[loc], sim_id))
	print(gen, best)
	return send_file(BytesIO(bytes(best, "utf-8")),
		attachment_filename="best_{0}_{1}.py".format(sim_id.replace(".", "_"), gen),
		as_attachment=True)

@app.route('/sim/info')
def info():
	math_jax = url_for('static', filename='ASCIIMathML.js')
	ascii_math = url_for('static', filename='ASCIIMathML.js')
	return INFO_TEMPLATE.replace("{simulations}", simulations_info()).replace("{ascii_math_src}", ascii_math)

@app.route('/sim')
def index():
	return DEF_TEMPLATE.replace("{simulations}", simulations_format())
