from flask import Flask, send_file
import os
import datetime
import itertools
import time
from sim_load import *
from io import BytesIO

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

				row = """
				<tr class="active" style="background-color:{9}">
				   <td><strong style="color:{4};">{3}<strong></td>
				   <td><a href="/sim/{3}/{0}">{0}</a></td>
				   <td>{1}</td>
				   <td>{7}</td>
				   <td><i class="icon icon-{5}" style="color:{8};"></i>{6}</td>
				   <td>{2}</td>
				   <td style="text-align:center;"><button onclick="location.href='/sim/{3}/best_{0}'" class="btn btn-primary btn-sm"><i class="icon icon-download"></i></button></td>
				</tr>
				""".format(name, start, ngens, loc, col, status, end, readable_date(last_gen_t), scol, rowc)
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
	# generations = list(filter(lambda x: not x == 'README', os.listdir(sim_path)))
	#return len(generations)
	return last_gen_n, last_gen_time

@app.route('/sim/<loc>/<sim_id>')
def sim_plot(loc, sim_id):
	return plot_logbook(load_logbook(os.path.join(OUT_DIR_MAP[loc], sim_id)))

@app.route('/sim/<loc>/best_<sim_id>')
def sim_get_best(loc, sim_id):
	gen, best =  get_best_indiv(os.path.join(OUT_DIR_MAP[loc], sim_id))
	print(gen, best)
	return send_file(BytesIO(bytes(best, "utf-8")),
		attachment_filename="best_{0}_{1}.py".format(sim_id.replace(".", "_"), gen),
		as_attachment=True)

@app.route('/sim')
def index():
	content = """
		<!doctype html>
		<html>
		<head>
			<link rel="stylesheet" href="https://unpkg.com/spectre.css/dist/spectre.min.css">
			<link rel="stylesheet" href="https://unpkg.com/spectre.css/dist/spectre-exp.min.css">
			<link rel="stylesheet" href="https://unpkg.com/spectre.css/dist/spectre-icons.min.css">
			<style>
				body {
					margin: 10px;
				}
			</style>
		</head>
		<body>
		<h1>Simulations</h1>
		<table class="table table-striped table-hover">
		  <thead>
		    <tr>
		      <th>location</th>
		      <th>name</th>
		      <th>start</th>
		      <th>last update</th>
		      <th>status</th>
		      <th>ngens</th>
		      <th>best individual</th>
		    </tr>
		  </thead>
		  <tbody>
		    {simulations}
		  </tbody>
		</table>
		</body>
		</html>
	"""
	return content.replace("{simulations}", simulations_format())
