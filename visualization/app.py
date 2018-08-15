from flask import Flask, send_file
import os
import datetime
from sim_load import *
from io import BytesIO

app = Flask(__name__)

SIM_OUT_DIR = '/home/gazebo/output/'

with open("root", "r") as root_dir:
	SIM_OUT_DIR = root_dir.readline()

print(" * {0}".format(SIM_OUT_DIR))


def readable_date(ms):
	return datetime.datetime.fromtimestamp(ms).strftime("%d/%m/%Y %H:%M:%S")

def list_simulations():
	return os.listdir(SIM_OUT_DIR)

def simulations_format():
	sims = list_simulations()
	content = ""
	for s in sims:
		try:
			path = os.path.join(SIM_OUT_DIR, s)
			start = readable_date(float(s))
			ngens = get_gen_num(s)
			row = """
			<tr class="active">
			   <td><a href="/sim/{0}">{0}</a></td>
			   <td>{1}</td>
			   <td>{2}</td>
			   <td><button onclick="location.href='/sim/best_{0}'" class="btn btn-primary btn-sm"><i class="icon icon-download"></i></button></td>
			</tr>
			""".format(s, start, ngens)
			content = content + row + "\n"
		except:
			continue
	return content

def get_gen_num(sim_name):
	generations = list(filter(lambda x: not x == 'README', os.listdir(os.path.join(SIM_OUT_DIR, sim_name))))
	return len(generations)

@app.route('/sim/<sim_id>')
def sim_plot(sim_id):
	return plot_logbook(load_logbook(os.path.join(SIM_OUT_DIR, sim_id)))

@app.route('/sim/best_<sim_id>')
def sim_get_best(sim_id):
	best =  get_best_indiv(os.path.join(SIM_OUT_DIR, sim_id))
	return send_file(BytesIO(bytes(best, "utf-8")),
		attachment_filename="best_{0}.py".format(sim_id.replace(".", "_")),
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
		      <th>name</th>
		      <th>start</th>
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
