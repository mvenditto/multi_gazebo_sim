DEF_TEMPLATE = """
		<!doctype html>
		<html>
		<head>
			<link rel="stylesheet" href="https://unpkg.com/spectre.css/dist/spectre.min.css">
			<link rel="stylesheet" href="https://unpkg.com/spectre.css/dist/spectre-exp.min.css">
			<link rel="stylesheet" href="https://unpkg.com/spectre.css/dist/spectre-icons.min.css">
			<script src="https://code.jquery.com/jquery-1.10.0.min.js"></script>
			<script src="https://omnipotent.net/jquery.sparkline/2.1.2/jquery.sparkline.min.js"></script>
			<script type="text/javascript">
			    $(function() { $('.inlinesparkline').sparkline(); });
			</script>
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
		      <th>summary</th>
		      <th>max</th>
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

INFO_TEMPLATE = """
		<!doctype html>
		<html>
		<head>
			<link rel="stylesheet" href="https://unpkg.com/spectre.css/dist/spectre.min.css">
			<link rel="stylesheet" href="https://unpkg.com/spectre.css/dist/spectre-exp.min.css">
			<link rel="stylesheet" href="https://unpkg.com/spectre.css/dist/spectre-icons.min.css">
			<script src="https://code.jquery.com/jquery-1.10.0.min.js"></script>
			<style>
				body {
					margin: 10px;
				}
			</style>
		</head>
		<body>
			<h1>Simulations info</h1>
			<table class="table table-striped table-hover">
			  <thead>
			    <tr>
			      <th>name</th>
			      <th>pop_size</th>
			      <th>seed</th>
			      <th>ngens</th>
			      <th>cxpb</th>
			      <th>mutpb</th>
			      <th>crossover</th>
			      <th>mutation</th>
			      <th>selection</th>
			      <th>fitness</th>
			      <th>topology</th>
			      <th>normalization</th>
			      <th>weights_init</th>
			    </tr>
			  </thead>
			  <tbody>
			    {simulations}
			  </tbody>
			</table>
			<script type="text/javascript" src="{ascii_math_src}"></script>
		</body>
		</html>
	"""