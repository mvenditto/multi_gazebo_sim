var SIMPLIFY_THRESH = 100 
var PPM = 10
var H_PPM = PPM / 2

var paths = { }
var individuals = { }
var wh = paper.view.size.width / 2
var hh = paper.view.size.height / 2
var audit_ws_uri = "ws://192.168.33.10:9090/simulation-audit"
var sim_man_ws_uri = "ws://192.168.33.10:9090/simulation-manager"

var test_job =  {
	duration: 25, 
	client_ws: audit_ws_uri
}

init()

var audit_ws = new WebSocket(audit_ws_uri)
var sim_man_ws = new WebSocket(sim_man_ws_uri)

sim_man_ws.onopen = function() {
	sim_man_ws.send(JSON.stringify(test_job))
	sim_man_ws.send(JSON.stringify(test_job))
	sim_man_ws.send(JSON.stringify(test_job))
	sim_man_ws.send(JSON.stringify(test_job))
}

audit_ws.onmessage = function (msg) {
  console.log(msg.data)
  p = JSON.parse(msg.data)
  if (p.type == "sim_data") {
  	add_or_update_path(p)
  } else if (p.type == "sim_end") {
  	sim_man_ws.close()
  	audit_ws.close()
  }
}

function rand_color() { return "#"+((1<<24)*Math.random()|0).toString(16) }

function add_or_update_path(data) {

	var point = new Point((data.x * PPM)  + wh, (data.y * PPM) + hh)

	if (!(data.sim_id in paths)) {

		var color = rand_color()

		var path = new Path({
			segments: [point],
			strokeColor: color,
			strokeWidth: 2,
			strokeCap: 'round',
			strokeJoin: 'round',
			fullySelected: false
		});

		paths[data.sim_id] = path

		var indiv = new Path.Circle(point, PPM / 2);
		indiv.fillColor = color

		var tag_line = new Path({
			segments: [
				[point.x, point.y], 
				[point.x + 2*PPM, point.y - 2*PPM],
				[point.x + 4*PPM, point.y - 2*PPM],
			],
			strokeColor: color,
			strokeWidth: 1,
		});

		var tag_text = new PointText(point);
		tag_text.content = data.sim_id;
		tag_text.style = {
		    fontWeight: 'bold',
		    fontSize: 10,
		    fillColor: color,
		    justification: 'right'
		};

		console.log(tag_text.bounds)
		tag_text.position = new Point(point.x + 4*PPM + tag_text.bounds.width/2, point.y - 2*PPM)

		individuals[data.sim_id] = new Group(indiv, tag_line, tag_text)
	}


	var path = paths[data.sim_id]

	path.add(point)

	if (path.segments.length > SIMPLIFY_THRESH) {
		path.simplify(10)
	}

	var last_point =  path.segments[path.segments.length - 1].point
	var indiv_group = individuals[data.sim_id]
	indiv_group.position.x = (last_point.x + indiv_group.bounds.width / 2) - PPM / 2
	indiv_group.position.y = (last_point.y - indiv_group.bounds.height / 2) + PPM / 2
}

function init() {
	var rect = new Path.Rectangle({
    	point: [0, 0],
    	size: [view.size.width, view.size.height],
    	strokeColor: 'white',
    	selected: true
	});
	rect.sendToBack();
	rect.fillColor = '#2b2d2b';

	var x_axis = new Path({
		segments: [[0, hh], [view.size.width, hh]],
		strokeColor: 'red'
	});
	var y_axis = new Path({
		segments: [[wh, 0], [wh, view.size.height]],
		strokeColor: 'green'
	});
}