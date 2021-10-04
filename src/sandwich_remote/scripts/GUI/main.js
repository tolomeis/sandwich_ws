createJoystick = function () {
	var options = {
		zone: document.getElementById('zone_joystick'),
		threshold: 0.1,
		position: {left: '50%', top: '50%'},
		mode: 'static',
		size: 150,
		color: 'black',
	};
	manager = nipplejs.create(options);

	linear_speed = 0;	//initial values for the twist message
	angular_speed = 0;

	//on start start sending ROS messages with the speed.
	manager.on('start', function (event, nipple) {
		timer = setInterval(function () {
			app.move(linear_speed, angular_speed);
		}, 25);
	});

	//if moved compute the twist message using the joystick position
	manager.on('move', function (event, nipple) { 
		max_linear = 1.0; 		// m/s
		max_angular = 1.0; 		// rad/s
		max_distance = 75.0; 	// pixels: it is size/2 
		//math magic
		linear_speed = Math.sin(nipple.angle.radian) * max_linear * nipple.distance/max_distance;
		angular_speed = -Math.cos(nipple.angle.radian) * max_angular * nipple.distance/max_distance;
	});

	//needed otherwise the robot keep going with the last message recieved when you stop using the joystick
	manager.on('end', function () { 
		if (timer) {
		clearInterval(timer);
		}
		//app.move(0, 0);
		app.stop();
	});
}

window.onload = function () {
	createJoystick();
	app.ip = location.host.slice(0,-5)
}

// load the camera stream as an image, with the right IP address (from the wp_address input box)
//port 11315 is fixed
loadCamera = function(){
	document.getElementById('cam').innerHTML = '<img src="http://' + app.ip + ':11315/stream?topic='+app.cam_topic + '" class="w3-image" style="width:100%;max-width:960px width="960" height="720"/>';
	app.cstarted = true;
}

stopCamera = function(){
	document.getElementById('cam').innerHTML = '<br/>';
	app.cstarted = false;
}

var app = new Vue({
	el: '#app',
	// storing the state of the page
	data: {
		connected: false,
		cstarted: false,
		ros: null,
		ip: '192.168.1.203',
		ws_address: 'ws://' + this.ip + ':9090',
		cam_topic: 'camera/image_rect_color',
		logs: [],
	},
	// helper methods to connect to ROS
	methods: {
		connect: function() {
			this.logs.unshift('connect to rosbridge server!!')
			this.ros = new ROSLIB.Ros({
				url: 'ws://' + this.ip + ':9090'
			})
			this.ros.on('connection', () => {
				this.connected = true
				this.logs.unshift('Connected!')
				// console.log('Connected!')
			})
			this.ros.on('error', (error) => {
				this.logs.unshift('Error connecting to websocket server')
				// console.log('Error connecting to websocket server: ', error)
			})
			this.ros.on('close', () => {
				this.connected = false
				this.logs.unshift('Connection to websocker server closed')
				// console.log('Connection to websocket server closed.')
			})

		},
		disconnect: function() {
			this.ros.close()
		},
		setTopic: function() {
			this.topic = new ROSLIB.Topic({
				ros: this.ros,
				name: '/cmd_vel',
				messageType: 'geometry_msgs/Twist'
			})
		},
		stop: function() {
			this.message = new ROSLIB.Message({
				linear: { x: 0, y: 0, z: 0, },
				angular: { x: 0, y: 0, z: 0, },
			})
			this.setTopic()
			this.topic.publish(this.message)
		},
		move: function(linear, angular) {
			this.message = new ROSLIB.Message({
				linear: { x: linear, y: 0, z: 0, },
				angular: { x: 0, y: 0, z: angular, },
			})
			this.setTopic()
			this.topic.publish(this.message)
		},
	},
})