"""
Interface class for the Habitat Simulator

Provides robot state packets
Sends goal pose from local planner to simulator

TODO: display util for testing + nav commands
"""


import habitat_sim


class HabitatInterface:

	"""
	Initialize the Habitat Simulator interface
	scene_path: string path to simulator scene file (.glb, .gltf, .obj, .ply)
	"""
	def __init__(self, scene_path):
		# Set up the simulator scene and agent
		self.sim_steps = 0
		self.sim_settings = {
			"scene": scene_path,
			"default_agent": 0,  # Index of the default agent
			"sensor_height": 1.5,  # In meters, relative to agent
			"width": 256,  # Spatial resolution of the observations
			"height": 256,
		}
		self.sim_config = self.configure_sim(self.sim_settings)
		self.sim = habitat_sim.Simulator(self.sim_config) # TODO: try/catch

		# Set up the robot and place in the scene
		self.agent = self.configure_agent(self.sim_settings["default_agent"])

	"""
	Set up the simulation scene and agent
	"""
	def configure_sim(self, sim_settings):
		# Configure simulator with a scene and agent
		sim_cfg = habitat_sim.SimulatorConfiguration()
		sim_cfg.scene.id = settings["scene"]

		# Agent config
		agent_cfg = habitat_sim.agent.AgentConfiguration()

		# Add RGB sensor to agent (camera)
		rgb_sensor_spec = habitat_sim.SensorSpec()
		rgb_sensor_spec.uuid = "color_sensor"
		rgb_sensor_spec.sensor_type = habitat_sim.SensorType.COLOR
		rgb_sensor_spec.resolution = [settings["height"], settings["width"]]
		rgb_sensor_spec.position = [0.0, settings["sensor_height"], 0.0]
		agent_cfg.sensor_specifications = [rgb_sensor_spec]

		return habitat_sim.Configuration(sim_cfg, [agent_cfg])

	"""
	Set up the simulated robot
	"""
	def configure_agent(self, agent_settings):
		# Robot init
		agent = sim.initialize_agent(agent_settings)

		# Set initial pose
		agent_state = habitat_sim.AgentState()
		agent_state.position = np.array([-0.6, 0.0, 0.0]) # Global frame ref
		agent.set_state(agent_state)

		return agent

	"""
	Assemble a packet of robot state data from the simulator
	{packet: int,
		timestamp: float,
		rgb_sensor: image[],
		pose_estimate: {'position', 'rotation'}}
	"""
	def get_robot_state(self):
		state = {}

		# Metadata
		state['packet'] = self.sim_steps += 1
		state['timestamp'] = self.sim.get_world_time()

		# Sensors
		state['rgb_sensor'] = self.sim.get_sensor_observations()['color_sensor']
		state['pose_estimate'] = self.agent.get_state() # TODO: Noisy odom

		return state