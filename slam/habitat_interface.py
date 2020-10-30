"""
Interface class for the Habitat Simulator

Provides robot state packets
Sends goal pose from local planner to simulator

TODO: nav commands
"""


import habitat_sim

from PIL import Image
import numpy as np
from matplotlib import pyplot as plt


class HabitatInterface:

	"""
	Initialize the Habitat Simulator interface
	scene_path: string path to simulator scene file (.glb, .gltf, .obj, .ply)
	"""
	def __init__(self, scene_path):
		# Download the simulator if not found


		# Set up the simulator scene and robot
		self.sim_steps = 0
		self.sim_settings = {
			"scene": scene_path,
			"default_agent": 0,  # Index of the default agent
			"rgb_height": 1.5,  # In meters, relative to agent
			"rgb_res_width": 256,  # Spatial resolution of the observations
			"rgb_res_height": 256,
		}
		self.sim_config = self.configure_sim(self.sim_settings)
		self.sim = habitat_sim.Simulator(self.sim_config) # TODO: try/catch

		# Set up the robot and place in the scene
		self.agent = self.configure_agent(self.sim_settings["default_agent"])

	"""
	Set up the simulation scene and robot
	"""
	def configure_sim(self, sim_settings):
		# Configure simulator with a scene and robot
		sim_cfg = habitat_sim.SimulatorConfiguration()
		sim_cfg.scene.id = self.sim_settings["scene"]

		# Robot config
		agent_cfg = habitat_sim.agent.AgentConfiguration()

		# Add RGB sensor to agent (camera)
		rgb_sensor_spec = habitat_sim.SensorSpec()
		rgb_sensor_spec.uuid = "color_sensor"
		rgb_sensor_spec.sensor_type = habitat_sim.SensorType.COLOR
		rgb_sensor_spec.resolution = [self.sim_settings["rgb_res_height"], 
										self.sim_settings["rgb_res_width"]]
		rgb_sensor_spec.position = [0.0, self.sim_settings["rgb_height"], 0.0]
		agent_cfg.sensor_specifications = [rgb_sensor_spec]

		return habitat_sim.Configuration(sim_cfg, [agent_cfg])

	"""
	Set up the simulated robot
	"""
	def configure_agent(self, agent_settings):
		# Robot init
		agent = self.sim.initialize_agent(agent_settings)

		# Set initial pose
		agent_state = habitat_sim.AgentState()
		agent_state.position = np.array([1.5, 1, 0.0]) # Global frame ref
		agent.set_state(agent_state)

		return agent

	"""
	Assemble a packet of robot state data from the simulator
	{packet: int,
		timestamp: float,
		rgb_sensor: image[],
		pose_estimate: {'position', 'rotation'}}
	"""
	def get_agent_state(self):
		state = {}

		# Metadata
		state['packet'] = self.sim_steps
		state['timestamp'] = self.sim.get_world_time()

		# Sensors
		state['rgb_sensor'] = self.sim.get_sensor_observations()['color_sensor']
		state['pose_estimate'] = self.agent.get_state() # TODO: Noisy odom

		return state

	"""
	Display the rgb sensor image and robot pose in the simulator topo map
	"""
	def display_state(self, semantic_obs=np.array([]), depth_obs=np.array([])):
		state = self.get_agent_state()

		# Show the rgb sensor image
		rgb_image = Image.fromarray(state['rgb_sensor'], mode='RGBA')
		fig, axs = plt.subplots(1, 2, figsize=(12, 8))
		axs[0].set_title('RGB Sensor Image')
		axs[0].imshow(rgb_image)

		# Show the robot pose in the simulator topo map
		pose_point = state['pose_estimate'].position
		meters_per_pixel = 0.1
		height = self.sim.pathfinder.get_bounds()[0][1]
		topdown_map = self.sim.pathfinder.get_topdown_view(meters_per_pixel, 
															height)
		axs[1].set_title('Topological Map')
		axs[1].imshow(topdown_map)
		# Plot points on map
		if pose_point is not None:
			plt.plot(pose_point[0]/meters_per_pixel, 
						pose_point[1]/meters_per_pixel, 
						marker="o", markersize=10, alpha=0.8)

		plt.show()


if __name__ == '__main__':
	scene_path = './data/apartment_1.glb'

	# DEBUG: Start sim and display initial pose and sensor data
	habitat = HabitatInterface(scene_path)
	habitat.display_state()