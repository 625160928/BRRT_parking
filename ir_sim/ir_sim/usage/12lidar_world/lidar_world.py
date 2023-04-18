from ir_sim.env import EnvBase

# env = EnvBase('lidar_world_car.yaml', control_mode='keyboard')
env = EnvBase('lidar_world.yaml', control_mode='keyboard')
# env = EnvBase('lidar_world_omni.yaml', control_mode='keyboard')

for i in range(3000):

    env.step()
    env.render(show_text=True, show_goal=False)
    print(env.get_lidar_scan())
    env.reset('single')

env.end(show_text=True)
