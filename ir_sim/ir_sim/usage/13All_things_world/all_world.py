from ir_sim.env import EnvBase

env = EnvBase('all_world_car.yaml', control_mode='keyboard')
# env = EnvBase('all_world.yaml', control_mode='keyboard')

for i in range(3000):

    env.step()
    env.render(show_traj=False, show_text=True, show_goal=False)
    env.reset('single')

env.end(show_text=True)
