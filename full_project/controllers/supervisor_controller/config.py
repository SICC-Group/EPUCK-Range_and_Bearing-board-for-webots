import argparse


def get_config():
    parser = argparse.ArgumentParser(
        description="EPUCK_RAB", formatter_class=argparse.RawDescriptionHelpFormatter)

    parser.add_argument('--episodes', type=int, default=20)
    parser.add_argument("--episode_length", type=int, default=200)
    parser.add_argument("--measure_radius", type=float, default=0.5)
    parser.add_argument("--timestep", type=int, default=100)
    parser.add_argument("--interval", type=int, default=5)
    parser.add_argument("--radius_epuck", type=float, default=0.035)
    parser.add_argument('--num_agents', type=int,default=4, help="number of agents")
    parser.add_argument('--num_actions', type=int, default=6, help="number of actions")
    parser.add_argument('--need_real_message', default=True)
    parser.add_argument("--std_gaussian_noise", type=float, default=0.01)
    parser.add_argument('--need_real_range_noise', default=False)
    parser.add_argument('--prob_measure_fail', type=float, default=0.0)
    return parser
