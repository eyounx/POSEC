# POSEC
**Learning Environmental Calibration Actions for Policy Self-Evolution**<br>
POSEC code for the paper
> Chao Zhang, Yang Yu, Zhi-Hua Zhou. **Learning environmental calibration actions for policy self-evolution**. In: Proceedings of the 27th International Joint Conference on Artificial Intelligence (IJCAI'18), Stockholm, Sweden.

## Requirement
- python 2.7
- argparse
- pickle
- keras (1.0.1)
- theano (0.8.2)
- tabulate
- scipy
- numpy

# Instructions

We mainly use three tasks of mujoco environment in Gym, namely Pusher, Striker and Thrower. Here, we take the Pusher task as an example, and Striker and Thrower are alternative.

- **Step 1:Training Base Policies**<br>
```bash
# Generating multiple different configuration environments<br>
python change_env_config.py pusher 
    
# Using TRPO[1] to generate multiple base policies
python run_pg.py --env pusher --agent modular_rl.agentzoo.TrpoAgent  
```

- **Step 2:Optimizing Combination Weightss**<br>
```bash
# In a batch of new configuration environments, the optimal weights of base policies are obtained based on zoopt[2]
python get_best_weight.py pusher
```

- **Step 3:Optimizing Calibration Actions**<br>
```bash
# In a batch of new configuration environments, the optimal calibration actions are obtained based on zoopt[2]
python get_best_action.py pusher 
```

[1] Schulman J, Levine S, Abbeel P, et al. **Trust region policy optimization**. In: *Proceedings of the 32nd International Conference on Machine Learning (ICML'15)*, pages 1889-1897, Lille, France, 2015.<br>
[2] Yu-Ren Liu, Yi-Qi Hu, Hong Qian, Yang Yu, Chao Qian. **ZOOpt: Toolbox for Derivative-Free Optimization**. arXiv:1801.00329, 2017.
