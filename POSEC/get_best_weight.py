#!/usr/bin/env python
"""
Load a snapshotted agent from an hdf5 file and animate it's behavior
"""

import argparse
import cPickle, h5py, numpy as np, time
from collections import defaultdict
import gym
import argparse
from zoopt import Dimension, Objective, Parameter, Opt, Solution
from matplotlib import pyplot

parser = argparse.ArgumentParser()
parser.add_argument("data",type=str,help="pusher, striker, thrower")
args = parser.parse_args()

def ensembel(solution):
    global env, agent_list
    ob = env.reset()
    sum_re = 0
    w = solution.get_x()
    a = []
    act = 0
    for i in xrange(200):
        for j in range(100):
            action, _info = agent_list[j].act(ob)
            a.append(action)
        for m in range(100):
            act += (w[m]*a[m])
        act = act/100.0	

        (ob, rew, done, info) = env.step(act)
        sum_re += rew
        if done:
            print("terminated after %s timesteps"%i)
            break
        value = sum_re
        value = -value
        return value



def main():
    global env, agent_list
    if args.data=="pusher":
        agent_list = []
        best_w_list = []
        for i in xrange(100,120):
            for j in range(100):
                hdf = h5py.File('/home/zhangc/POSEC/base_policy/Pusher/Pusher%d'%j,'r')
                snapnames = hdf['agent_snapshots'].keys()
                snapname = snapnames[-1]        
                agent = cPickle.loads(hdf['agent_snapshots'][snapname].value)
                agent.stochastic=False
                timestep_limit = 200        
                if hasattr(agent,"reset"): agent.reset()
                agent_list.append(agent)
            env = gym.make("Pusher-v%d"%i)
            ob = env.reset()
            dim = 100
            obj = Objective(ensembel, Dimension(dim, [[-1, 1]] * dim, [True] * dim)) # setup objective
            # perform optimization
            solution = Opt.min(obj, Parameter(budget=250))
            # print result
            best_w,reward = solution.print_solution()
            with open('/home/zhangc/POSEC/base_policy/Pusher/Pusher%d_best_w.txt'% i,'a') as f:
                f.write(str(best_w))
            best_w_list.append(best_w)
        with open("/home/zhangc/POSEC/base_policy/Pusher/all_best_w.txt", 'w') as outfile:
            for l in best_w_list:
                num = 0
                for i in l:
                    outfile.write(str(i))
                    if num != 99:                        
                        outfile.write(",")
                    num +=1
                outfile.write("\n")

    elif args.data=="striker":
        agent_list = []
        best_w_list = []
        for i in xrange(100,120):
            for j in range(100):
                hdf = h5py.File('/home/zhangc/POSEC/base_policy/Striker/Striker%d'%j,'r')
                snapnames = hdf['agent_snapshots'].keys()
                snapname = snapnames[-1]        
                agent = cPickle.loads(hdf['agent_snapshots'][snapname].value)
                agent.stochastic=False
                timestep_limit = 200        
                if hasattr(agent,"reset"): agent.reset()
                agent_list.append(agent)
            env = gym.make("Striker-v%d"% i)
            ob = env.reset()
            dim = 100
            obj = Objective(ensembel, Dimension(dim, [[-1, 1]] * dim, [True] * dim)) # setup objective
            # perform optimization
            solution = Opt.min(obj, Parameter(budget=250))
            # print result
            best_w,reward = solution.print_solution()
            with open('/home/zhangc/POSEC/base_policy/Striker/Striker%d_best_w.txt'% i,'a') as f:
                f.write(str(best_w))
            best_w_list.append(best_w)
        with open("/home/zhangc/POSEC/base_policy/Pusher/all_best_w.txt", 'w') as outfile:
            for l in best_w_list:
                num = 0
                for i in l:
                    outfile.write(str(i))
                    if num != 99:                        
                        outfile.write(",")
                    num +=1
                outfile.write("\n")


    elif args.data=="thrower":
        agent_list = []
        best_w_list = []
        for i in xrange(100,120):
            for j in range(100):
                hdf = h5py.File('/home/zhangc/POSEC/base_policy/Thrower/Thrower%d'%j,'r')
                snapnames = hdf['agent_snapshots'].keys()
                snapname = snapnames[-1]        
                agent = cPickle.loads(hdf['agent_snapshots'][snapname].value)
                agent.stochastic=False
                timestep_limit = 200        
                if hasattr(agent,"reset"): agent.reset()
                agent_list.append(agent)
            env = gym.make("Thrower-v%d"% i)
            ob = env.reset()
            dim = 100
            obj = Objective(ensembel, Dimension(dim, [[-1, 1]] * dim, [True] * dim)) # setup objective
            # perform optimization
            solution = Opt.min(obj, Parameter(budget=250))
            # print result
            best_w,reward = solution.print_solution()
            with open('/home/zhangc/POSEC/base_policy/Thrower/Thrower%d_best_w.txt'% i,'a') as f:
                f.write(str(best_w))
            best_w_list.append(best_w)
        with open("/home/zhangc/POSEC/base_policy/Pusher/all_best_w.txt", 'w') as outfile:
            for l in best_w_list:
                num = 0
                for i in l:
                    outfile.write(str(i))
                    if num != 99:                        
                        outfile.write(",")
                    num +=1
                outfile.write("\n")

    else:
        print("Invalid argument")


if __name__ == "__main__":
    global env, agent_list
    main()
