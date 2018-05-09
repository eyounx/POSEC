# -*- coding: utf-8 -*-
"""

@author: Chao Zhang
"""
from numpy import *
import numpy as np 
import gym
import argparse
import cPickle, h5py, numpy as np, time
from zoopt import Dimension, Objective, Parameter, Opt, Solution
from matplotlib import pyplot
#import random 
parser = argparse.ArgumentParser()
parser.add_argument("data",type=str,help="pusher, striker, thrower")
args = parser.parse_args()

def loadfile(filename):
    file = open(filename,'r')
    lines = file.readlines()
    rawdata = []   
    rawtrain = []   
    for line in lines:
       temp = line.split(',')
       rawdata.append(map(float,temp))
    for i in range(len(rawdata)):
       rawtrain.append(rawdata[i][:])
    arawtrain = asarray(rawtrain)  
    return arawtrain

def Regression(X,Y,theta,alpha,m,numIteration):  
    x_trains = X.transpose()   
    for i in range(0,numIteration):      
        hypothesis = np.dot(X,theta)        
        loss = hypothesis - Y             
        cos = np.sum(loss**2)/(2*m)  
        #print "Iteration %d | Cost:%f" % (i,cos)  
          
        gradient = np.dot(x_trains,loss)/m  
        theta = theta - alpha*gradient  
          
    return theta 
    
def max_a_w(solution):
    global agent_list, theta_best, action_sample
    env_list_test = []
    new_state = [[0 for i in range(15)] for j in range(20)]
    ob_list = []
    a = []
    action_list = []
    for t in range(5):
        for i in xrange(120,140):
            env = gym.make("Pusher-v%d"%i)
            env_list_test.append(env)
            ob = env.reset()
            new_state[i-120] = np.zeros((1,15))
            ob, reward, done, info = env.step(action_sample[t])	
            ob_list.append(ob)
            new_state[i-120][0,3*t:3*t+3] = ob[14:17]
    action_sample = solution.get_x()
    sum_re = 0
    for i in xrange(200):
        for m in range(20):
            acti = 0
            for j in range(100):
                action, _info = agent_list[j].act(ob_list[m])
                acti += np.dot(new_state[m],theta_best[j])*action
            (o, r, done, info) = env_list_test[m].step(acti)
            sum_re += r
            if done:
                print("terminated after %s timesteps"%i)
                break
        value = sum_re/20
        value = -value
        return value   

def main():
    global agent_list, theta_best, action_sample
    if args.data=="pusher":
        env_list = []
        agent_list = []
        observation_list = []
        for i in xrange(100,120):
            for j in range(100):
                hdf = h5py.File('/home/zhangc/POSEC/base_policy/Pusher/Pusher%d'%j,'r')
                snapnames = hdf['agent_snapshots'].keys()
                snapname = snapnames[-1]        
                agent = cPickle.loads(hdf['agent_snapshots'][snapname].value)
                agent.stochastic=False
                agent_list.append(agent)
            env = gym.make("Pusher-v%d"%i)
            env_list.append(env)
            observation = env.reset()
            observation_list.append(observation)
        feature = np.zeros((20,15))
        action_sample = []
        for t in range(5):
            for m in range(20):
                a = env.action_space.sample()
                observation, reward, done, info = env_list[m].step(a)
                feature[m,3*t:3*t+3] = observation[14:17]
                action_sample.append(a)
        X = feature
        theta_best = []
        Y = loadfile('/home/zhangc/POSEC/base_policy/Pusher/all_best_w.txt')
        m, n = np.shape(X)
        numIterations =1000  
        alpha = 0.05  
        theta = np.ones(n) 
        for i in range(100):
            YY = Y[:,i]
            theta_op = Regression(X, YY, theta, alpha, m, numIterations) 
            theta_best.append(theta_op)
            with open('/home/zhangc/POSEC/base_policy/Pusher/best_theta.txt', 'a') as f:
                f.write(("theta%d:"+str(theta_op)+"\n")%i)
        dim = 35 # dimension
        obj = Objective(max_a_w, Dimension(dim, [[-2, 2]] * dim, [True] * dim)) 
        solution = Opt.min(obj, Parameter(budget=250))
        best_action,reward = solution.print_solution()
        with open('/home/zhangc/POSEC/base_policy/Pusher/best_action.txt','a') as f:
            f.write("best_action:"+str(best_action)+"\n")
            f.write("reward:"+str(reward))

    elif args.data=="striker":
        env_list = []
        agent_list = []
        observation_list = []
        for i in xrange(100,120):
            for j in range(100):
                hdf = h5py.File('/home/zhangc/POSEC/base_policy/Striker/Striker%d'%j,'r')
                snapnames = hdf['agent_snapshots'].keys()
                snapname = snapnames[-1]        
                agent = cPickle.loads(hdf['agent_snapshots'][snapname].value)
                agent.stochastic=False
                agent_list.append(agent)
            env = gym.make("Striker-v%d"%i)
            env_list.append(env)
            observation = env.reset()
            observation_list.append(observation)
        feature = np.zeros((20,15))
        action_sample = []
        for t in range(5):
            for m in range(20):
                a = env.action_space.sample()
                observation, reward, done, info = env_list[m].step(a)
                feature[i,3*t:3*t+3] = observation[14:17]
                action_sample.append(a)
        X = feature
        theta_best = []
        Y = loadfile('/home/zhangc/POSEC/base_policy/Striker/all_best_w.txt')
        m, n = np.shape(X)
        numIterations =1000  
        alpha = 0.05  
        theta = np.ones(n) 
        for i in range(100):
            YY = Y[:,i]
            theta_op = Regression(X, YY, theta, alpha, m, numIterations) 
            theta_best.append(theta_op)
            with open('/home/zhangc/POSEC/base_policy/Striker/best_theta.txt', 'a') as f:
                f.write("theta%d:"+str(theta_op)+'\n' %i)
        dim = 35 # dimension
        obj = Objective(max_a_w, Dimension(dim, [[-2, 2]] * dim, [True] * dim)) 
        solution = Opt.min(obj, Parameter(budget=250))
        best_action,reward = solution.print_solution()
        with open('/home/zhangc/POSEC/base_policy/Striker/best_action.txt','a') as f:
            f.write(str(best_action))

    elif args.data=="thrower":
        env_list = []
        agent_list = []
        observation_list = []
        for i in xrange(100,120):
            for j in range(100):
                hdf = h5py.File('/home/zhangc/POSEC/base_policy/Thrower/Thrower%d'%j,'r')
                snapnames = hdf['agent_snapshots'].keys()
                snapname = snapnames[-1]        
                agent = cPickle.loads(hdf['agent_snapshots'][snapname].value)
                agent.stochastic=False
                agent_list.append(agent)
            env = gym.make("Thrower-v%d"%i)
            env_list.append(env)
            observation = env.reset()
            observation_list.append(observation)
        feature = np.zeros((20,15))
        action_sample = []
        for t in range(5):
            for m in range(20):
                a = env.action_space.sample()
                observation, reward, done, info = env_list[m].step(a)
                feature[i,3*t:3*t+3] = observation[14:17]
                action_sample.append(a)
        X = feature
        theta_best = []
        Y = loadfile('/home/zhangc/POSEC/base_policy/Thrower/all_best_w.txt')
        m, n = np.shape(X)
        numIterations =1000  
        alpha = 0.05  
        theta = np.ones(n) 
        for i in range(100):
            YY = Y[:,i]
            theta_op = Regression(X, YY, theta, alpha, m, numIterations) 
            theta_best.append(theta_op)
            with open('/home/zhangc/POSEC/base_policy/Thrower/best_theta.txt', 'a') as f:
                f.write("theta%d:"+str(theta_op)+'\n' %i)
        dim = 35 # dimension
        obj = Objective(max_a_w, Dimension(dim, [[-2, 2]] * dim, [True] * dim)) 
        solution = Opt.min(obj, Parameter(budget=250))
        best_action,reward = solution.print_solution()
        with open('/home/zhangc/POSEC/base_policy/Thrower/best_action.txt','a') as f:
            f.write(str(best_action))

    else:
        print("Invalid argument")

if __name__ == "__main__":
    global agent_list, theta_best, action_sample
    main()