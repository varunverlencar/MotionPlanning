#!/usr/bin/env python
# -*- coding: utf-8 -*-
#HW3 for RBE 595/CS 525 Motion Planning
import time
import scipy
import openravepy

#### YOUR IMPORTS GO HERE #### 

#### END OF YOUR IMPORTS ####

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def tuckarms(env,robot):
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]);        
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    env.Reset()        
    # load a scene from ProjectRoom environment XML file
    env.Load('hw3.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    ### INITIALIZE YOUR PLUGIN HERE ###
    RaveInitialize()
    RaveLoadPlugin('rrtplugin/build/rrtplugin')
    ### END INITIALIZING YOUR PLUGIN ###
   

    # tuck in the PR2's arms for driving
    tuckarms(env,robot);

    #set active joints
    jointnames =['l_shoulder_pan_joint','l_shoulder_lift_joint','l_elbow_flex_joint','l_upper_arm_roll_joint','l_forearm_roll_joint','l_wrist_flex_joint','l_wrist_roll_joint']
    robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])        

    # set start config
    startconfig = [-0.15,0.075,-1.008,0.0,0.0,0.0,0.0]
    robot.SetActiveDOFValues(startconfig);        
    robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

    recorder = RaveCreateModule(env,'viewerrecorder')
    env.AddModule(recorder,'')
    filename = 'openrave.mpg'
    codec = 13 # mpeg2
    recorder.SendCommand('Start 640 480 30 codec %d timing realtime filename %s\nviewer %s'%(codec,filename,env.GetViewer().GetName()))
    
    with env:

        goalconfig = [0.449,-0.201,-0.151,0.0,0.0,-0.11,0.0]

        ### YOUR CODE HERE ###
        ###call your plugin to plan, draw, and execute a path from the current configuration of the left arm to the goalconfig
        initConfig = startconfig + goalconfig
        
        RRTModule = RaveCreateModule(env,'RRTModule')

        a = time.time()
        path = RRTModule.SendCommand('rrtconnect %f %f %f %f %f %f %f %f %f %f %f %f %f %f'%tuple(initConfig))
        a = time.time()-a
        print '\n time:', a

        _unsmoothPath = []; nodes =[]; lowerlimit=[];upperlimit=[]
        # n = cmdout.size();
        # n =n/7
        # print " size",n
        # print cmdout
        if path is None:
            raveLogWarn('command failed!')
        else:
            nodes = path.split(';')
            print 'lenghth',len(nodes)

            # for i in lines[:-1]:
            for i in range(0,len(nodes)-1):
                d = nodes[i].split()
                # print 'node:',i,' ', d,"\n";
                _unsmoothPath.append([float(x) for x in d])
                # for x in d[:-1]:
                #     print x,","
                # print "\n"
        
        # print _unsmoothPath
        indices = robot.GetActiveDOFIndices()       
        lowerlimit,upperlimit = robot.GetDOFLimits(indices)
        lowerlimit[4]= -3.14
        upperlimit[4]= 3.14
        lowerlimit[6]= -3.14
        upperlimit[6]= 3.14

        handles1=[]
        for i in (_unsmoothPath):

                for k in range(0,len(i)-1):
                    if (i[k] != goalconfig[k]):
                        if (i[k] < lowerlimit[k]):
                            i[k]  = lowerlimit[k]
                        elif(i[k] > upperlimit[k]):
                            i[k] = upperlimit[k]
                arr=array([i[0],i[1],i[2],i[3],i[4],i[5],i[6]])
                robot.SetActiveDOFValues(arr)
                pt=robot.GetLinks()[49].GetTransform()[0:3,3]
                handles1.append(env.plot3(pt,pointsize=0.015,colors=array(((1,0,0))),drawstyle=1))



        traj = RaveCreateTrajectory(env,'')
        traj.Init(robot.GetActiveConfigurationSpecification())
        
        for p in range(0,3):
            for j in range(len(_unsmoothPath)):
                    traj.Insert(j,_unsmoothPath[j])

        planningutils.RetimeActiveDOFTrajectory(traj,robot,hastimestamps=False,maxvelmult=1)
        print 'duration',traj.GetDuration()

    robot.GetController().SetPath(traj)
    robot.WaitForController(0)

        
        ### END OF YOUR CODE ###
    waitrobot(robot)

    raw_input("Press enter to exit...")

