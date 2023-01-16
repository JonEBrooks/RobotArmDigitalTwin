# -*- coding: utf-8 -*-
"""
Created on Sun Jan 15 14:23:32 2023
@author: Jon Brooks jonedwinbrooks@gmail.com

Credit to Ann Zen on StackOverFlow for lines 102-103
https://stackoverflow.com/questions/38118598/3d-animation-using-matplotlib

This code is intended to help with the design and coding of a 4 DOF robotic arm

I'm new to coding, any suggjestions or improvments to code are appreciated.

Edit robot rotation sequence on line 74, lengths on 72, and camera angle on 96

**NOTE** file should be run in an empty folder as it creates 50 .png files and
a .gif file in the folder it is ran in. 
"""

import numpy as np
import matplotlib.pyplot as plt
import math as m
from PIL import Image

def update_arm(i,Starting_angles,Ending_angles,Arm_lengths):
    angle1 = i*(Ending_angles[0]-Starting_angles[0])/N      #Rotation about xy, Radians
    angle2 = i*(Ending_angles[1]-Starting_angles[1])/N      #Rotation about yz, Radians
    angle3 = i*(Ending_angles[2]-Starting_angles[2])/N      #Rotation about xz, Radians
    angle4 = i*(Ending_angles[3]-Starting_angles[3])/N      #Rotation about xz, Radians
    
    # Calculate end position of Arm1
    Arm1_init = np.mat([[0],[Arm_lengths[0]],[0]])
    Arm_xy_rot = np.mat([[m.cos(angle1),-1*m.sin(angle1),0],
                            [m.sin(angle1),m.cos(angle1),0],
                            [0,0,1]])
    Arm1_yz_rot = yz_rot(angle2)
    Arm1 = np.squeeze(np.asarray(Arm1_yz_rot*Arm1_init))

    # Calculate end position of Arm2
    Arm2 = np.mat([[0],[Arm_lengths[1]],[0]])
    Arm2_yz_rot = yz_rot(angle3)
    Arm2 = np.squeeze(np.asarray(Arm2_yz_rot*Arm2))
    Arm2 = Arm2 + Arm1

    #print(Arm2)

    # Calculate end position of Arm3/end effector
    Arm3 = np.mat([[0],[Arm_lengths[2]],[0]])
    Arm3_yz_rot = yz_rot(angle4)
    Arm3 = np.squeeze(np.asarray(Arm3_yz_rot*Arm3))
    Arm3 = Arm3 + Arm2

    # Rotate Arm about Z axis
    Arm1 = np.squeeze(np.asarray(Arm_xy_rot * np.asmatrix(Arm1).T))
    Arm2 = np.squeeze(np.asarray(Arm_xy_rot * np.asmatrix(Arm2).T))
    Arm3 = np.squeeze(np.asarray(Arm_xy_rot * np.asmatrix(Arm3).T))
    
    # Plot arm segments
    ax.plot([Base_pos[0], Arm1[0]], [Base_pos[1],Arm1[1]],zs=[Base_pos[2],Arm1[2]],linewidth=8,color='k')
    ax.plot([Arm1[0],Arm2[0]],[Arm1[1],Arm2[1]],zs=[Arm1[2],Arm2[2]],linewidth=8,color='k')
    ax.plot([Arm2[0],Arm3[0]],[Arm2[1],Arm3[1]],zs=[Arm2[2],Arm3[2]],linewidth=8,color='k')
    

# Rotation matrix function just to make code cleaner
def yz_rot(angle):
    return np.mat([[1,0,0],[0,m.cos(angle),-1*m.sin(angle)],[0,m.sin(angle),m.cos(angle)]])

# Fixing random state for reproducibility? not sure what this does but it fixes things
np.random.seed(19680801)

# Initialize Robot, Base Position
Base_pos = np.array([0,0,0])
Arm_lengths = np.array([10,8,4])                                #inches
Starting_angles = np.array([0,0,0,0])                           #radians
Ending_angles = np.array([3*m.pi/4,3*m.pi/4,m.pi/6,-m.pi/12])   #radians
#print(str(Base_pos) + " " + str(angle1) + ", " + str(angle2))

# Plot and animation details
Plot_height = 10
Plot_width = 20
N = 50

"""loop to generate images to be merged into gif, calls update_arm to 
generate plot and saves the plot as a .png file"""
for i in range(N):
    # Attaching 3D axis to the figure
    fig = plt.figure()
    ax = fig.add_subplot(projection="3d")
    
    # Setting the axes properties
    ax.set(xlim3d=(-1*Plot_width/2, Plot_width/2), xlabel='X')
    ax.set(ylim3d=(-1*Plot_width/2, Plot_width/2), ylabel='Y')
    ax.set(zlim3d=(0, Plot_height), zlabel='Z')
    
    update_arm(i,Starting_angles,Ending_angles,Arm_lengths)
    
    ax.view_init(0,0)
    
    plt.savefig(f"{i}.png")
    plt.close()

# Use pillow to save all frames as an animation in a gif file
images = [Image.open(f"{i}.png") for i in range(N)]
images[0].save('robot1.gif', save_all=True, append_images=images[1:], duration=100, loop=0)