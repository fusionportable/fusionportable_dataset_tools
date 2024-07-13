from dateutil import parser
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
from pyquaternion import Quaternion

import re
import os

# def pq_to_tran(t_, q_ , T_ )
    
    
#     R = np.array([[1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
#                 [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
#                 [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]])
#     T_ = np.vstack((np.hstack((R, t[:, np.newaxis])),
#                 np.array([0, 0, 0, 1])))    

def color_map(data, cmap):
 
    dmin, dmax = np.nanmin(data), np.nanmax(data)
    cmo = plt.cm.get_cmap(cmap)
    cs, k = list(), 256/cmo.N
    
    for i in range(cmo.N):
        c = cmo(i)
        for j in range(int(i*k), int((i+1)*k)):
            cs.append(c)
    cs = np.array(cs)
    data = np.uint8(255*(data-dmin)/(dmax-dmin))
    
    return cs[data]

if __name__ == '__main__': 


    # define the extrinsic
    qw, qx, qy, qz = 1, 0, 0, 0  # quant   
    t = np.array([-0.0010, -0.0000, 0.1441])  # translation

    R = np.array([[1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
                [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
                [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]])
    R_left2right = np.array([[1,0,0],
                             [0,-1,0],
                             [0,0,1]]) 
    R = R_left2right * R          #### transform left hand frame to right hand frame

    T = np.vstack((np.hstack((R, t[:, np.newaxis])),
                np.array([0, 0, 0, 1])))
    print( T)

    raw_folder = "./raw"
    output_folder = "./tran2body"

    for file_name in os.listdir(raw_folder):
        if file_name.endswith(".txt"):
            input_file_path=os.path.join(raw_folder,file_name)
            output_file_path=os.path.join(output_folder,file_name)

            print("Current file is {}".format(input_file_path))
            with open(input_file_path, "r") as f_in, open(output_file_path, "w") as f_out:
                lines = f_in.readlines()
                for line in lines:
                    data = line.split()
                    timestamp = float(data[0])
                    x, y, z = float(data[1]), float(data[2]), float(data[3])
                    qw_m, qx_m, qy_m, qz_m = float(data[4]), float(data[5]), float(data[6]), float(data[7])
                    p_marker = np.array([x, y, z, 1])
                    q_marker = np.array([qw_m, qx_m, qy_m, qz_m])
                    T_marker_to_body = np.linalg.inv(T)
                    # print (T_marker_to_body)

                    p_body = T_marker_to_body @ p_marker

                    q_marker = Quaternion(q_marker)
                    q_m2b = Quaternion(qw, qx, qy, qz)
                    q_body = q_m2b * q_marker
                    #print (p_body)
                    #print (q_body)
                    f_out.write("{:.6f} {:.6f} {:.6f} {:.6f} 0 0 0 1\n".format(timestamp, p_body[0], p_body[1], p_body[2])) 

        # extract frame data
        x = []
        y = []
        z = []
        times = []
        num_point =0
        for line in lines:
            data = line.split()
            times.append(float(data[0]))
            x.append(float(data[1]))
            y.append(float(data[2]))
            z.append(float(data[3]))   
            num_point +=1
        # plot 3d traj
        times = np.array(times)
        
        norm = plt.Normalize(times.min(), times.max())
        colors = cm.ScalarMappable(norm=norm, cmap='jet').to_rgba(times)

        fig = plt.figure(figsize=(16,12))
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(x[:], y[:], z[:],color =colors[0], label= file_name)

        for i in range(num_point -1):
            color1, color2 = colors[i], colors[i+1]
            ax.plot(x[i:i+2], y[i:i+2], z[i:i+2],color =color1)
        ax.scatter(x[:],y[:],z[:], c=colors)


        ##ax.plot(x, y, z, label=file_name, color =colors )
        sm = plt.cm.ScalarMappable(cmap='jet', norm = norm)
        sm.set_array([])
        plt.colorbar(sm)
        
        ax.legend()
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        
        #plt.show()

        image_file_name = os.path.splitext(file_name)[0] + ".png"
        plt.savefig(os.path.join("image", image_file_name))
        # plt.show(block=False )
        # plt.pause(1)
        plt.close()