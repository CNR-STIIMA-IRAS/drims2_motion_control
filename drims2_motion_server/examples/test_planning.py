import os
clear = lambda: os.system('clear')
clear()
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
import numpy as np
import trimesh
import random 
from scipy.spatial.transform import Rotation as R
# from interfaces.srv import Simplan
import sys
from scipy.special import expit
from drims2_motion_server.motion_client import MotionClient
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
# from sim_plan.modello_pour_liq import reward_pouring



def remap_trajectory(trj: JointTrajectory, joint_name_map, dt):
    """
    Riordina q e interpola traj per avere un punto ogni dt secondi nello stesso tempo totale
    """
    name_to_idx  = {n:i for i,n in enumerate(trj.joint_names)}
    order        = [name_to_idx[n] for n in joint_name_map]
    
    new=[]
    time=[]
    for p in trj.points:
        q = [p.positions[i] for i in order]
        time_from_start = p.time_from_start
        t = time_from_start.sec + time_from_start.nanosec * 1e-9

        new.append(q)
        time.append(t)
    
    # Funziona anche così ma è meno ottimizzato
    # new=[]
    # time=[]
    # if getattr(result, "val") == 1:
    #     for pt in trj.points:
    #         q=np.zeros(7)
    #         for i,q_trj in enumerate(pt.positions):
    #             for j,name in enumerate(joint_name_map):
    #                 if name == trj.joint_names[i]:
    #                     q[j]=q_trj
    #         t=p.time_from_start.sec+p.time_from_start.nanosec*1e-9
    #         time.append(t)
    #         new.append(q)

    t = np.asarray(time, dtype=float)
    Q = np.asarray(new, dtype=float)  # shape: (N, n_joints)

    # Interpolazione lineare per ogni giunto
    t_new = np.arange(0.0, t[-1] + dt, dt)
    q_new = np.zeros((len(t_new), Q.shape[1]))

    for j in range(Q.shape[1]):
        q_new[:, j] = np.interp(t_new, t, Q[:, j])

    return q_new

def ik(pos, quat, q_guess, n_ik, motion_client):
    q_guess=np.asarray(q_guess,dtype=float)
    seed = [float(x) for x in q_guess]
    pose_msg = PoseStamped()
    pose_msg.header.frame_id = "world" # relative motion wrt tool0 frame
    pose_msg.pose.position.x = pos[0]
    pose_msg.pose.position.y = pos[1]
    pose_msg.pose.position.z = pos[2]
    pose_msg.pose.orientation.w = quat[0]
    pose_msg.pose.orientation.x = quat[1]
    pose_msg.pose.orientation.y = quat[2]
    pose_msg.pose.orientation.z = quat[3]

    if n_ik > 0:
        #best_norm = 1e30
        best_err = 1e30
        q_best = None
        for i in range(n_ik):
            # if i%40==0: print("Almost there or maybe not")
            result, tmp = motion_client.solve_ik(pose=pose_msg, seed=seed)
            if tmp is None:
                continue
            tmp=np.asarray(tmp,dtype=float)
            #norm = np.linalg.norm(tmp - q_old)
            err = np.sum(np.abs(tmp - q_guess))
            # print(q_guess)
            # print(tmp)
            # print(f"err: {err}")
            # if norm < best_norm:
            #     best_norm = norm
            #     q_best = tmp
            if err < best_err:
                best_err = err
                q_best = tmp
                #print(best_err)
                if err < np.pi/4:
                    q=q_best
                    q = [float(x) for x in q]
                    print("OK", flush=True)
                    return q

        if q_best is None or best_err > 2*np.pi:
            raise Warning(f"Ocio batocio te funziona mia la IK. Best err: {best_err}")
        elif best_err > np.pi/4:
            print("FAILED", flush=True)
        else:
            print("best err: ", best_err)
        q = q_best
        q_guess = q_best.copy()

    else:
        result, q = motion_client.solve_ik(pose=pose_msg)
    q = [float(x) for x in q]
    return q

def plan_path_full_moveit(
        theta_f,
        parameters,
        motion_client,
        num_waypoints=1000,
        debug=False,
        dt=0.01,
    ):
    logger=get_logger("path logger")
    path=np.empty((0, 6))
    print(f"planning started")
    
    joint_name_map = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]
    #################################
    container_size = np.array([0.0405, 0.054, 0.0404874])
    container2_size = np.array([0.0351, 0.0468, 0.03508908])
    #################################
    x_shift=0.15
    z_min=0.967
    n_ik=100
    lip_height = parameters['pos_cont_goal'][2] + container2_size[2]+0.07
    quat_orizz = np.array([0.5,-0.5,0.5,-0.5])
    
    pos1=np.array([parameters['pos_grip_ee'][0]+x_shift, parameters['pos_grip_ee'][1],parameters['pos_grip_ee'][2]])
    quat1=np.array([parameters['pos_grip_ee'][6], parameters['pos_grip_ee'][3], parameters['pos_grip_ee'][4], parameters['pos_grip_ee'][5]]) # xyzw-> wxyz
    
    
    q_guess1=np.deg2rad(np.array([-122,-125,-122,-115,-123,-180]))
    q1=ik(pos1,quat1,q_guess1,n_ik,motion_client)

    # print(f"pos: {pos1}, quat: {quat1}")
    # print(f"q1: {q1}")

    motion_client.move_to_joint(q1)
    
    ################################# 

    # q2 (sollevam)
    pos2 = pos1.copy()
    pos2[2]+=0.10
    pos2[2]=max(pos2[2],z_min)
    quat2 = quat_orizz
    
    q_guess2=np.deg2rad(np.array([-122,-115,-121,-125,-123,-180]))
    q2=ik(pos2,quat2,q_guess2,n_ik,motion_client)

    #print(f"q2: {q2}, type: {type(q2)}")
    result, trj2 = motion_client.plan_to_joint(joint_target=q2, joint_start=q1)
    
    #motion_client.execute_last_planned_trajectory()

    if getattr(result,"val")==1:
        path2=remap_trajectory(trj2, joint_name_map, dt)
        q2=path2[-1]
        q2 = [float(x) for x in q2]
        path = np.concatenate((path, path2))
        logger.info(f"Pianificazione salita eseguita")
    else:
        print(f"result: {result}")
        logger.error(f"Pianificazione salita fallita")
        quit()

    #################################
    # q3 (approach cont target): movimento principale nel piano Y-Z
    pos3 = np.array([parameters['pos_cont_goal'][0],parameters['pos_cont_goal'][1],parameters['pos_cont_goal'][2]])
    pos3[0]-= 0.02 #x_shift
    pos3[1]-= (0.01+container2_size[0]/2+container_size[0]/2)
    pos3[2] = pos2[2]
    pos3[2]=max(pos3[2],z_min)
    quat3 = quat_orizz
    
    q_guess3=np.deg2rad(np.array([-24,-110,-130,-123,-24,-180]))
    q3=ik(pos3,quat3,q_guess3,n_ik,motion_client)

    #print(f"q2: {q2}")
    v_scale = (1.2 - 0.8) / (400 - 300) * (num_waypoints - 300) + 0.8

    result, trj3 = motion_client.plan_to_joint(joint_target=q3, joint_start=q2, velocity_scaling=v_scale)
    
    #motion_client.execute_last_planned_trajectory()
    
    if getattr(result,"val")==1:
        path3=remap_trajectory(trj3, joint_name_map, dt)
        q3=path3[-1]
        q3 = [float(x) for x in q3]
        # Interpolate with new duration
        # tf=dt*(len(path3)-1)
        # t=np.arange(0.0, tf + 1e-12, dt)
        # tf_new=num_waypoints*dt
        # t_new = np.arange(0.0, tf_new + dt, dt)
        # new = np.zeros((len(t_new), path3.shape[1]))
        # for j in range(path3.shape[1]):
        #     new[:, j] = np.interp(t_new, t, path3[:, j])
        # path3 = new

        path = np.concatenate((path, path3))
        logger.info(f"Pianificazione trasporto eseguita")
    else:
        print(f"result: {result}")
        logger.error(f"Pianificazione trasporto fallita")
        quit()
    #################################
    # q4 (pre vers)
    pos4 = np.array([parameters['pos_cont_goal'][0],parameters['pos_cont_goal'][1],parameters['pos_cont_goal'][2]])
    pos4[0]-= 0.02 #x_shift
    pos4[1] -= (container2_size[0]/2+0.03)
    pos4[2] += container2_size[2]
    pos4[2]=max(pos4[2],z_min,lip_height)
    quat4 = quat_orizz

    n_ik=3
    q_guess4=np.deg2rad(np.array([-24,-120,-130,-112,-24,-180]))
    q4=ik(pos4,quat4,q_guess4,n_ik,motion_client)
    
    result, trj4 = motion_client.plan_to_joint(joint_target=q4, joint_start=q3)

    #motion_client.execute_last_planned_trajectory()
    if getattr(result,"val")==1:
        path4 = remap_trajectory(trj4, joint_name_map, dt)
        q4 = path4[-1]
        q4 = [float(x) for x in q4]
        path = np.concatenate((path, path4))
        logger.info(f"Pianificazione discesa eseguita")
    else:
        print(f"result: {result}")
        logger.error(f"Pianificazione discesa fallita")
        quit()

    ######################################
    # Versamento (4->5)
    CoR3D = np.array([
        parameters['pos_cont_goal'][0] + parameters['dCoR'][0], # 0.0
        parameters['pos_cont_goal'][1] - 0.005 + parameters['dCoR'][1], # - 0.01 
        parameters['pos_cont_goal'][2] + parameters['dCoR'][2], # + 0.04
    ])
    p_tcp0 = pos4.copy()
    #scene.draw_debug_sphere(CoR3D, radius=0.005, color=(1.0, 0.0, 0.0, 1.0))
    R0 = R.from_quat(quat4) # matrice rot init
    l = R0.inv().apply(CoR3D - p_tcp0) # offset tool0 --> CoR3D
    tool_x_axis = np.array([1.0, 0.0, 0.0])             # asse x nel frame tool
    axis_world=tool_x_axis
    
    path5 = []
    q5=q4
    n_steps = int(num_waypoints/2)
    n_new = int(n_steps/20)
    for theta in np.linspace(0, theta_f, n_new):
        R_theta = R.from_rotvec(theta * axis_world) * R0 # matrice rotazione lungo x
        quat5 = R_theta.as_quat()
        delta_pos=R_theta.apply(l)
        p_tcp = CoR3D - delta_pos
        p_tcp[2] = max(pos4[2],z_min, lip_height) 

        tmp=q5.copy()
        q5=ik(p_tcp,quat5,tmp,n_ik,motion_client)    

        path5.append(q5)
    path5 = np.asarray(path5, dtype=float)

    n = path5.shape[0]
    d = path5.shape[1]
    N = n_steps
    t = np.linspace(0, 1, n)
    T = np.linspace(0, 1, N)

    path_dense = np.zeros((N, d))
    for i in range(d):
        path_dense[:, i] = np.interp(T, t, path5[:, i])
    path5 = path_dense
   
    path = np.concatenate((path, path5))

    ###########################################
    # Ritorno dal versamento (5->6)
    path6 = []
    q6=q5
    for theta in np.linspace(theta_f, 0.0, n_new):
        R_theta = R.from_rotvec(theta * axis_world) * R0
        quat6 = R_theta.as_quat()

        p_tcp = CoR3D - R_theta.apply(l)
        p_tcp[2] = max(pos4[2],z_min,lip_height)

        tmp=q6.copy()
        q6=ik(p_tcp,quat6,tmp,n_ik,motion_client) 

        path6.append(q6)
    path6 = np.asarray(path6, dtype=float)

    n = path6.shape[0]
    d = path6.shape[1]
    N = n_steps
    t = np.linspace(0, 1, n)
    T = np.linspace(0, 1, N)

    path_dense = np.zeros((N, d))
    for i in range(d):
        path_dense[:, i] = np.interp(T, t, path6[:, i])
    path6 = path_dense

    q6=path6[-1]
    q6 = [float(x) for x in q6]

    path = np.concatenate((path, path6))
    logger.info(f"Pianificazione pouring e unpouring eseguita")

    pos6=p_tcp
    #################################
    # q7
    n_ik=50
    pos7 = pos6.copy() 
    pos7[2] =parameters['pos_init_cont'][2]+container_size[2]
    pos7[2]=max(pos7[2],z_min)
    quat7 = quat_orizz
    
    q7=ik(pos7,quat7,q6,n_ik,motion_client) 
    #print(f"q2: {q2}")
    result, trj7 = motion_client.plan_to_joint(joint_target=q7, joint_start=q6)
    
    if getattr(result,"val")==1:
        path7=remap_trajectory(trj7, joint_name_map, dt)
        path = np.concatenate((path, path7))
    else:
        logger.error("failure")
        quit()

    # for p in [path, path2, path3, path4, path5, path6, path7]:
    #     for q in p:        
    #         q[0] += np.pi  # somma π alla prima colonna
    #         q = np.concatenate((q, np.array([0.0, 0.0])))
        
    path, path2, path3, path4, path5, path6, path7 = [
    np.hstack((p + np.array([np.pi, 0, 0, 0, 0, 0]), np.zeros((p.shape[0], 2))))
    for p in [path, path2, path3, path4, path5, path6, path7]
    ]



    if debug: print(path)
    print(f"Planning complete")
    
    return {
    "lift": path2,
    "transport": path3,
    "pre_pour": path4,
    "pour": path5,
    "unpour": path6,
    "release": path7,
    "all": path
    }

def main():
    rclpy.init()
    motion_client = MotionClient()

    dt=0.01
    
    parameters = {
        "pos_init_cont": [0.8021465039268282, 0.2847160024669491, 0.9564999991059303],
        "pos_cont_goal": [0.746, 0.961, 0.960],
        "pos_init_ee":  (0.12394027698787038, 0.2665910764094347, 1.1638763741892955, -0.5837257860699608, 0.5925713099055904, -0.388471134978874, 0.3965017359886389),
        "pos_grip_ee": (0.6509734785173125, 0.2847438888358813, 0.9764986855761588, -0.5000033337808922, 0.49998929956451965, -0.4999940109139501, 0.5000133554007884),
        "offset": (0, 0.15, -0.02),
        "dCoR": [0.0, 0.06, -0.004], 
        "vol_init": 60.0, #2e-5, +-MAE
        "densità": 998.0,
        "viscosità": 0.001,
        "tens_sup": 0.072,
        "vol_target": 20.0, #0.75e-5,
        "err_target": 5e-6,
        "theta_f": 87, #+-15°
        "num_wp": 320,
    }
    theta_f=90
    num_wp=320  
    vol_target=40

    parameters["vol_target"]=vol_target 

   
    paths = plan_path_full_moveit(
        theta_f,
        parameters,
        motion_client,
        num_waypoints=1000,
        debug=False,
        dt=0.01,
    )


if __name__ == '__main__':
    main()
