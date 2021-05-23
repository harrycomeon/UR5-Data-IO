import numpy as np
import socket
import time
import struct
import util
import rtde

HOST = "192.168.1.107"
#PORT = 30003
PORT = 30003


def get_current_tcp():
    tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_socket.connect((HOST, PORT))
    data = tcp_socket.recv(1108)
    position = struct.unpack('!6d', data[444:492])
    tcp_socket.close()
    return np.asarray(position)


def get_current_pos():  # x, y, theta
    tcp = get_current_tcp()
    rpy = util.rv2rpy(tcp[3], tcp[4], tcp[5])
    return np.asarray([tcp[0], tcp[1], rpy[-1]])


def get_current_pos_same_with_simulation():
    tcp = get_current_tcp()
    rpy = util.rv2rpy(tcp[3], tcp[4], tcp[5])
    print rpy
    return np.asarray([tcp[1], tcp[0], rpy[-1]])

def move_to_tcp(target_tcp):
#    tool_acc = 0.15  # Safe: 0.5
#    tool_vel = 0.05  # Safe: 0.2
    tool_acc = 1.0  # Safe: 0.5
    tool_vel = 0.4  # Safe: 0.2
    tool_pos_tolerance = [0.001, 0.001, 0.001, 0.05, 0.05, 0.05]
    tcp_command = "movel(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0,r=0)\n" % (
        target_tcp[0], target_tcp[1], target_tcp[2], target_tcp[3], target_tcp[4],
        target_tcp[5],
        tool_acc, tool_vel)
    tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_socket.connect((HOST, PORT))
    tcp_socket.send(str.encode(tcp_command)) 
    tcp_socket.close()
    actual_pos = get_current_tcp()
    target_rpy = util.rv2rpy(target_tcp[3], target_tcp[4], target_tcp[5])
    rpy = util.rv2rpy(actual_pos[3], actual_pos[4], actual_pos[5])
    while not (all([np.abs(actual_pos[j] - target_tcp[j]) < tool_pos_tolerance[j] for j in range(3)])
               and all([np.abs(rpy[j] - target_rpy[j]) < tool_pos_tolerance[j+3] for j in range(3)])):
        actual_pos = get_current_tcp()
        rpy = util.rv2rpy(actual_pos[3], actual_pos[4], actual_pos[5])
        time.sleep(0.01)


def increase_move(delta_x, delta_y, delta_z, delta_theta):
    tcp = get_current_tcp()
    rpy = util.rv2rpy(tcp[3], tcp[4], tcp[5])
    rpy[2] = rpy[2] + delta_theta
    target_rv = util.rpy2rv(rpy)
    target_tcp = np.asarray([tcp[0] + delta_x, tcp[1] + delta_y, tcp[2] + delta_z,
                             target_rv[0], target_rv[1], target_rv[2]])
    move_to_tcp(target_tcp)


def get_digital_output():
    tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_socket.connect((HOST, PORT))
    data = tcp_socket.recv(1108)
    tool = struct.unpack('!d', data[1044:1052])[0]
    print tool
    tcp_socket.close()
    return tool


def operate_gripper(target_width):
    tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_socket.connect((HOST, PORT))

    tcp_command = "def rg2ProgOpen():\n"
    tcp_command += "\ttextmsg(\"inside RG2 function called\")\n"

    tcp_command += '\ttarget_width={}\n'.format(target_width)
    tcp_command += "\ttarget_force=40\n"
    tcp_command += "\tpayload=1.0\n"
    tcp_command += "\tset_payload1=False\n"
    tcp_command += "\tdepth_compensation=False\n"
    tcp_command += "\tslave=False\n"

    tcp_command += "\ttimeout = 0\n"
    tcp_command += "\twhile get_digital_in(9) == False:\n"
    tcp_command += "\t\ttextmsg(\"inside while\")\n"
    tcp_command += "\t\tif timeout > 400:\n"
    tcp_command += "\t\t\tbreak\n"
    tcp_command += "\t\tend\n"
    tcp_command += "\t\ttimeout = timeout+1\n"
    tcp_command += "\t\tsync()\n"
    tcp_command += "\tend\n"
    tcp_command += "\ttextmsg(\"outside while\")\n"

    tcp_command += "\tdef bit(input):\n"
    tcp_command += "\t\tmsb=65536\n"
    tcp_command += "\t\tlocal i=0\n"
    tcp_command += "\t\tlocal output=0\n"
    tcp_command += "\t\twhile i<17:\n"
    tcp_command += "\t\t\tset_digital_out(8,True)\n"
    tcp_command += "\t\t\tif input>=msb:\n"
    tcp_command += "\t\t\t\tinput=input-msb\n"
    tcp_command += "\t\t\t\tset_digital_out(9,False)\n"
    tcp_command += "\t\t\telse:\n"
    tcp_command += "\t\t\t\tset_digital_out(9,True)\n"
    tcp_command += "\t\t\tend\n"
    tcp_command += "\t\t\tif get_digital_in(8):\n"
    tcp_command += "\t\t\t\tout=1\n"
    tcp_command += "\t\t\tend\n"
    tcp_command += "\t\t\tsync()\n"
    tcp_command += "\t\t\tset_digital_out(8,False)\n"
    tcp_command += "\t\t\tsync()\n"
    tcp_command += "\t\t\tinput=input*2\n"
    tcp_command += "\t\t\toutput=output*2\n"
    tcp_command += "\t\t\ti=i+1\n"
    tcp_command += "\t\tend\n"
    tcp_command += "\t\treturn output\n"
    tcp_command += "\tend\n"
    tcp_command += "\ttextmsg(\"outside bit definition\")\n"

    tcp_command += "\ttarget_width=target_width+0.0\n"
    tcp_command += "\tif target_force>40:\n"
    tcp_command += "\t\ttarget_force=40\n"
    tcp_command += "\tend\n"

    tcp_command += "\tif target_force<4:\n"
    tcp_command += "\t\ttarget_force=4\n"
    tcp_command += "\tend\n"
    tcp_command += "\tif target_width>110:\n"
    tcp_command += "\t\ttarget_width=110\n"
    tcp_command += "\tend\n"
    tcp_command += "\tif target_width<0:\n"
    tcp_command += "\t\ttarget_width=0\n"
    tcp_command += "\tend\n"
    tcp_command += "\trg_data=floor(target_width)*4\n"
    tcp_command += "\trg_data=rg_data+floor(target_force/2)*4*111\n"
    tcp_command += "\tif slave:\n"
    tcp_command += "\t\trg_data=rg_data+16384\n"
    tcp_command += "\tend\n"

    tcp_command += "\ttextmsg(\"about to call bit\")\n"
    tcp_command += "\tbit(rg_data)\n"
    tcp_command += "\ttextmsg(\"called bit\")\n"

    tcp_command += "\tif depth_compensation:\n"
    tcp_command += "\t\tfinger_length = 55.0/1000\n"
    tcp_command += "\t\tfinger_heigth_disp = 5.0/1000\n"
    tcp_command += "\t\tcenter_displacement = 7.5/1000\n"

    tcp_command += "\t\tstart_pose = get_forward_kin()\n"
    tcp_command += "\t\tset_analog_inputrange(2, 1)\n"
    tcp_command += "\t\tzscale = (get_analog_in(2)-0.026)/2.976\n"
    tcp_command += "\t\tzangle = zscale*1.57079633-0.087266462\n"
    tcp_command += "\t\tzwidth = 5+110*sin(zangle)\n"

    tcp_command += "\t\tstart_depth = cos(zangle)*finger_length\n"

    tcp_command += "\t\tsync()\n"
    tcp_command += "\t\tsync()\n"
    tcp_command += "\t\ttimeout = 0\n"

    tcp_command += "\t\twhile get_digital_in(9) == True:\n"
    tcp_command += "\t\t\ttimeout=timeout+1\n"
    tcp_command += "\t\t\tsync()\n"
    tcp_command += "\t\t\tif timeout > 20:\n"
    tcp_command += "\t\t\t\tbreak\n"
    tcp_command += "\t\t\tend\n"
    tcp_command += "\t\tend\n"
    tcp_command += "\t\ttimeout = 0\n"
    tcp_command += "\t\twhile get_digital_in(9) == False:\n"
    tcp_command += "\t\t\tzscale = (get_analog_in(2)-0.026)/2.976\n"
    tcp_command += "\t\t\tzangle = zscale*1.57079633-0.087266462\n"
    tcp_command += "\t\t\tzwidth = 5+110*sin(zangle)\n"
    tcp_command += "\t\t\tmeasure_depth = cos(zangle)*finger_length\n"
    tcp_command += "\t\t\tcompensation_depth = (measure_depth - start_depth)\n"
    tcp_command += "\t\t\ttarget_pose = pose_trans(start_pose,p[0,0,-compensation_depth,0,0,0])\n"
    tcp_command += "\t\t\tif timeout > 400:\n"
    tcp_command += "\t\t\t\tbreak\n"
    tcp_command += "\t\t\tend\n"
    tcp_command += "\t\t\ttimeout=timeout+1\n"
    tcp_command += "\t\t\tservoj(get_inverse_kin(target_pose),0,0,0.008,0.033,1700)\n"
    tcp_command += "\t\tend\n"
    tcp_command += "\t\tnspeed = norm(get_actual_tcp_speed())\n"
    tcp_command += "\t\twhile nspeed > 0.001:\n"
    tcp_command += "\t\t\tservoj(get_inverse_kin(target_pose),0,0,0.008,0.033,1700)\n"
    tcp_command += "\t\t\tnspeed = norm(get_actual_tcp_speed())\n"
    tcp_command += "\t\tend\n"
    tcp_command += "\tend\n"
    tcp_command += "\tif depth_compensation==False:\n"
    tcp_command += "\t\ttimeout = 0\n"
    tcp_command += "\t\twhile get_digital_in(9) == True:\n"
    tcp_command += "\t\t\ttimeout = timeout+1\n"
    tcp_command += "\t\t\tsync()\n"
    tcp_command += "\t\t\tif timeout > 20:\n"
    tcp_command += "\t\t\t\tbreak\n"
    tcp_command += "\t\t\tend\n"
    tcp_command += "\t\tend\n"
    tcp_command += "\t\ttimeout = 0\n"
    tcp_command += "\t\twhile get_digital_in(9) == False:\n"
    tcp_command += "\t\t\ttimeout = timeout+1\n"
    tcp_command += "\t\t\tsync()\n"
    tcp_command += "\t\t\tif timeout > 400:\n"
    tcp_command += "\t\t\t\tbreak\n"
    tcp_command += "\t\t\tend\n"
    tcp_command += "\t\tend\n"
    tcp_command += "\tend\n"
    tcp_command += "\tif set_payload1:\n"
    tcp_command += "\t\tif slave:\n"
    tcp_command += "\t\t\tif get_analog_in(3) < 2:\n"
    tcp_command += "\t\t\t\tzslam=0\n"
    tcp_command += "\t\t\telse:\n"
    tcp_command += "\t\t\t\tzslam=payload\n"
    tcp_command += "\t\t\tend\n"
    tcp_command += "\t\telse:\n"
    tcp_command += "\t\t\tif get_digital_in(8) == False:\n"
    tcp_command += "\t\t\t\tzmasm=0\n"
    tcp_command += "\t\t\telse:\n"
    tcp_command += "\t\t\t\tzmasm=payload\n"
    tcp_command += "\t\t\tend\n"
    tcp_command += "\t\tend\n"
    tcp_command += "\t\tzsysm=0.0\n"
    tcp_command += "\t\tzload=zmasm+zslam+zsysm\n"
    tcp_command += "\t\tset_payload(zload)\n"
    tcp_command += "\tend\n"

    tcp_command += "end\n"

    tcp_socket.send(str.encode(tcp_command)) 
    tcp_socket.close()
    time.sleep(1)
    # gripper_fully_closed = check_grasp()


def check_grasp():
    con = rtde.RTDE(HOST, 30004)
    con.connect()
    output_names = ['tool_analog_input0']
    output_types = ['DOUBLE']
    con.send_output_setup(output_names, output_types, frequency=125)
    con.send_start()
    state = con.receive(True)
    voltage = struct.unpack('!d', state)
    return voltage[0] > 0.3


def move_down():
    tcp = get_current_tcp()
#    tcp[0] = tcp[0]+ 0.023  #success1
#    tcp[0] = tcp[0]- 0.030   #success2
    tcp[1] = tcp[1]+ 0.0025 #success1
#    tcp[1] = tcp[1]+ 0.000   #success2
#    tcp[2] = 0.235
    #tcp[4] = 0.1
    move_to_tcp(tcp)


def move_up():
    tcp = get_current_tcp()
    print tcp
#    tcp[2] = 0.6
    tcp[4] = tcp[4] + 0.55

#    tcp[1] = tcp[1]- 0.+
#    tcp[1] = tcp[1]- 0.1
    move_to_tcp(tcp)


def grasp():
    operate_gripper(100)
    move_down()
    operate_gripper(0)
    move_up()
    return check_grasp()


def go_home():
    tcp = get_current_tcp()
    print tcp
    #operate_gripper(100)
#    move_to_tcp([0, -0.47, 0.13, 3.14165, 0., 0.])
    move_to_tcp([ 0.31604773 , 0.3203634  , 0.18102446 , 2.54485043 , 1.78863356 , 0.07259045])
#  xuanzhuan vector 2.54498248  1.78863763  0.07259738 -> rpy 
    move_to_tcp([ 0.51306594 , 0.57392898 , 0.18096626 , 2.5479779 ,  1.78540601 , 0.07130216])


if __name__ == '__main__':
    go_home()
    #get_digital_output()
    #get_current_pos_same_with_simulation()
    #rpy1 = [3.09013946, -0.02080552, 1.22570673]
    #rpy1 = np.array(rpy1).astype('float64')
    #retv= util.rpy2rv(rpy1)
    #print retv
#    move_up()
#    move_up()
#    move_up()
#    move_up()   
#    move_down()
#    move_down()
#    move_down()
#    move_up()
#    move_down()
#    move_down()
#    operate_gripper(93)  ji mu zhou
#    operate_gripper(25)
#    move_up()

#    operate_gripper(100)
    


