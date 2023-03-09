#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
import numpy as np
from cs576.msg import Chain2D
import functools

def get_chain_msg():
    """Return a message from the "chain_config" channel.

    This function will wait until a message is received.
    """
    rospy.init_node("listener", anonymous=True)
    msg = rospy.wait_for_message("chatter", Chain2D)
    print(msg.config)
    return msg

class Vec:
    def __init__(self,a=[0,0],b=[0,0]):
        self.v = [(x2 - x1) for (x1,x2) in zip(a,b)]
    def normalize(self):
        factor = self.abs()
        self.v = [(x/factor) for x in self.v]
    def abs(self):
        return np.sqrt(self.v[0]*self.v[0] + self.v[1]*self.v[1])
    def project(self,a):
        '''
        finds projection of a vector a on this vector.
        Input: The vector to be projected
        Output: The projected vector.
        '''
        out = Vec()
        if (a.abs() == 0):
            raise Exception('abs of vec is 0')
        prod = dot(self,a)/(a.abs()*a.abs())
        out.v = [v*prod for v in a.v]
        return out




def dot(veca,vecb):
    return sum([x*y for (x,y) in zip(veca.v,vecb.v)]) 

def hp(p1,p2,p3,v):
    '''
    The points p1, and p2 make the vector.
    The third point p3 is s.t., vector (p2,p3) must be normal to the vector formed by (p1,p2)
    (all the links are like this)
    v is any point
    The function returns true if p3 and v are in the same side as the vector formed
    by (p1,p2). Otherwise it returns false.
    This is done using half-planes - but based on vector calculations.
    let a = vec(p1,p2), and let b = vec(p1,v) and let n1 = (p2,p3)
    The projection of b on a is (|b| cos(theta)) a^ = (a/|a|)*(a.b/|a|).
    Also, projection + normal = b, so n2 = b - projection.
    Lastly, if n1^.n2^ = -1, then they are on opposite sides, else they are on same side.  
    '''
    a = Vec(p1,p2)
    n1 = Vec(p2,p3)
    b = Vec(p1,v)
    # find projection of b on a:
    c = b.project(a)
    n2 = Vec()
    n2.v = [(x1 - x2) for (x1,x2) in zip(b.v,c.v)]
    n1.normalize()
    n2.normalize()
    if(dot(n1,n2) < 0):
        return False
    else:
        return True 

# Extra credit question.
def get_link_indices_containing(v, config, W, L, D):
    out = set()
    (joints,links) = get_link_positions(config, W, L, D)
    #3             0
    # -------------
    #|              |
    # -------------
    #2             1
    for i,link in enumerate(links):
        if (hp(link[0],link[1],link[2],v) and hp(link[1],link[2],link[3],v) and hp(link[2],link[3],link[0],v) and hp(link[3],link[0],link[1],v)):
            out.add(i)
    return out

def R(theta,x,y):
    return np.array([[np.cos(theta), -np.sin(theta),x],[np.sin(theta),np.cos(theta),y],[0,0,1]])

def plot_chain(config, W, L, D):
    """Plot a 2D kinematic chain A_1, ..., A_m

    @type config: a list [theta_1, ..., theta_m] where theta_1 represents the angle between A_1 and the x-axis,
        and for each i such that 1 < i <= m, \theta_i represents the angle between A_i and A_{i-1}.
    @type W: float, representing the width of each link
    @type L: float, representing the length of each link
    @type D: float, the distance between the two points of attachment on each link
    """

    (joint_positions, link_vertices) = get_link_positions(config, W, L, D)

    fig, ax = plt.subplots()
    plot_links(link_vertices, ax)
    plot_joints(joint_positions, ax)
    plot_q2(config,W,L,D,ax)
    ax.axis("equal")
    plt.show()

def plot_q2(config,W,L,D,ax):
    rot = R(config[0],0,0)
    verticeList = [[5,0,1],[11,0,1],[11,1,1]]
    val1 = np.matmul(rot, np.array(verticeList[0]))[0:2].tolist()

    theta = config[1]
    rot = np.matmul(rot,R(theta,D,0))
    val2 = np.matmul(rot, np.array(verticeList[1]))[0:2].tolist()

    theta = config[2]
    rot = np.matmul(rot,R(theta,D,0))
    val3 = np.matmul(rot, np.array(verticeList[2]))[0:2].tolist()
    ax.plot(val1[0], val1[1], "r.", markersize=10)
    ax.annotate("a",(val1[0],val1[1]))
    ax.plot(val2[0], val2[1], "b.", markersize=10)
    print("a: ", (val1[0],val1[1]))
    ax.annotate("b",(val2[0],val2[1]))
    print("b: ", (val2[0],val2[1]))
    ax.plot(val3[0], val3[1], "g.", markersize=10)
    ax.annotate("c",(val3[0],val3[1]))
    print("c: ", (val3[0],val3[1]))


def plot_links(link_vertices, ax):
    """Plot the links of a 2D kinematic chain A_1, ..., A_m on the axis ax

    @type link_vertices: a list [V_1, ..., V_m] where V_i is the list of [x,y] positions of vertices of A_i
    """

    for vertices in link_vertices:
        x = [vertex[0] for vertex in vertices]
        y = [vertex[1] for vertex in vertices]

        x.append(vertices[0][0])
        y.append(vertices[0][1])
        ax.plot(x, y, "k-", linewidth=2)


def plot_joints(joint_positions, ax):
    """Plot the joints of a 2D kinematic chain A_1, ..., A_m on the axis ax

    @type joint_positions: a list [p_1, ..., p_{m+1}] where p_i is the position [x,y] of the joint between A_i and A_{i-1}
    """
    x = [pos[0] for pos in joint_positions]
    y = [pos[1] for pos in joint_positions]
    ax.plot(x, y, "k.", markersize=10)


def get_link_positions(config, W, L, D):
    """Compute the positions of the links and the joints of a 2D kinematic chain A_1, ..., A_m

    @type config: a list [theta_1, ..., theta_m] where theta_1 represents the angle between A_1 and the x-axis,
        and for each i such that 1 < i <= m, \theta_i represents the angle between A_i and A_{i-1}.
    @type W: float, representing the width of each link
    @type L: float, representing the length of each link
    @type D: float, the distance between the two points of attachment on each link

    @return: a tuple (joint_positions, link_vertices) where
        * joint_positions is a list [p_1, ..., p_{m+1}] where p_i is the position [x,y] of the joint between A_i and A_{i-1}
        * link_vertices is a list [V_1, ..., V_m] where V_i is the list of [x,y] positions of vertices of A_i
    """
    joint_positions = []
    link_vertices = []
    if (len(config) == 0):
        return (joint_positions, link_vertices)
    joint_positions.append([0,0])
    rot = R(config[0],0,0)
    dd = (L-D)/2.0
    val = np.matmul(rot,np.array([D,0,1]))
    joint_positions.append([val[0],val[1]])
    #3             0
    # -------------
    #|              |
    # -------------
    #2             1
    # 1: (-dd,W/2)
    # 2: (D+dd,W/2)
    # 3: (D+dd,-W/2)
    # 4: (-dd,-W/2)
    
    V = []
    V.append([-dd,W/2])
    V.append( [-dd,-W/2])
    V.append([D+dd,-W/2])
    V.append([D+dd,W/2])
    V.reverse()
    
    V1 = [np.matmul(rot, np.append(v,1))[0:2].tolist() for v in V]
    link_vertices.append(V1)
    for i in range(1,len(config),1):
        theta = config[i]
        rot = np.matmul(rot,R(theta,D,0))
        val = np.matmul(rot,np.array([D,0,1]))
        joint_positions.append([val[0],val[1]])
        V1 = [np.matmul(rot, np.append(v,1))[0:2].tolist() for v in V]
        link_vertices.append(V1)
        

    return (joint_positions, link_vertices)
        



if __name__ == "__main__":
    chain = get_chain_msg()
    u = get_link_indices_containing((0.0,14.0),chain.config, chain.W, chain.L, chain.D)
    print("Extra credit, question, for v = (0,14), this is the set: ")
    print(u)
    plot_chain(chain.config, chain.W, chain.L, chain.D)
    # 0.78539816339 1.57079632679 -0.78539816339
    # 0.78539816339 2.09439510239 1.04719755119
    # 0.78539816339 2.09439510239 2.09439510239
    # rosrun cs576 hw2_chain_configurator.py 0.7853939816339 2.09439510239 2.09439510239 -W 2 -L 12 -D 10