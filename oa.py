import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32
import tf.transformations as tf

class OA:
    def __init__(self):
        rospy.init_node('oa', anonymous=True)
        self.gw = 1.0 
        self.ml = 0.5  
        self.ma = 1.0
        self.ow = 2.0 
        self.sw = 0.5 
        self.md = 0.6 
        self.sd = 1.0 
        self.cp = None 
        self.cv = Twist() 
        self.ld = None 
        self.gp = Point() 
        self.aa = False 
        self.wf = False     
        self.fl = True   
        self.cv_pub = rospy.Publisher('/cmd_vel_avoid', Twist, queue_size=1)
        self.ob_pub = rospy.Publisher('/obstacle_detected', Bool, queue_size=1)
        self.ds_pub = rospy.Publisher('/min_obstacle_distance', Float32, queue_size=1)
        self.ls_sub = rospy.Subscriber('/scan', LaserScan, self.lcb)
        self.od_sub = rospy.Subscriber('/odom', Odometry, self.ocb)
        self.gl_sub = rospy.Subscriber('/move_base_simple/goal', Point, self.gcb)
        self.cm_sub = rospy.Subscriber('/cmd_vel', Twist, self.ccb)
        rospy.loginfo("OA init")

    def lcb(self, m):
        self.ld = m
        vr = [r for r in m.ranges if m.range_min <= r <= m.range_max]
        mn = min(vr) if vr else float('inf')
        ob = mn < self.md
        self.ob_pub.publish(Bool(data=ob))
        self.ds_pub.publish(Float32(data=mn))
        self.aa = ob

    def ocb(self, m):
        self.cp = m.pose.pose
        self.cv = m.twist.twist

    def gcb(self, m):
        self.gp = m
        self.wf = False

    def ccb(self, m):
        if not self.aa or self.ld is None:
            self.cv_pub.publish(m)
            return
        av = self.ca(m)
        self.cv_pub.publish(av)

    

    def gc(self):
        cs = []
        vr = 0.1
        wr = 0.2
        for v in np.arange(0, self.ml + vr, vr):
            for w in np.arange(-self.ma, self.ma + wr, wr):
                c = Twist()
                c.linear.x = v
                c.angular.z = w
                if self.iv(c):
                    cs.append(c)
        return cs
    
    def ca(self, dc):
        if self.cp is None:
            return Twist()
        cs = self.gc()
        bc = Twist()
        bs = float('-inf')
        for c in cs:
            s = self.ev(c, dc)
            if s > bs:
                bs = s
                bc = c
        return bc

    def iv(self, c):
        if self.ld is None:
            return False
        dt = 0.1
        hz = 1.0
        x, y, th = 0, 0, 0
        for t in np.arange(0, hz, dt):
            x += c.linear.x * math.cos(th) * dt
            y += c.linear.x * math.sin(th) * dt
            th += c.angular.z * dt
            if self.cc(x, y):
                return False
        return True

    
    def ev(self, c, dc):
        gs = self.gs(c)
        os = self.osc(c)
        ss = self.ss(c, dc)
        return (self.gw * gs + self.ow * os + self.sw * ss)

    def gs(self, c):
        if self.cp is None:
            return 0.0
        dx = self.gp.x - self.cp.position.x
        dy = self.gp.y - self.cp.position.y
        ga = math.atan2(dy, dx)
        o = self.cp.orientation
        _, _, yw = tf.euler_from_quaternion([o.x, o.y, o.z, o.w])
        py = yw + c.angular.z * 0.5
        ad = abs(ga - py)
        ad = min(ad, 2*math.pi - ad)
        al = 1.0 - (ad / math.pi)
        sp = c.linear.x / self.ml
        return al * sp

    def cc(self, rx, ry):
        if self.ld is None:
            return True
        d = math.sqrt(rx**2 + ry**2)
        a = math.atan2(ry, rx)
        am = self.ld.angle_min
        ai = self.ld.angle_increment
        if a < am or a > self.ld.angle_max:
            return False
        idx = int((a - am) / ai)
        if 0 <= idx < len(self.ld.ranges):
            sr = self.ld.ranges[idx]
            if self.ld.range_min <= sr <= self.ld.range_max:
                return d >= sr - 0.2
        return False


    def osc(self, c):
        if self.ld is None:
            return 0.0
        mc = float('inf')
        fa = np.linspace(-math.pi/6, math.pi/6, 20)
        for a in fa:
            idx = int((a - self.ld.angle_min) / self.ld.angle_increment)
            if 0 <= idx < len(self.ld.ranges):
                rv = self.ld.ranges[idx]
                if self.ld.range_min <= rv <= self.ld.range_max:
                    mc = min(mc, rv)
        if mc < self.md:
            return -1.0
        elif mc > self.sd:
            return 1.0
        else:
            r = (mc - self.md) / (self.sd - self.md)
            return 2.0 * r - 1.0

    def wf_bh(self):
        if self.ld is None:
            return Twist()
        c = Twist()
        wa = math.pi / 2 if self.fl else -math.pi / 2
        ai = int((wa - self.ld.angle_min) / self.ld.angle_increment)
        if 0 <= ai < len(self.ld.ranges):
            wd = self.ld.ranges[ai]
            dwd = 0.8
            de = wd - dwd
            if self.fl:
                c.angular.z = -0.5 * de
            else:
                c.angular.z = 0.5 * de
            c.linear.x = 0.2
        return c


    def ss(self, c, dc):
        ld = abs(c.linear.x - dc.linear.x) / self.ml
        ad = abs(c.angular.z - dc.angular.z) / self.ma
        s = 1.0 - 0.5 * (ld + ad)
        return max(0.0, s)

    

def mn():
    try:
        oa = OA()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    mn()