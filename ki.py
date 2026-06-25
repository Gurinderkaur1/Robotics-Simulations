#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String

class KI: 
    def __init__(self):
        rospy.init_node('ki', anonymous=True)
        self.gp = rospy.Publisher('/delivery_goal', Point, queue_size=1)
        self.cp = rospy.Publisher('/robot_command', String, queue_size=1)
        self.ss = rospy.Subscriber('/robot_state', String, self.scb)
        self.st = rospy.Subscriber('/robot_status', String, self.stb)
        self.cs = "uk"  
        self.ls = "ns"
        self.ru = True
        rospy.loginfo("KI ready")
        self.hp()

    def scb(self, m): 
        self.cs = m.data

    def stb(self, m):   
        self.ls = m.data
        print(f"\n[ST] {self.ls}")
        self.pr()

    def hp(self): 
        print("\n" + "="*40)
        print("keyboard controls")
        print("="*40)
        print(" t = show status")
        print(" s = stop robot")
        print(" q = quit")
        print(" g = goal (with coords)")
        print(" r = return to base")
        print("="*40)
        print(f"State: {self.cs}")
        print(f"Status: {self.ls}")
        print("="*40)

    def pr(self): 
        print(f"\n[{self.cs.upper()}] Command (h=help): ", end='', flush=True)

    def gc(self): 
        try:
            print("\nEnter coords:")
            xs = input("X: ")
            ys = input("Y: ")
            x = float(xs)
            y = float(ys)
            return Point(x=x, y=y, z=0.0)
        except ValueError:
            print("Bad coords!")
            return None
        except KeyboardInterrupt:
            print("\nCancelled.")
            return None

    def hs(self): 
        self.sd("stop")
        print("Stop.")
    
    def sg(self, gp): 
        self.gp.publish(gp)
        print(f"Goal: ({gp.x:.2f}, {gp.y:.2f})")

    def sd(self, c): 
        self.cp.publish(String(data=c))
        print(f"Cmd: {c}")

    def rn(self): 
        while self.ru and not rospy.is_shutdown():
            try:
                print(f"\n[{self.cs.upper()}] Enter cmd: ", end='', flush=True)
                c = input().strip().lower()
                if c == 'g':
                    self.hg()
                elif c == 's':
                    self.hs()
                elif c == 'r':
                    self.hr()
                elif c == 't':
                    self.ht()
                elif c == 'h':
                    self.hp()
                elif c == 'q':
                    self.hq()
                    break
                elif c == '':
                    continue
                else:
                    print(f"Unknown: '{c}'. h=help.")
            except KeyboardInterrupt:
                print("\nBye!")
                break
            except EOFError:
                print("\nEOF, bye!")
                break
            except Exception as e:
                print(f"\nErr: {e}")
                continue
        print("\nEnded.")
        
    def hr(self): 
        self.sd("return")
        print("Return.")

    def ht(self): 
        self.sd("status")
        print("Status.")

    def hq(self): 
        print("\nBye!")
        self.ru = False

def mn():
    try:
        ki = KI()
        ki.rn()
    except rospy.ROSInterruptException:
        print("\nROS down.")
    except KeyboardInterrupt:
        print("\nCtrl-C.")
    except Exception as e:
        print(f"\nErr: {e}")

if __name__ == '__main__':
    mn()