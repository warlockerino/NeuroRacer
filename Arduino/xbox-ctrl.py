#!/usr/bin/env python
    import rospy as ros
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import Joy

    #   Author: Andrew Dai
    #   This ROS Node converts Joystick inputs from the joy node
    #   into commands for turtlesim

    #   Controller Message lesen 
    #   Joysticks in twist umwandeln
    #   Wenn der Deadman Button (A Button) gedrueckt ist,
    #   wird der Twist gepublished fuer andere RosNodes
    #
    #   Fogende Buttons koennen gemapped werden (return 0 | 1)
    #
    #   0   -   A
    #   1   -   B
    #   2   -   X
    #   3   -   Y
    #   4   -   LB
    #   5   -   RB
    #   6   -   Back/Select
    #   7   -   Start   
    #   8   -   Power
    #   9   -   L3
    #   10  -   R3
    #
    #   Zudem koennen folgende Axen, des Controllers genutzt werden (return [-1, 1])
    #
    #   0   -   Linker Stick    (Links / Rechts)
    #   1   -   Linker Stick    (Hoch / Runter)
    #   2   -   Rechter Stick   (Links / Rechts)
    #   3   -   Rechter Stick   (Hoch / Runter)
    #   4   -   Rechter Trigger
    #   5   -   Linker Trigger
    #   6   -   D-Pad           (Links / Rechts)
    #   7   -   D-Pad           (Hoch / Runter)

    def callback(data):
        # Twist erstellen
        twist = Twist()
        # Wenn der DeadmanButton gedrueckt ist, werden die Joystickdaten gelesen
        if (data.buttons[0] == 1):
            twist.linear.x = 4*data.axes[1]
            twist.angular.z = 4*data.axes[4]
        # Ansonsten wird das Auto gestoppt
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        # Twist publishen
        pub.publish(twist)


    # Intializes everything
    def start():
        # publishing "/jetsoncar_teleop_joystick/cmd_vel"
        global pub
        pub = ros.Publisher('/jetsoncar_teleop_joystick/cmd_vel', Twist)
        # Benoetigt, um ControllerInput lesen zu koennen
        ros.Subscriber("joy", Joy, callback)
        # Node starten
        rosinit_node('XBoneCtrl')
        rosspin()

    if __name__ == '__main__':
        start()
