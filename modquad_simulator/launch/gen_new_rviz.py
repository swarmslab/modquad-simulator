#!/usr/bin/env python

import sys
import time
import rospy

nr = -1
try:
    nr = int(sys.argv[1])
except:
    raise Exception("Issue with num robot arg")

rospy.loginfo("VISUALIZING {} ROBOTS".format(nr))

with open("/home/arch/catkin_ws/src/modquad-simulator/modquad_simulator/rviz/generated.rviz", "w+") as outfile:
    outfile.write("Panels:\n")
    outfile.write("  - Class: rviz/Displays\n")
    outfile.write("    Help Height: 84\n")
    outfile.write("    Name: Displays\n")
    outfile.write("    Property Tree Widget:\n")
    outfile.write("      Expanded:\n")
    outfile.write("        - /Global Options1\n")
    outfile.write("        - /Status1\n")
    outfile.write("        - /modquad021/Namespaces1\n")
    outfile.write("        - /Origin1\n")
    outfile.write("        - /goals11\n")
    outfile.write("      Splitter Ratio: 0.5\n")
    outfile.write("    Tree Height: 708\n")
    outfile.write("  - Class: rviz/Selection\n")
    outfile.write("    Name: Selection\n")
    outfile.write("  - Class: rviz/Tool Properties\n")
    outfile.write("    Expanded:\n")
    outfile.write("      - /2D Pose Estimate1\n")
    outfile.write("      - /2D Nav Goal1\n")
    outfile.write("      - /Publish Point1\n")
    outfile.write("    Name: Tool Properties\n")
    outfile.write("    Splitter Ratio: 0.5886790156364441\n")
    outfile.write("  - Class: rviz/Views\n")
    outfile.write("    Expanded:\n")
    outfile.write("      - /Current View1\n")
    outfile.write("    Name: Views\n")
    outfile.write("    Splitter Ratio: 0.5\n")
    outfile.write("  - Class: rviz/Time\n")
    outfile.write("    Experimental: false\n")
    outfile.write("    Name: Time\n")
    outfile.write("    SyncMode: 0\n")
    outfile.write("    SyncSource: \"\"\n")
    outfile.write("Preferences:\n")
    outfile.write("  PromptSaveOnExit: true\n")
    outfile.write("Toolbars:\n")
    outfile.write("  toolButtonStyle: 2\n")
    outfile.write("Visualization Manager:\n")
    outfile.write("  Class: \"\"\n")
    outfile.write("  Displays:\n")
    outfile.write("    - Alpha: 0.5\n")
    outfile.write("      Cell Size: 1\n")
    outfile.write("      Class: rviz/Grid\n")
    outfile.write("      Color: 160; 160; 164\n")
    outfile.write("      Enabled: true\n")
    outfile.write("      Line Style:\n")
    outfile.write("        Line Width: 0.029999999329447746\n")
    outfile.write("        Value: Lines\n")
    outfile.write("      Name: Grid\n")
    outfile.write("      Normal Cell Count: 0\n")
    outfile.write("      Offset:\n")
    outfile.write("        X: 0\n")
    outfile.write("        Y: 0\n")
    outfile.write("        Z: 0\n")
    outfile.write("      Plane: XY\n")
    outfile.write("      Plane Cell Count: 10\n")
    outfile.write("      Reference Frame: <Fixed Frame>\n")
    outfile.write("      Value: true\n")

    for i in range(1, nr+1):
        outfile.write("    - Class: rviz/Marker\n")
        outfile.write("      Enabled: true\n")
        outfile.write("      Marker Topic: /modquad{:02d}/mesh_visualization/robot\n".format(i))
        outfile.write("      Name: modquad{:02d}\n".format(i))
        outfile.write("      Namespaces:\n")
        outfile.write("        {}\n")
        outfile.write("      Queue Size: 100\n")
        outfile.write("      Value: true\n")

    outfile.write("    - Class: rviz/Axes\n")
    outfile.write("      Enabled: true\n")
    outfile.write("      Length: 0.5\n")
    outfile.write("      Name: Origin\n")
    outfile.write("      Radius: 0.009999999776482582\n")
    outfile.write("      Reference Frame: world\n")
    outfile.write("      Value: true\n")
    outfile.write("    - Class: rviz/Group\n")
    outfile.write("      Displays:\n")

    for i in range(1, nr + 1):
        outfile.write("        - Alpha: 1\n")
        outfile.write("          Axes Length: 1\n")
        outfile.write("          Axes Radius: 0.10000000149011612\n")
        outfile.write("          Class: rviz/Pose\n")
        outfile.write("          Color: 255; 25; 0\n")
        outfile.write("          Enabled: true\n")
        outfile.write("          Head Length: 0.05000000074505806\n")
        outfile.write("          Head Radius: 0.029999999329447746\n")
        outfile.write("          Name: goal{:02d}\n".format(i))
        outfile.write("          Shaft Length: 0.10000000149011612\n")
        outfile.write("          Shaft Radius: 0.009999999776482582\n")
        outfile.write("          Shape: Arrow\n")
        outfile.write("          Topic: /modquad{:02d}/goal\n".format(i))
        outfile.write("          Unreliable: false\n")
        outfile.write("          Value: true\n")

    outfile.write("      Enabled: true\n")
    outfile.write("      Name: goals1\n")
    outfile.write("  Enabled: true\n")
    outfile.write("  Global Options:\n")
    outfile.write("    Background Color: 48; 48; 48\n")
    outfile.write("    Default Light: true\n")
    outfile.write("    Fixed Frame: world\n")
    outfile.write("    Frame Rate: 30\n")
    outfile.write("  Name: root\n")
    outfile.write("  Tools:\n")
    outfile.write("    - Class: rviz/Interact\n")
    outfile.write("      Hide Inactive Objects: true\n")
    outfile.write("    - Class: rviz/MoveCamera\n")
    outfile.write("    - Class: rviz/Select\n")
    outfile.write("    - Class: rviz/FocusCamera\n")
    outfile.write("    - Class: rviz/Measure\n")
    outfile.write("    - Class: rviz/SetInitialPose\n")
    outfile.write("      Theta std deviation: 0.2617993950843811\n")
    outfile.write("      Topic: /initialpose\n")
    outfile.write("      X std deviation: 0.5\n")
    outfile.write("      Y std deviation: 0.5\n")
    outfile.write("    - Class: rviz/SetGoal\n")
    outfile.write("      Topic: /move_base_simple/goal\n")
    outfile.write("    - Class: rviz/PublishPoint\n")
    outfile.write("      Single click: true\n")
    outfile.write("      Topic: /clicked_point\n")
    outfile.write("  Value: true\n")
    outfile.write("  Views:\n")
    outfile.write("    Current:\n")
    outfile.write("      Class: rviz/Orbit\n")
    outfile.write("      Distance: 8.334490776062012\n")
    outfile.write("      Enable Stereo Rendering:\n")
    outfile.write("        Stereo Eye Separation: 0.05999999865889549\n")
    outfile.write("        Stereo Focal Distance: 1\n")
    outfile.write("        Swap Stereo Eyes: false\n")
    outfile.write("        Value: false\n")
    outfile.write("      Focal Point:\n")
    outfile.write("        X: -0.4153444468975067\n")
    outfile.write("        Y: -0.1520715206861496\n")
    outfile.write("        Z: 0.4364039897918701\n")
    outfile.write("      Focal Shape Fixed Size: false\n")
    outfile.write("      Focal Shape Size: 0.05000000074505806\n")
    outfile.write("      Invert Z Axis: false\n")
    outfile.write("      Name: Current View\n")
    outfile.write("      Near Clip Distance: 0.009999999776482582\n")
    outfile.write("      Pitch: 0.32979756593704224\n")
    outfile.write("      Target Frame: world\n")
    outfile.write("      Value: Orbit (rviz)\n")
    outfile.write("      Yaw: 5.5629353523254395\n")
    outfile.write("    Saved: ~\n")
    outfile.write("Window Geometry:\n")
    outfile.write("  \"&Displays\":\n")
    outfile.write("    collapsed: false\n")
    outfile.write("  \"&Time\":\n")
    outfile.write("    collapsed: false\n")
    outfile.write("  Height: 1053\n")
    outfile.write("  Hide Left Dock: false\n")
    outfile.write("  Hide Right Dock: false\n")
    outfile.write("  QMainWindow State: 000000ff00000000fd00000004000000000000016a00000365fc0200000008fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000007601000003fb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000004400000365000000f701000003fb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500530074006500720065006f02000000e6000000d2000003ee0000030bfb0000000c004b0069006e0065006300740200000186000001060000030c00000261000000010000010f0000035dfc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073000000003d0000035d000000d001000003fb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000006ba0000004cfc0100000002fb0000000800540069006d00650100000000000006ba000002ad01000003fb0000000800540069006d006501000000000000045000000000000000000000054f0000036500000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000\n")
    outfile.write("  Selection:\n")
    outfile.write("    collapsed: false\n")
    outfile.write("  Tool Properties:\n")
    outfile.write("    collapsed: false\n")
    outfile.write("  Views:\n")
    outfile.write("    collapsed: false\n")
    outfile.write("  Width: 1722\n")
    outfile.write("  X: 1950\n")
    outfile.write("  Y: 35\n")

while not rospy.is_shutdown():
    time.sleep(1)
