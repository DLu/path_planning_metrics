
STATIC = "<static>true</static>"

def box(name, size=[1,1,1], xyz=[0,0,0], rpy=[0,0,0], is_static=True):
    static_string = STATIC if is_static else ""
    w,h,d = size
    x,y,z = xyz
    r,p,yaw = rpy
    return """
<?xml version="1.0" ?>
<sdf version="1.3">
    <world name="default">
        <model name="%(name)s">
            <link name="body">
                <pose>%(x)f %(y)f %(z)f %(r)f %(p)f %(yaw)f</pose>
                <collision name="geom">
                    <geometry>
                        <box>
                            <size>%(w)f %(h)f %(d)f</size>
                        </box>
                    </geometry>
                </collision>
                <visual name="visual1">
                    <geometry>
                        <box>
                            <size>%(w)f %(h)f %(d)f</size>
                        </box>
                    </geometry>
                </visual>
            </link>
            %(static_string)s
            %(plugin_string)s
        </model>
    </world>
</sdf>
"""%locals()

