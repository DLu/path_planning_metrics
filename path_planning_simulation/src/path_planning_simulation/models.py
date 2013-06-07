
STATIC = "<gazebo><static>true</static></gazebo>"

def box(name, x, y, z, mass=1.0, color='Blue', is_static=True):
    static_string = STATIC if is_static else ""
    return """
<robot name="%(name)s">%(static_string)s
  <link name="%(name)s">
    <inertial>
      <mass value="%(mass)f" />
      <inertia  ixx="1.0" ixy="0.0"  ixz="0.0"  iyy="100.0"  iyz="0.0"  izz="1.0" />
    </inertial>
    <visual>
      <geometry>
        <box size="%(x)f %(y)f %(z)f" />
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="%(x)f %(y)f %(z)f" />
      </geometry>
    </collision>
  </link>
  <gazebo reference="%(name)s">
    <material>Gazebo/%(color)s</material>
  </gazebo>
</robot>
"""%locals()


