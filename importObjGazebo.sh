#!/bin/bash

dir=`pwd`"/catkin_ws/src/test/worlds/myWorld.world"
if [[ -f "$dir" ]]
then
	echo "found myWorld.world"
else
	echo "creating ./catkin_ws/src/test/worlds/myWorld.world"
	cd ./catkin_ws/src/test/worlds/ && setup.sh
fi

echo "<sdf version='1.7'>
  <world name='default'>
    <model name='ground_plane'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <contact>
              <collide_bitmask>65535</collide_bitmask>
              <ode/>
            </contact>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
       <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    <light name='sun' type='directional'>
      <cast_shadows>1</cast_shadows>
      <pose>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
      <spot>
        <inner_angle>0</inner_angle>
        <outer_angle>0</outer_angle>
        <falloff>0</falloff>
      </spot>
    </light>" > $dir

workdir=`pwd`

while :
do
	echo "give name of file: or write \"done\""
	read file_name
	if  [[ "$file_name" == "done" ]]; then
		break;
	else
		res=`find . -name $file_name`
		echo "found file: $res"

		if [ -f "$res" ]
		then
			echo "give object name:\n"
			read obj_name
			echo "    <model name='$obj_name'>
    <static>1</static>
      <link name='body'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
          <pose>0 0 0 0 -0 0</pose>
        </inertial>
        <pose> 1 1 -0.7 0 -0 0</pose>
        <visual name='visual'>
          <pose>0 -0.008746 1.829789 1.579713 -0 0</pose>
          <geometry>
		<mesh><uri>"$workdir/$res"</uri></mesh>
          </geometry>
          <material>
            <lighting>1</lighting>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
            <shader type='pixel'/>
          </material>
          <transparency>0</transparency>
          <cast_shadows>1</cast_shadows>
        </visual>
        <!-- <collision name='collision'>
          <laser_retro>0</laser_retro>
          <max_contacts>10</max_contacts>
          <pose>0 -0.008746 1.829789 1.579713 -0 0</pose>
          <geometry>
            <mesh><uri>"$workdir/$res"</uri></mesh>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1</mu>
                <mu2>1</mu2>
                <fdir1>0 0 0</fdir1>
                <slip1>0</slip1>
                <slip2>0</slip2>
              </ode>
              <torsional>
                <coefficient>1</coefficient>
                <patch_radius>0</patch_radius>
                <surface_radius>0</surface_radius>
                <use_patch_radius>1</use_patch_radius>
                <ode>
                  <slip>0</slip>
                </ode>
              </torsional>
            </friction>
            <bounce>
              <restitution_coefficient>0</restitution_coefficient>
<threshold>1e+06</threshold>
            </bounce>
            <contact>
              <collide_without_contact>0</collide_without_contact>
              <collide_without_contact_bitmask>1</collide_without_contact_bitmask>
              <collide_bitmask>1</collide_bitmask>
              <ode>
                <soft_cfm>0</soft_cfm>
                <soft_erp>0.2</soft_erp>
                <kp>1e+13</kp>
                <kd>1</kd>
                <max_vel>0.01</max_vel>
                <min_depth>0</min_depth>
              </ode>
              <bullet>
                <split_impulse>1</split_impulse>
                <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
                <soft_cfm>0</soft_cfm>
                <soft_erp>0.2</soft_erp>
                <kp>1e+13</kp>
                <kd>1</kd>
              </bullet>
            </contact>
          </surface>
        </collision>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic> -->
      </link>
    </model>" >>$dir
	else 
		echo "could not find $file_name"
		continue;
	fi
	fi
done

echo "    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type='adiabatic'/>
    <physics type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>200</real_time_update_rate>
    </physics>
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>1</shadows>
    </scene>
    <wind/>
<spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>0</latitude_deg>
      <longitude_deg>0</longitude_deg>
      <elevation>0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
    <state world_name='default'>
      <sim_time>111 500000000</sim_time>
      <real_time>111 908160819</real_time>
      <wall_time>1645016758 410611967</wall_time>
      <iterations>111500</iterations>
      <model name='ground_plane'>
        <pose>0 0 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>0 0 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <light name='sun'>
        <pose>0 0 10 0 -0 0</pose>
      </light>
    </state>
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>4.71354 0.32298 1.14855 0 -0 3.14159</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>
  </world>
</sdf>" >> $dir
