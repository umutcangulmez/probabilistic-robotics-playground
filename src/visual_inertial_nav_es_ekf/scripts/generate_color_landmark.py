#!/usr/bin/env python3
"""
Generate Gazebo SDF world file with configurable number of landmarks.

Usage:
    python generate_world.py --num-landmarks 10 --output my_world.sdf
    python generate_world.py -n 20 -o world_20_landmarks.sdf
"""

import argparse
import colorsys
import math
import random
from typing import List, Tuple


def generate_unique_colors(n: int, saturation: float = 1.0, value: float = 1.0, hue_range: int = 10) -> List[dict]:
    """
    Generate n unique, visually distinct colors using HSV color space.

    Args:
        n: Number of unique colors to generate
        saturation: Color saturation (0-1)
        value: Color brightness (0-1)
        hue_range: +/- range for HSV detection bounds (in OpenCV 0-179 scale)

    Returns:
        List of color dictionaries with RGB, HSV, and detection ranges
    """
    # Golden ratio gives ~222° (110 in OpenCV scale) separation between consecutive hues
    # This is much better than uniform distribution, so we can have more landmarks
    # But warn if hue ranges might still overlap due to too many landmarks
    min_separation = int(179 * 0.618 / 2)  # ~55 in OpenCV scale (half golden ratio)
    if n > 1 and (2 * hue_range) > min_separation:
        max_safe = 179 // (2 * hue_range)
        if n > max_safe:
            print(f"WARNING: {n} landmarks with hue_range={hue_range} may cause color overlap. Consider reducing to {max_safe} or using smaller hue_range.")

    colors = []

    # Use golden ratio to distribute hues evenly
    golden_ratio = 0.618033988749895
    hue = random.random()  # Start with random hue

    for i in range(n):
        # Convert HSV to RGB
        r, g, b = colorsys.hsv_to_rgb(hue, saturation, value)

        # OpenCV uses H: 0-179, S: 0-255, V: 0-255
        h_opencv = int(hue * 179)
        s_opencv = int(saturation * 255)
        v_opencv = int(value * 255)

        # Create detection bounds (handle hue wraparound)
        h_lower = max(0, h_opencv - hue_range)
        h_upper = min(179, h_opencv + hue_range)

        # Create color name based on hue angle
        hue_deg = int(hue * 360)
        color_name = f"color_{hue_deg:03d}"

        # Format RGBA string for SDF
        rgba = f"{r:.3f} {g:.3f} {b:.3f} 1"

        colors.append({
            "name": color_name,
            "rgba": rgba,
            "rgb": [round(r, 3), round(g, 3), round(b, 3)],
            "hsv_opencv": [h_opencv, s_opencv, v_opencv],
            "h_lower": h_lower,
            "h_upper": h_upper,
        })

        # Increment hue using golden ratio for maximum visual separation
        hue = (hue + golden_ratio) % 1.0

    return colors


def generate_landmark_positions(
        num_landmarks: int,
        min_radius: float = 4.0,
        max_radius: float = 15.0,
        min_distance: float = 2.0,
        seed: int = None
) -> List[Tuple[float, float]]:
    """
    Generate landmark positions distributed around the origin.

    Args:
        num_landmarks: Number of landmarks to generate
        min_radius: Minimum distance from origin
        max_radius: Maximum distance from origin
        min_distance: Minimum distance between landmarks
        seed: Random seed for reproducibility

    Returns:
        List of (x, y) positions
    """
    if seed is not None:
        random.seed(seed)

    positions = []
    max_attempts = 1000

    for i in range(num_landmarks):
        for attempt in range(max_attempts):
            # Generate random position in annular region
            angle = random.uniform(0, 2 * math.pi)
            radius = random.uniform(min_radius, max_radius)
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)

            # Check distance from all existing landmarks
            valid = True
            for px, py in positions:
                dist = math.sqrt((x - px)**2 + (y - py)**2)
                if dist < min_distance:
                    valid = False
                    break

            if valid:
                positions.append((x, y))
                break
        else:
            # If we couldn't place the landmark, use a grid fallback
            angle = 2 * math.pi * i / num_landmarks
            radius = min_radius + (max_radius - min_radius) * (i % 3) / 2
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            positions.append((x, y))

    return positions


def generate_landmark_sdf(
        name: str,
        x: float,
        y: float,
        color_name: str,
        color_rgba: str,
        height: float = 1.0,
        radius: float = 0.2
) -> str:
    """Generate SDF XML for a single landmark."""
    return f"""
        <model name='landmark_{name}'>
            <static>true</static>
            <pose>{x:.3f} {y:.3f} {height/2:.3f} 0 0 0</pose>
            <link name='link'>
                <visual name='visual'>
                    <geometry>
                        <cylinder>
                            <radius>{radius}</radius>
                            <length>{height}</length>
                        </cylinder>
                    </geometry>
                    <material>
                        <ambient>{color_rgba}</ambient>
                        <diffuse>{color_rgba}</diffuse>
                    </material>
                </visual>
                <collision name='collision'>
                    <geometry>
                        <cylinder>
                            <radius>{radius}</radius>
                            <length>{height}</length>
                        </cylinder>
                    </geometry>
                </collision>
            </link>
        </model>
"""


def generate_turtlebot3_waffle_sdf() -> str:
    """Generate TurtleBot3 Waffle model with IMU and camera sensors (using simple shapes)."""
    return """
        <model name='turtlebot3_waffle' canonical_link='base_footprint'>
            <pose>0 0 0 0 0 0</pose>
            
            <link name='base_footprint'>
                <pose>0 0 0 0 0 0</pose>
                <inertial>
                    <mass>0.001</mass>
                    <inertia>
                        <ixx>0.0001</ixx><ixy>0</ixy><ixz>0</ixz>
                        <iyy>0.0001</iyy><iyz>0</iyz><izz>0.0001</izz>
                    </inertia>
                </inertial>
            </link>
            
            <link name='base_link'>
                <pose relative_to='base_footprint'>0 0 0.010 0 0 0</pose>
                <inertial>
                    <mass>1.3729096</mass>
                    <inertia>
                        <ixx>0.0117754808</ixx><ixy>0</ixy><ixz>0</ixz>
                        <iyy>0.0117754808</iyy><iyz>0</iyz><izz>0.0184</izz>
                    </inertia>
                </inertial>
                <!-- Main body - circular base -->
                <visual name='base_visual'>
                    <pose>0 0 0.047 0 0 0</pose>
                    <geometry>
                        <cylinder><radius>0.133</radius><length>0.094</length></cylinder>
                    </geometry>
                    <material>
                        <ambient>0.1 0.1 0.1 1</ambient>
                        <diffuse>0.1 0.1 0.1 1</diffuse>
                    </material>
                </visual>
                <!-- Top plate -->
                <visual name='top_plate_visual'>
                    <pose>0 0 0.12 0 0 0</pose>
                    <geometry>
                        <cylinder><radius>0.133</radius><length>0.02</length></cylinder>
                    </geometry>
                    <material>
                        <ambient>0.3 0.3 0.3 1</ambient>
                        <diffuse>0.3 0.3 0.3 1</diffuse>
                    </material>
                </visual>
                <collision name='base_collision'>
                    <pose>0 0 0.047 0 0 0</pose>
                    <geometry>
                        <cylinder><radius>0.133</radius><length>0.094</length></cylinder>
                    </geometry>
                </collision>
                
                <!-- IMU Sensor -->
                <sensor name='imu_sensor' type='imu'>
                    <always_on>true</always_on>
                    <update_rate>50</update_rate>
                    <topic>imu</topic>
                    <imu>
                        <enable_orientation>false</enable_orientation>
                        <angular_velocity>
                            <x><noise type='gaussian'><mean>0</mean><stddev>0.0</stddev></noise></x>
                            <y><noise type='gaussian'><mean>0</mean><stddev>0.0</stddev></noise></y>
                            <z><noise type='gaussian'><mean>0</mean><stddev>0.0</stddev></noise></z>
                        </angular_velocity>
                        <linear_acceleration>
                            <x><noise type='gaussian'><mean>0</mean><stddev>0.0</stddev></noise></x>
                            <y><noise type='gaussian'><mean>0</mean><stddev>0.0</stddev></noise></y>
                            <z><noise type='gaussian'><mean>0</mean><stddev>0.0</stddev></noise></z>
                        </linear_acceleration>
                    </imu>
                </sensor>
            </link>
            
            <!-- Camera Link -->
            <link name='camera_link'>
                <pose relative_to='base_link'>0.12 0 0.20 0 -0.15 0</pose>
                <inertial>
                    <mass>0.035</mass>
                    <inertia>
                        <ixx>0.00001</ixx><ixy>0</ixy><ixz>0</ixz>
                        <iyy>0.00001</iyy><iyz>0</iyz><izz>0.00001</izz>
                    </inertia>
                </inertial>
                <visual name='camera_visual'>
                    <geometry>
                        <box><size>0.02 0.06 0.02</size></box>
                    </geometry>
                    <material>
                        <ambient>0.0 0.0 0.8 1</ambient>
                        <diffuse>0.0 0.0 0.8 1</diffuse>
                    </material>
                </visual>
                <sensor name='camera' type='camera'>
                    <always_on>true</always_on>
                    <update_rate>30</update_rate>
                    <topic>camera/image</topic>
                    <camera>
                        <horizontal_fov>1.0472</horizontal_fov>
                        <image>
                            <width>640</width>
                            <height>480</height>
                            <format>R8G8B8</format>
                        </image>
                        <clip><near>0.1</near><far>100</far></clip>
                    </camera>
                </sensor>
            </link>
            
            <!-- Left Wheel -->
            <link name='wheel_left_link'>
                <pose relative_to='base_link'>0 0.144 0.033 -1.5708 0 0</pose>
                <inertial>
                    <mass>0.1</mass>
                    <inertia>
                        <ixx>0.0001</ixx><ixy>0</ixy><ixz>0</ixz>
                        <iyy>0.0001</iyy><iyz>0</iyz><izz>0.0001</izz>
                    </inertia>
                </inertial>
                <visual name='wheel_left_visual'>
                    <geometry>
                        <cylinder><radius>0.033</radius><length>0.018</length></cylinder>
                    </geometry>
                    <material>
                        <ambient>0.2 0.2 0.2 1</ambient>
                        <diffuse>0.2 0.2 0.2 1</diffuse>
                    </material>
                </visual>
                <collision name='wheel_left_collision'>
                    <geometry>
                        <cylinder><radius>0.033</radius><length>0.018</length></cylinder>
                    </geometry>
                    <surface>
                        <friction>
                            <ode><mu>1.0</mu><mu2>1.0</mu2></ode>
                        </friction>
                    </surface>
                </collision>
            </link>
            
            <!-- Right Wheel -->
            <link name='wheel_right_link'>
                <pose relative_to='base_link'>0 -0.144 0.033 -1.5708 0 0</pose>
                <inertial>
                    <mass>0.1</mass>
                    <inertia>
                        <ixx>0.0001</ixx><ixy>0</ixy><ixz>0</ixz>
                        <iyy>0.0001</iyy><iyz>0</iyz><izz>0.0001</izz>
                    </inertia>
                </inertial>
                <visual name='wheel_right_visual'>
                    <geometry>
                        <cylinder><radius>0.033</radius><length>0.018</length></cylinder>
                    </geometry>
                    <material>
                        <ambient>0.2 0.2 0.2 1</ambient>
                        <diffuse>0.2 0.2 0.2 1</diffuse>
                    </material>
                </visual>
                <collision name='wheel_right_collision'>
                    <geometry>
                        <cylinder><radius>0.033</radius><length>0.018</length></cylinder>
                    </geometry>
                    <surface>
                        <friction>
                            <ode><mu>1.0</mu><mu2>1.0</mu2></ode>
                        </friction>
                    </surface>
                </collision>
            </link>
            
            <!-- Caster Wheels -->
            <link name='caster_back_left_link'>
                <pose relative_to='base_link'>-0.090 0.047 0.005 0 0 0</pose>
                <inertial>
                    <mass>0.01</mass>
                    <inertia>
                        <ixx>0.00001</ixx><ixy>0</ixy><ixz>0</ixz>
                        <iyy>0.00001</iyy><iyz>0</iyz><izz>0.00001</izz>
                    </inertia>
                </inertial>
                <visual name='caster_visual'>
                    <geometry><sphere><radius>0.005</radius></sphere></geometry>
                    <material>
                        <ambient>0.2 0.2 0.2 1</ambient>
                        <diffuse>0.2 0.2 0.2 1</diffuse>
                    </material>
                </visual>
                <collision name='caster_collision'>
                    <geometry><sphere><radius>0.005</radius></sphere></geometry>
                    <surface>
                        <friction><ode><mu>0.0</mu><mu2>0.0</mu2></ode></friction>
                    </surface>
                </collision>
            </link>
            
            <link name='caster_back_right_link'>
                <pose relative_to='base_link'>-0.090 -0.047 0.005 0 0 0</pose>
                <inertial>
                    <mass>0.01</mass>
                    <inertia>
                        <ixx>0.00001</ixx><ixy>0</ixy><ixz>0</ixz>
                        <iyy>0.00001</iyy><iyz>0</iyz><izz>0.00001</izz>
                    </inertia>
                </inertial>
                <visual name='caster_visual'>
                    <geometry><sphere><radius>0.005</radius></sphere></geometry>
                    <material>
                        <ambient>0.2 0.2 0.2 1</ambient>
                        <diffuse>0.2 0.2 0.2 1</diffuse>
                    </material>
                </visual>
                <collision name='caster_collision'>
                    <geometry><sphere><radius>0.005</radius></sphere></geometry>
                    <surface>
                        <friction><ode><mu>0.0</mu><mu2>0.0</mu2></ode></friction>
                    </surface>
                </collision>
            </link>
            
            <!-- Joints -->
            <joint name='base_joint' type='fixed'>
                <parent>base_footprint</parent>
                <child>base_link</child>
            </joint>
            
            <joint name='camera_joint' type='fixed'>
                <parent>base_link</parent>
                <child>camera_link</child>
            </joint>
            
            <joint name='wheel_left_joint' type='revolute'>
                <parent>base_link</parent>
                <child>wheel_left_link</child>
                <axis>
                    <xyz>0 0 1</xyz>
                    <limit><lower>-1.79769e+308</lower><upper>1.79769e+308</upper></limit>
                </axis>
            </joint>
            
            <joint name='wheel_right_joint' type='revolute'>
                <parent>base_link</parent>
                <child>wheel_right_link</child>
                <axis>
                    <xyz>0 0 1</xyz>
                    <limit><lower>-1.79769e+308</lower><upper>1.79769e+308</upper></limit>
                </axis>
            </joint>
            
            <joint name='caster_back_left_joint' type='ball'>
                <parent>base_link</parent>
                <child>caster_back_left_link</child>
            </joint>
            
            <joint name='caster_back_right_joint' type='ball'>
                <parent>base_link</parent>
                <child>caster_back_right_link</child>
            </joint>
            
            <!-- Diff Drive Plugin -->
            <plugin filename='libignition-gazebo-diff-drive-system.so' name='ignition::gazebo::systems::DiffDrive'>
                <left_joint>wheel_left_joint</left_joint>
                <right_joint>wheel_right_joint</right_joint>
                <wheel_separation>0.287</wheel_separation>
                <wheel_radius>0.033</wheel_radius>
                <odom_publish_frequency>30</odom_publish_frequency>
                <topic>cmd_vel</topic>
                <odom_topic>odom</odom_topic>
                <frame_id>odom</frame_id>
                <child_frame_id>base_footprint</child_frame_id>
            </plugin>
        </model>
"""


def generate_turtlebot3_burger_sdf() -> str:
    """Generate TurtleBot3 Burger model with IMU and camera sensors (using simple shapes)."""
    return """
        <model name='turtlebot3_burger' canonical_link='base_footprint'>
            <pose>0 0 0 0 0 0</pose>
            
            <link name='base_footprint'>
                <pose>0 0 0 0 0 0</pose>
                <inertial>
                    <mass>0.001</mass>
                    <inertia>
                        <ixx>0.0001</ixx><ixy>0</ixy><ixz>0</ixz>
                        <iyy>0.0001</iyy><iyz>0</iyz><izz>0.0001</izz>
                    </inertia>
                </inertial>
            </link>
            
            <link name='base_link'>
                <pose relative_to='base_footprint'>0 0 0.010 0 0 0</pose>
                <inertial>
                    <mass>0.9</mass>
                    <inertia>
                        <ixx>0.0082</ixx><ixy>0</ixy><ixz>0</ixz>
                        <iyy>0.0082</iyy><iyz>0</iyz><izz>0.0120</izz>
                    </inertia>
                </inertial>
                <!-- Main body - circular base -->
                <visual name='base_visual'>
                    <pose>0 0 0.047 0 0 0</pose>
                    <geometry>
                        <cylinder><radius>0.07</radius><length>0.094</length></cylinder>
                    </geometry>
                    <material>
                        <ambient>0.1 0.1 0.1 1</ambient>
                        <diffuse>0.1 0.1 0.1 1</diffuse>
                    </material>
                </visual>
                <!-- Top plates (stacked) -->
                <visual name='top_plate1_visual'>
                    <pose>0 0 0.10 0 0 0</pose>
                    <geometry>
                        <cylinder><radius>0.07</radius><length>0.01</length></cylinder>
                    </geometry>
                    <material>
                        <ambient>0.3 0.3 0.3 1</ambient>
                        <diffuse>0.3 0.3 0.3 1</diffuse>
                    </material>
                </visual>
                <visual name='top_plate2_visual'>
                    <pose>0 0 0.13 0 0 0</pose>
                    <geometry>
                        <cylinder><radius>0.07</radius><length>0.01</length></cylinder>
                    </geometry>
                    <material>
                        <ambient>0.3 0.3 0.3 1</ambient>
                        <diffuse>0.3 0.3 0.3 1</diffuse>
                    </material>
                </visual>
                <collision name='base_collision'>
                    <pose>0 0 0.070 0 0 0</pose>
                    <geometry>
                        <cylinder><radius>0.07</radius><length>0.140</length></cylinder>
                    </geometry>
                </collision>
                
                <!-- IMU Sensor -->
                <sensor name='imu_sensor' type='imu'>
                    <always_on>true</always_on>
                    <update_rate>50</update_rate>
                    <topic>imu</topic>
                    <imu>
                        <enable_orientation>false</enable_orientation>
                        <angular_velocity>
                            <x><noise type='gaussian'><mean>0</mean><stddev>0.0</stddev></noise></x>
                            <y><noise type='gaussian'><mean>0</mean><stddev>0.0</stddev></noise></y>
                            <z><noise type='gaussian'><mean>0</mean><stddev>0.0</stddev></noise></z>
                        </angular_velocity>
                        <linear_acceleration>
                            <x><noise type='gaussian'><mean>0</mean><stddev>0.0</stddev></noise></x>
                            <y><noise type='gaussian'><mean>0</mean><stddev>0.0</stddev></noise></y>
                            <z><noise type='gaussian'><mean>0</mean><stddev>0.0</stddev></noise></z>
                        </linear_acceleration>
                    </imu>
                </sensor>
            </link>
            
            <!-- Camera Link (added for visual navigation) -->
            <link name='camera_link'>
                <pose relative_to='base_link'>0.15 0 0.30 0 0 0</pose>
                <inertial>
                    <mass>0.02</mass>
                    <inertia>
                        <ixx>0.00001</ixx><ixy>0</ixy><ixz>0</ixz>
                        <iyy>0.00001</iyy><iyz>0</iyz><izz>0.00001</izz>
                    </inertia>
                </inertial>
                <visual name='camera_visual'>
                    <geometry>
                        <box><size>0.015 0.030 0.020</size></box>
                    </geometry>
                    <material>
                        <ambient>0.0 0.0 0.8 1</ambient>
                        <diffuse>0.0 0.0 0.8 1</diffuse>
                    </material>
                </visual>
                <sensor name='camera' type='camera'>
                    <always_on>true</always_on>
                    <update_rate>30</update_rate>
                    <topic>camera/image</topic>
                    <camera>
                        <horizontal_fov>1.0472</horizontal_fov>
                        <image>
                            <width>640</width>
                            <height>480</height>
                            <format>R8G8B8</format>
                        </image>
                        <clip><near>0.1</near><far>100</far></clip>
                    </camera>
                </sensor>
            </link>
            
            <!-- Left Wheel -->
            <link name='wheel_left_link'>
                <pose relative_to='base_link'>0 0.08 0.033 -1.5708 0 0</pose>
                <inertial>
                    <mass>0.05</mass>
                    <inertia>
                        <ixx>0.00001</ixx><ixy>0</ixy><ixz>0</ixz>
                        <iyy>0.00001</iyy><iyz>0</iyz><izz>0.00001</izz>
                    </inertia>
                </inertial>
                <visual name='wheel_left_visual'>
                    <geometry>
                        <cylinder><radius>0.033</radius><length>0.018</length></cylinder>
                    </geometry>
                    <material>
                        <ambient>0.2 0.2 0.2 1</ambient>
                        <diffuse>0.2 0.2 0.2 1</diffuse>
                    </material>
                </visual>
                <collision name='wheel_left_collision'>
                    <geometry>
                        <cylinder><radius>0.033</radius><length>0.018</length></cylinder>
                    </geometry>
                    <surface>
                        <friction>
                            <ode><mu>1.0</mu><mu2>1.0</mu2></ode>
                        </friction>
                    </surface>
                </collision>
            </link>
            
            <!-- Right Wheel -->
            <link name='wheel_right_link'>
                <pose relative_to='base_link'>0 -0.08 0.033 -1.5708 0 0</pose>
                <inertial>
                    <mass>0.05</mass>
                    <inertia>
                        <ixx>0.00001</ixx><ixy>0</ixy><ixz>0</ixz>
                        <iyy>0.00001</iyy><iyz>0</iyz><izz>0.00001</izz>
                    </inertia>
                </inertial>
                <visual name='wheel_right_visual'>
                    <geometry>
                        <cylinder><radius>0.033</radius><length>0.018</length></cylinder>
                    </geometry>
                    <material>
                        <ambient>0.2 0.2 0.2 1</ambient>
                        <diffuse>0.2 0.2 0.2 1</diffuse>
                    </material>
                </visual>
                <collision name='wheel_right_collision'>
                    <geometry>
                        <cylinder><radius>0.033</radius><length>0.018</length></cylinder>
                    </geometry>
                    <surface>
                        <friction>
                            <ode><mu>1.0</mu><mu2>1.0</mu2></ode>
                        </friction>
                    </surface>
                </collision>
            </link>
            
            <!-- Caster Ball -->
            <link name='caster_link'>
                <pose relative_to='base_link'>-0.045 0 0.005 0 0 0</pose>
                <inertial>
                    <mass>0.01</mass>
                    <inertia>
                        <ixx>0.00001</ixx><ixy>0</ixy><ixz>0</ixz>
                        <iyy>0.00001</iyy><iyz>0</iyz><izz>0.00001</izz>
                    </inertia>
                </inertial>
                <visual name='caster_visual'>
                    <geometry><sphere><radius>0.005</radius></sphere></geometry>
                    <material>
                        <ambient>0.2 0.2 0.2 1</ambient>
                        <diffuse>0.2 0.2 0.2 1</diffuse>
                    </material>
                </visual>
                <collision name='caster_collision'>
                    <geometry><sphere><radius>0.005</radius></sphere></geometry>
                    <surface>
                        <friction><ode><mu>0.0</mu><mu2>0.0</mu2></ode></friction>
                    </surface>
                </collision>
            </link>
            
            <!-- Joints -->
            <joint name='base_joint' type='fixed'>
                <parent>base_footprint</parent>
                <child>base_link</child>
            </joint>
            
            <joint name='camera_joint' type='fixed'>
                <parent>base_link</parent>
                <child>camera_link</child>
            </joint>
            
            <joint name='wheel_left_joint' type='revolute'>
                <parent>base_link</parent>
                <child>wheel_left_link</child>
                <axis>
                    <xyz>0 0 1</xyz>
                    <limit><lower>-1.79769e+308</lower><upper>1.79769e+308</upper></limit>
                </axis>
            </joint>
            
            <joint name='wheel_right_joint' type='revolute'>
                <parent>base_link</parent>
                <child>wheel_right_link</child>
                <axis>
                    <xyz>0 0 1</xyz>
                    <limit><lower>-1.79769e+308</lower><upper>1.79769e+308</upper></limit>
                </axis>
            </joint>
            
            <joint name='caster_joint' type='ball'>
                <parent>base_link</parent>
                <child>caster_link</child>
            </joint>
            
            <!-- Diff Drive Plugin -->
            <plugin filename='libignition-gazebo-diff-drive-system.so' name='ignition::gazebo::systems::DiffDrive'>
                <left_joint>wheel_left_joint</left_joint>
                <right_joint>wheel_right_joint</right_joint>
                <wheel_separation>0.160</wheel_separation>
                <wheel_radius>0.033</wheel_radius>
                <odom_publish_frequency>30</odom_publish_frequency>
                <topic>cmd_vel</topic>
                <odom_topic>odom</odom_topic>
                <frame_id>odom</frame_id>
                <child_frame_id>base_footprint</child_frame_id>
            </plugin>
        </model>
"""


def generate_vehicle_blue_sdf() -> str:
    """Generate the original vehicle_blue model."""
    return """
        <model name='vehicle_blue' canonical_link='chassis'>
            <pose relative_to='world'>0.0 0.0 0 0 0 0</pose>
            <frame name="lidar_frame" attached_to='chassis'>
                <pose>1.0 0 0.0 0 0 0</pose>
            </frame>

            <frame name="camera_frame" attached_to="chassis">
              <pose>1.0 0 0.0 0 0 0</pose>
            </frame>

            <link name='chassis'>
                <pose relative_to='__model__'>0.5 0 0.4 0 0 0</pose>
                <inertial>
                    <mass>1.14395</mass>
                    <inertia>
                        <ixx>0.095329</ixx>
                        <ixy>0</ixy>
                        <ixz>0</ixz>
                        <iyy>0.381317</iyy>
                        <iyz>0</iyz>
                        <izz>0.476646</izz>
                    </inertia>
                </inertial>
                <visual name='visual'>
                    <geometry>
                        <box>
                            <size>2.0 1.0 0.5</size>
                        </box>
                    </geometry>
                    <material>
                        <ambient>0.0 0.0 1.0 1</ambient>
                        <diffuse>0.0 0.0 1.0 1</diffuse>
                        <specular>0.0 0.0 1.0 1</specular>
                    </material>
                </visual>
                <collision name='collision'>
                    <geometry>
                        <box>
                            <size>2.0 1.0 0.5</size>
                        </box>
                    </geometry>
                </collision>
                <sensor name="imu_sensor" type="imu">
                    <imu>
                      <enable_orientation>false</enable_orientation>
                      <angular_velocity>
                        <x><noise type="gaussian"><mean>0.0</mean><stddev>0.0</stddev><bias_mean>0.0</bias_mean><bias_stddev>0.0</bias_stddev><dynamic_bias_stddev>0.0</dynamic_bias_stddev><dynamic_bias_correlation_time>3600</dynamic_bias_correlation_time></noise></x>
                        <y><noise type="gaussian"><mean>0.0</mean><stddev>0.0</stddev><bias_mean>0.0</bias_mean><bias_stddev>0.0</bias_stddev><dynamic_bias_stddev>0.0</dynamic_bias_stddev><dynamic_bias_correlation_time>3600</dynamic_bias_correlation_time></noise></y>
                        <z><noise type="gaussian"><mean>0.0</mean><stddev>0.0</stddev><bias_mean>0.0</bias_mean><bias_stddev>0.0</bias_stddev><dynamic_bias_stddev>0.0</dynamic_bias_stddev><dynamic_bias_correlation_time>3600</dynamic_bias_correlation_time></noise></z>
                      </angular_velocity>
                      <linear_acceleration>
                        <x><noise type="gaussian"><mean>0.0</mean><stddev>0.0</stddev><bias_mean>0.0</bias_mean><bias_stddev>0.0</bias_stddev><dynamic_bias_stddev>0.0</dynamic_bias_stddev><dynamic_bias_correlation_time>3600</dynamic_bias_correlation_time></noise></x>
                        <y><noise type="gaussian"><mean>0.0</mean><stddev>0.0</stddev><bias_mean>0.0</bias_mean><bias_stddev>0.0</bias_stddev><dynamic_bias_stddev>0.0</dynamic_bias_stddev><dynamic_bias_correlation_time>3600</dynamic_bias_correlation_time></noise></y>
                        <z><noise type="gaussian"><mean>0.0</mean><stddev>0.0</stddev><bias_mean>0.0</bias_mean><bias_stddev>0.0</bias_stddev><dynamic_bias_stddev>0.0</dynamic_bias_stddev><dynamic_bias_correlation_time>3600</dynamic_bias_correlation_time></noise></z>
                      </linear_acceleration>
                    </imu>
                    <always_on>1</always_on>
                    <update_rate>50</update_rate>
                    <visualize>true</visualize>
                    <topic>vehicle_blue/imu</topic>
                </sensor>
                <sensor name="front_camera" type="camera">
                  <pose relative_to="camera_frame">0 0 0 0 0 0</pose>
                  <topic>vehicle_blue/camera/image</topic>
                  <update_rate>30</update_rate>
                  <camera>
                    <horizontal_fov>1.0472</horizontal_fov>
                    <image>
                      <width>640</width>
                      <height>480</height>
                      <format>R8G8B8</format>
                    </image>
                    <clip>
                      <near>0.1</near>
                      <far>80.0</far>
                    </clip>
                    <noise>
                      <type>gaussian</type>
                      <mean>0.0</mean>
                      <stddev>0.05</stddev>
                    </noise>
                  </camera>
                  <always_on>1</always_on>
                  <visualize>true</visualize>
                </sensor>
            </link>
            <link name='left_wheel'>
                <pose relative_to="chassis">-0.5 0.6 0 -1.5707 0 0</pose>
                <inertial>
                    <mass>1</mass>
                    <inertia>
                        <ixx>0.043333</ixx>
                        <ixy>0</ixy>
                        <ixz>0</ixz>
                        <iyy>0.043333</iyy>
                        <iyz>0</iyz>
                        <izz>0.08</izz>
                    </inertia>
                </inertial>
                <visual name='visual'>
                    <geometry>
                        <cylinder>
                            <radius>0.4</radius>
                            <length>0.2</length>
                        </cylinder>
                    </geometry>
                    <material>
                        <ambient>1.0 0.0 0.0 1</ambient>
                        <diffuse>1.0 0.0 0.0 1</diffuse>
                        <specular>1.0 0.0 0.0 1</specular>
                    </material>
                </visual>
                <collision name='collision'>
                    <geometry>
                        <cylinder>
                            <radius>0.4</radius>
                            <length>0.2</length>
                        </cylinder>
                    </geometry>
                </collision>
            </link>
            <link name='right_wheel'>
                <pose relative_to="chassis">-0.5 -0.6 0 -1.5707 0 0</pose>
                <inertial>
                    <mass>1</mass>
                    <inertia>
                        <ixx>0.043333</ixx>
                        <ixy>0</ixy>
                        <ixz>0</ixz>
                        <iyy>0.043333</iyy>
                        <iyz>0</iyz>
                        <izz>0.08</izz>
                    </inertia>
                </inertial>
                <visual name='visual'>
                    <geometry>
                        <cylinder>
                            <radius>0.4</radius>
                            <length>0.2</length>
                        </cylinder>
                    </geometry>
                    <material>
                        <ambient>1.0 0.0 0.0 1</ambient>
                        <diffuse>1.0 0.0 0.0 1</diffuse>
                        <specular>1.0 0.0 0.0 1</specular>
                    </material>
                </visual>
                <collision name='collision'>
                    <geometry>
                        <cylinder>
                            <radius>0.4</radius>
                            <length>0.2</length>
                        </cylinder>
                    </geometry>
                </collision>
            </link>
            <frame name="caster_frame" attached_to='chassis'>
              <pose>0.8 0 -0.2 0 0 0</pose>
            </frame>
            <link name='caster'>
                <pose relative_to='caster_frame'/>
                <inertial>
                    <mass>1</mass>
                    <inertia>
                        <ixx>0.016</ixx>
                        <ixy>0</ixy>
                        <ixz>0</ixz>
                        <iyy>0.016</iyy>
                        <iyz>0</iyz>
                        <izz>0.016</izz>
                    </inertia>
                </inertial>
                <visual name='visual'>
                    <geometry>
                        <sphere>
                            <radius>0.2</radius>
                        </sphere>
                    </geometry>
                    <material>
                        <ambient>0.0 1 0.0 1</ambient>
                        <diffuse>0.0 1 0.0 1</diffuse>
                        <specular>0.0 1 0.0 1</specular>
                    </material>
                </visual>
                <collision name='collision'>
                    <geometry>
                        <sphere>
                            <radius>0.2</radius>
                        </sphere>
                    </geometry>
                </collision>
            </link>

            <joint name='left_wheel_joint' type='revolute'>
                <pose relative_to='left_wheel'/>
                <parent>chassis</parent>
                <child>left_wheel</child>
                <axis>
                    <xyz expressed_in='__model__'>0 1 0</xyz>
                    <limit>
                        <lower>-1.79769e+308</lower>
                        <upper>1.79769e+308</upper>
                    </limit>
                </axis>
            </joint>
            <joint name='right_wheel_joint' type='revolute'>
                <pose relative_to='right_wheel'/>
                <parent>chassis</parent>
                <child>right_wheel</child>
                <axis>
                    <xyz expressed_in='__model__'>0 1 0</xyz>
                    <limit>
                        <lower>-1.79769e+308</lower>
                        <upper>1.79769e+308</upper>
                    </limit>
                </axis>
            </joint>
            <joint name='caster_wheel' type='ball'>
                <parent>chassis</parent>
                <child>caster</child>
            </joint>
            <plugin
                filename="libignition-gazebo-diff-drive-system.so"
                name="ignition::gazebo::systems::DiffDrive">
                <left_joint>left_wheel_joint</left_joint>
                <right_joint>right_wheel_joint</right_joint>
                <wheel_separation>1.2</wheel_separation>
                <wheel_radius>0.4</wheel_radius>
                <odom_publish_frequency>1</odom_publish_frequency>
                <topic>vehicle_blue/cmd_vel</topic>
            </plugin>
        </model>
"""


def get_robot_sdf(robot_type: str) -> str:
    """Get robot SDF based on type."""
    if robot_type == "vehicle_blue":
        return generate_vehicle_blue_sdf()
    elif robot_type == "turtlebot3_waffle":
        return generate_turtlebot3_waffle_sdf()
    elif robot_type == "turtlebot3_burger":
        return generate_turtlebot3_burger_sdf()
    elif robot_type == "none":
        return ""
    else:
        return generate_vehicle_blue_sdf()


def generate_world_sdf(
        num_landmarks: int,
        min_radius: float = 4.0,
        max_radius: float = 15.0,
        seed: int = None,
        landmark_height: float = 1.0,
        landmark_radius: float = 0.2,
        robot: str = "vehicle_blue"
) -> Tuple[str, List[dict]]:
    """
    Generate complete SDF world file with landmarks.

    Args:
        num_landmarks: Number of landmarks to place
        min_radius: Minimum distance from origin
        max_radius: Maximum distance from origin
        seed: Random seed for reproducibility
        landmark_height: Height of landmark cylinders
        landmark_radius: Radius of landmark cylinders
        robot: Robot model to include ("vehicle_blue", "turtlebot3_waffle", "turtlebot3_burger", "none")

    Returns:
        Tuple of (SDF world file string, landmark data list)
    """
    positions = generate_landmark_positions(
        num_landmarks, min_radius, max_radius, seed=seed
    )

    # Generate unique colors for all landmarks
    colors = generate_unique_colors(num_landmarks)

    # Generate landmark models and collect data for export
    landmarks_sdf = ""
    landmark_data = []

    for i, (x, y) in enumerate(positions):
        color = colors[i]
        color_name = color["name"]
        color_rgba = color["rgba"]

        # Create unique name with index
        name = f"{i}_{color_name}"
        landmarks_sdf += generate_landmark_sdf(
            name, x, y, color_name, color_rgba,
            height=landmark_height, radius=landmark_radius
        )

        # Export data compatible with visual detector
        # Use lower S/V thresholds to handle Gazebo lighting variations
        landmark_data.append({
            "id": i + 1,  # 1-indexed for detector compatibility
            "name": color_name,
            "x": round(x, 3),
            "y": round(y, 3),
            "z": round(landmark_height / 2, 3),
            "rgb": color["rgb"],
            "lower": [color["h_lower"], 50, 30],   # HSV lower bound (relaxed S, V)
            "upper": [color["h_upper"], 255, 255],  # HSV upper bound
        })

    # Build the complete world SDF
    world_sdf = f'''<?xml version="1.0" ?>
<sdf version="1.7">
    <world name="project_world">
        <physics name="1ms" type="ignored">
            <max_step_size>0.001</max_step_size>
            <real_time_factor>1.0</real_time_factor>
        </physics>
        <plugin
            filename="libignition-gazebo-physics-system.so"
            name="ignition::gazebo::systems::Physics">
        </plugin>
        <plugin
            filename="libignition-gazebo-user-commands-system.so"
            name="ignition::gazebo::systems::UserCommands">
        </plugin>
        <plugin
            filename="libignition-gazebo-scene-broadcaster-system.so"
            name="ignition::gazebo::systems::SceneBroadcaster">
        </plugin>
        <plugin filename="libignition-gazebo-imu-system.so"
                name="ignition::gazebo::systems::Imu">
        </plugin>
        <plugin
          filename="libignition-gazebo-sensors-system.so"
          name="ignition::gazebo::systems::Sensors">
          <render_engine>ogre2</render_engine>
        </plugin>

        <gui fullscreen="0">
            <plugin filename="GzScene3D" name="3D View">
                <ignition-gui>
                <title>3D View</title>
                <property type="bool" key="showTitleBar">false</property>
                <property type="string" key="state">docked</property>
                </ignition-gui>
                <engine>ogre2</engine>
                <scene>scene</scene>
                <ambient_light>0.4 0.4 0.4</ambient_light>
                <background_color>0.8 0.8 0.8</background_color>
            </plugin>

            <plugin filename="WorldControl" name="World control">
                <ignition-gui>
                <title>World control</title>
                <property type="bool" key="showTitleBar">false</property>
                <property type="bool" key="resizable">false</property>
                <property type="double" key="height">72</property>
                <property type="double" key="width">121</property>
                <property type="double" key="z">1</property>
                <property type="string" key="state">floating</property>
                <anchors target="3D View">
                    <line own="left" target="left"/>
                    <line own="bottom" target="bottom"/>
                </anchors>
                </ignition-gui>
                <play_pause>true</play_pause>
                <step>true</step>
                <start_paused>true</start_paused>
                <service>/world/project_world/control</service>
                <stats_topic>/world/project_world/stats</stats_topic>
            </plugin>

            <plugin filename="WorldStats" name="World stats">
                <ignition-gui>
                <title>World stats</title>
                <property type="bool" key="showTitleBar">false</property>
                <property type="bool" key="resizable">false</property>
                <property type="double" key="height">110</property>
                <property type="double" key="width">290</property>
                <property type="double" key="z">1</property>
                <property type="string" key="state">floating</property>
                <anchors target="3D View">
                    <line own="right" target="right"/>
                    <line own="bottom" target="bottom"/>
                </anchors>
                </ignition-gui>
                <sim_time>true</sim_time>
                <real_time>true</real_time>
                <real_time_factor>true</real_time_factor>
                <iterations>true</iterations>
                <topic>/world/project_world/stats</topic>
            </plugin>

            <plugin filename="EntityTree" name="Entity tree">
            </plugin>
        </gui>

        <light type="directional" name="sun">
            <cast_shadows>true</cast_shadows>
            <pose>0 0 10 0 0 0</pose>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
            <attenuation>
                <range>1000</range>
                <constant>0.9</constant>
                <linear>0.01</linear>
                <quadratic>0.001</quadratic>
            </attenuation>
            <direction>-0.5 0.1 -0.9</direction>
        </light>

        <model name="ground_plane">
            <static>true</static>
            <link name="link">
                <collision name="collision">
                <geometry>
                    <plane>
                    <normal>0 0 1</normal>
                    </plane>
                </geometry>
                </collision>
                <visual name="visual">
                <geometry>
                    <plane>
                    <normal>0 0 1</normal>
                    <size>100 100</size>
                    </plane>
                </geometry>
                <material>
                    <ambient>0.8 0.8 0.8 1</ambient>
                    <diffuse>0.8 0.8 0.8 1</diffuse>
                    <specular>0.8 0.8 0.8 1</specular>
                </material>
                </visual>
            </link>
        </model>

        <!-- Generated Landmarks ({num_landmarks} total) -->
{landmarks_sdf}
{get_robot_sdf(robot)}
    </world>
</sdf>
'''
    return world_sdf, landmark_data


def main():
    parser = argparse.ArgumentParser(
        description="Generate Gazebo SDF world file with configurable landmarks"
    )
    parser.add_argument(
        "-n", "--num-landmarks",
        type=int,
        default=4,
        help="Number of landmarks to generate (default: 4)"
    )
    parser.add_argument(
        "-o", "--output",
        type=str,
        default="generated_world.sdf",
        help="Output file path (default: generated_world.sdf)"
    )
    parser.add_argument(
        "--min-radius",
        type=float,
        default=4.0,
        help="Minimum distance of landmarks from origin (default: 4.0)"
    )
    parser.add_argument(
        "--max-radius",
        type=float,
        default=15.0,
        help="Maximum distance of landmarks from origin (default: 15.0)"
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Random seed for reproducible landmark placement"
    )
    parser.add_argument(
        "--landmark-height",
        type=float,
        default=1.0,
        help="Height of landmark cylinders (default: 1.0)"
    )
    parser.add_argument(
        "--landmark-radius",
        type=float,
        default=0.2,
        help="Radius of landmark cylinders (default: 0.2)"
    )
    parser.add_argument(
        "--export-map",
        type=str,
        default=None,
        help="Export landmark positions to JSON file (for EKF ground truth)"
    )
    parser.add_argument(
        "--no-robot",
        action="store_true",
        help="Don't include robot in world (spawn separately via launch file)"
    )
    parser.add_argument(
        "--robot",
        type=str,
        choices=["vehicle_blue", "turtlebot3_waffle", "turtlebot3_burger", "none"],
        default="vehicle_blue",
        help="Robot model to include (default: vehicle_blue)"
    )

    args = parser.parse_args()

    # Set random seed early if provided (affects both colors and positions)
    if args.seed is not None:
        random.seed(args.seed)

    # Handle --no-robot flag
    robot_type = "none" if args.no_robot else args.robot

    world_sdf, landmark_data = generate_world_sdf(
        num_landmarks=args.num_landmarks,
        min_radius=args.min_radius,
        max_radius=args.max_radius,
        seed=args.seed,
        landmark_height=args.landmark_height,
        landmark_radius=args.landmark_radius,
        robot=robot_type
    )

    with open(args.output, "w") as f:
        f.write(world_sdf)

    robot_msg = f" with {robot_type}" if robot_type != "none" else " (no robot)"
    print(f"Generated world with {args.num_landmarks} landmarks{robot_msg}: {args.output}")

    # Export landmark map if requested
    if args.export_map:
        import json
        map_data = {
            "landmarks": landmark_data,
            "num_landmarks": args.num_landmarks,
            "seed": args.seed
        }
        with open(args.export_map, "w") as f:
            json.dump(map_data, f, indent=2)
        print(f"Exported landmark map: {args.export_map}")


if __name__ == "__main__":
    main()