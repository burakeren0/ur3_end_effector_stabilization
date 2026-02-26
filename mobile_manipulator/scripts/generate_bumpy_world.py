import random
import os

# Dünyanın kaydedileceği yol (Paketimizin içine)
os.makedirs("src/mobile_manipulator/worlds", exist_ok=True)
output_path = "src/mobile_manipulator/worlds/bumpy_world.sdf"

sdf_content = """<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="bumpy_world">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation><range>1000</range><constant>0.9</constant><linear>0.01</linear><quadratic>0.001</quadratic></attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.6 0.6 0.6 1</ambient><diffuse>0.6 0.6 0.6 1</diffuse></material>
        </visual>
      </link>
    </model>
"""

# Yoğunluğu artırdık: 400 adet rastgele tümsek/taş
for i in range(400):
    # Robotun kalkış noktasından (x=1) ileriye (x=20) ve sağa-sola (y=-3.5, 3.5) yayılmış taşlar
    x = random.uniform(1.0, 20.0) 
    y = random.uniform(-3.5, 3.5) 
    z = 0.0 # Yarı gömülü olması için
    
    # Boyutu artırdık: 5 cm ile 15 cm arası
    radius = random.uniform(0.05, 0.15) 
    
    sdf_content += f"""
    <model name="bump_{i}">
      <static>true</static>
      <pose>{x} {y} {z} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><sphere><radius>{radius}</radius></sphere></geometry></collision>
        <visual name="visual"><geometry><sphere><radius>{radius}</radius></sphere></geometry>
        <material><ambient>0.3 0.3 0.3 1</ambient><diffuse>0.3 0.3 0.3 1</diffuse></material>
        </visual>
      </link>
    </model>
    """

sdf_content += """
  </world>
</sdf>
"""

with open(output_path, "w") as f:
    f.write(sdf_content)

print(f"Zorlu Bozucu parkur {output_path} konumunda oluşturuldu.")
