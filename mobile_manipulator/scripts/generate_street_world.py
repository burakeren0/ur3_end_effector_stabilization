import os

os.makedirs("src/mobile_manipulator/worlds", exist_ok=True)
output_path = "src/mobile_manipulator/worlds/street_world.sdf"

sdf_content = """<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="street_world">
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
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="base_ground">
      <static>true</static>
      <pose>0 0 -0.10 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.4 0.3 0.2 1</ambient><diffuse>0.4 0.3 0.2 1</diffuse></material>
        </visual>
      </link>
    </model>
"""

def create_road_block(name, x_center, y_center, length, width=4.0):
    thickness = 0.10 # Yol kalınlığı 10 cm
    z_center = -thickness / 2.0
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x_center} {y_center} {z_center} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{length} {width} {thickness}</size></box></geometry></collision>
        <visual name="visual">
          <geometry><box><size>{length} {width} {thickness}</size></box></geometry>
          <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material>
        </visual>
      </link>
    </model>
    """

def create_bump(name, x, y, radius, length, yaw=0.0):
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} 0.0 1.570796 0 {yaw}</pose>
      <link name="link">
        <collision name="collision"><geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry></collision>
        <visual name="visual">
          <geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry>
          <material><ambient>0.8 0.7 0.1 1</ambient><diffuse>0.8 0.7 0.1 1</diffuse></material>
        </visual>
      </link>
    </model>
    """

# --- DÜZELTİLMİŞ PARKUR İNŞAASI ---
bump_r = 0.025 

# 1. Başlangıç Yolu UZATILDI: (x = -2 ile 4 arası). Robot x=0'da doğduğunda arkasında 2 metre yol olacak.
sdf_content += create_road_block("road_start", 1.0, 0, 6.0) 

# 2. Tam Genişlikte Tümsek (x=4)
sdf_content += create_bump("bump_full", 4.0, 0, bump_r, 4.0)
sdf_content += create_road_block("road_2", 5.0, 0, 2.0) # x = 4 ile 6 arası

# 3. Sol Tekerlek Tümseği (x=6)
sdf_content += create_bump("bump_left_only", 6.0, 1.0, bump_r, 2.0)
sdf_content += create_road_block("road_3", 7.0, 0, 2.0) # x = 6 ile 8 arası

# 4. Sağ Tekerlek Tümseği (x=8)
sdf_content += create_bump("bump_right_only", 8.0, -1.0, bump_r, 2.0)
sdf_content += create_road_block("road_4", 9.0, 0, 2.0) # x = 8 ile 10 arası

# 5. Tam Genişlikte Çukur (x=10.0 ile 10.15 arası 15 cm'lik boşluk)
sdf_content += create_road_block("road_5", 11.075, 0, 1.85) # x = 10.15 ile 12.0 arası

# 6. Sağ Tekerlek Çukuru (x=12.0 ile 12.15 arası boş, sola 15 cm'lik köprü)
sdf_content += create_road_block("bridge_left", 12.075, 1.0, 0.15, 2.0) 
sdf_content += create_road_block("road_6", 13.075, 0, 1.85) # x = 12.15 ile 14.0 arası

# 7. Sol Tekerlek Çukuru (x=14.0 ile 14.15 arası boş, sağa 15 cm'lik köprü)
sdf_content += create_road_block("bridge_right", 14.075, -1.0, 0.15, 2.0)
sdf_content += create_road_block("road_7", 15.575, 0, 2.85) # x = 14.15 ile 17.0 arası

# 8. Açılı Tümsek (x=17.0)
sdf_content += create_bump("bump_angled", 17.0, 0, bump_r, 4.0, yaw=0.3)
sdf_content += create_road_block("road_end", 18.5, 0, 3.0) # x = 17.0 ile 20.0 arası

sdf_content += """
  </world>
</sdf>
"""

with open(output_path, "w") as f:
    f.write(sdf_content)

print("Yol geriye doğru uzatıldı. Parkur güncellendi!")
