import math
import random
import os

# --- RAMPALAR İÇİN KÜTÜK (BOX) MODELİ ---
def create_box_model(name, x, y, z, roll, pitch, yaw, sx, sy, sz, color="0.5 0.5 0.5 1"):
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {z:.4f} {roll:.4f} {pitch:.4f} {yaw:.4f}</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
          <surface><friction><ode><mu>1.5</mu><mu2>1.5</mu2></ode></friction></surface> <!-- Düz rampa yüzeylerinde tutunmayı artırmak için sürtünmeyi yükselttim. -->
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
          <material>
            <ambient>{color}</ambient>
            <diffuse>{color}</diffuse>
          </material>
        </visual>
      </link>
    </model>
    """

# --- BUMPY.WORLD İLE AYNI KALIPTAKİ KÜRE TAŞ MODELİ ---
def create_sphere_model(name, x, y, z, roll, pitch, yaw, radius):  # Taşların görünüş ve sürtünme kalıbını bumpy.world ile eşitledim.
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {z:.4f} {roll:.4f} {pitch:.4f} {yaw:.4f}</pose>
      <link name="link">
        <collision name="collision">
          <geometry><sphere><radius>{radius:.4f}</radius></sphere></geometry>
          <surface><friction><ode><mu>100.0</mu><mu2>100.0</mu2></ode></friction></surface> <!-- Taş sürtünmesini bumpy.world ile eşitledim. -->
        </collision>
        <visual name="visual">
          <geometry><sphere><radius>{radius:.4f}</radius></sphere></geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient> <!-- Taş ortam rengini bumpy.world ile eşitledim. -->
            <diffuse>0.4 0.4 0.4 1</diffuse> <!-- Taş yayınık rengini bumpy.world ile eşitledim. -->
          </material>
        </visual>
      </link>
    </model>
    """

def transform_to_global(x_L, y_L, z_L, X_R, Y_R, Z_R, roll, pitch):
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    
    X_G = X_R + (x_L * cp) + (y_L * sp * sr) + (z_L * sp * cr)
    Y_G = Y_R + (y_L * cr) - (z_L * sr)
    Z_G = Z_R - (x_L * sp) + (y_L * cp * sr) + (z_L * cp * cr)
    
    return X_G, Y_G, Z_G

def generate_world():
    world = '<?xml version="1.0" ?>\n<sdf version="1.8">\n<world name="extreme_disturbance_parkour">\n'
    
    world += """
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"></plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"></plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"></plugin>
    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.4 0.6 0.8 1</background>
      <shadows>true</shadows>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation><range>1000</range><constant>0.9</constant><linear>0.01</linear><quadratic>0.001</quadratic></attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <surface><friction><ode><mu>1.5</mu><mu2>1.5</mu2></ode></friction></surface> <!-- Zemin düzleminde tutunmayı artırmak için sürtünmeyi yükselttim. -->
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient><diffuse>0.7 0.7 0.7 1</diffuse></material>
        </visual>
      </link>
    </model>
    """

    # --- RAMPALAR ---
    # Roll (yanal yatma) eğimleri 0.15'ten 0.07'ye düşürüldü ki robot devrilmesin.
    ramps = [
        {"name": "ramp_pitch",      "X": 8.0,  "Y": 0, "Z": 0.35, "roll": 0,     "pitch": -0.1, "L": 8.5, "W": 4, "H": 0.2, "color": "0.5 0.5 0.5 1"},  # İlk giriş tümseğini azaltmak için rampayı 5 cm aşağı aldım.
        {"name": "ramp_roll_pitch", "X": 16.0, "Y": 0, "Z": 1.2, "roll": 0.07,  "pitch": -0.1, "L": 8.5, "W": 4, "H": 0.2, "color": "0.6 0.4 0.4 1"},
        {"name": "ramp_down",       "X": 24.0, "Y": 0, "Z": 1.2, "roll": -0.07, "pitch": 0.1,  "L": 8.5, "W": 4, "H": 0.2, "color": "0.4 0.6 0.4 1"}
    ]
    
    for r in ramps:
        world += create_box_model(r["name"], r["X"], r["Y"], r["Z"], r["roll"], r["pitch"], 0, r["L"], r["W"], r["H"], r["color"])

    # --- BUMPY.WORLD KALIBINDA, RAMPAYA GÖMÜLÜ KÜRE TAŞLAR ---
    num_stones_per_ramp = 120  # Mevcut extreme haritasının taş yoğunluğunu korudum.
    min_visible_height = 0.01  # bumpy.world kalıbında taşların en az 1 cm'si yüzeyin üstünde kalır.
    max_visible_height = 0.03  # bumpy.world kalıbında taşların en fazla 3 cm'si yüzeyin üstünde kalır.
    stone_idx = 0
    
    for r in ramps:
        for _ in range(num_stones_per_ramp):
            x_L = random.uniform(-r["L"]/2 + 0.2, r["L"]/2 - 0.2)
            y_L = random.uniform(-r["W"]/2 + 0.2, r["W"]/2 - 0.2)
            
            radius = random.uniform(0.05, 0.10)  # Taş yarıçapını bumpy.world ile aynı 5-10 cm aralığına getirdim.
            visible_height = random.uniform(min_visible_height, max_visible_height)  # Her taşın yüzeyde görünen yüksekliğini rastgele belirledim.
            z_L = r["H"]/2 + visible_height - radius  # Küreyi rampaya gömerek yalnızca 1-3 cm'lik kısmını görünür bıraktım.
            
            X_G, Y_G, Z_G = transform_to_global(x_L, y_L, z_L, r["X"], r["Y"], r["Z"], r["roll"], r["pitch"])
            
            world += create_sphere_model(f"stone_{stone_idx}", X_G, Y_G, Z_G, r["roll"], r["pitch"], 0, radius)  # bumpy.world taş kalıbını rampaya ekledim.
            stone_idx += 1

    world += '</world>\n</sdf>'
    
    # --- DOĞRUDAN PAKET İÇİNE KAYDETME ---
    output_dir = "/home/taylan/ur3_ws/src/ur3_end_effector_stabilization/worlds"
    os.makedirs(output_dir, exist_ok=True)
    file_path = os.path.join(output_dir, "extreme_disturbance.world")
    
    with open(file_path, "w") as f:
        f.write(world)
    print(f"Eğimi azaltılmış, yarım küresel taşlı harita başarıyla şuraya kaydedildi:\n{file_path}")

if __name__ == "__main__":
    generate_world()
