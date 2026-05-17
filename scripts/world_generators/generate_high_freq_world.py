import random

def generate_high_freq_world():
    # Harita Ayarları
    world_name = "high_freq_bumpy.world"
    num_bumps = 800       # Yüksek frekans için çok sayıda taş
    path_length = 10.0    # X ekseninde 7 metrelik bir yol
    path_width = 2.0      # Y ekseninde yolun genişliği (+1 ile -1 arası)
    
    # Tümsek (Bozucu) Ayarları
    # Husky'nin rahat geçmesi için tümseklerin yeryüzünde kalan yüksekliği 1 cm ile 3 cm arası olacak.
    min_visible_height = 0.01 
    max_visible_height = 0.03 

    # Gazebo World Şablonu (Başlangıç)
    world_xml = """<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="high_freq_world">
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
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>100.0</mu>
                <mu2>100.0</mu2>
              </ode>
            </friction>
          </surface>
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
          </material>
        </visual>
      </link>
    </model>
"""

    # Rastgele Tümsekleri (Küreleri) Üret
    # Robot X=0'dan başlayacağı için taşları X=1.5'tan itibaren diziyoruz.
    for i in range(num_bumps):
        x = random.uniform(1.5, path_length)
        y = random.uniform(-path_width/2, path_width/2)
        
        # Kürenin yarıçapı (örneğin 5 cm ile 10 cm arası)
        radius = random.uniform(0.05, 0.10)
        
        # Sadece minik bir kısmının dışarıda kalması için Z ekseninde aşağı çekiyoruz
        visible_height = random.uniform(min_visible_height, max_visible_height)
        z = visible_height - radius # Kürenin merkezinin Z koordinatı
        
        world_xml += f"""
    <model name="bump_{i}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <sphere><radius>{radius:.3f}</radius></sphere>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>100.0</mu>
                <mu2>100.0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <sphere><radius>{radius:.3f}</radius></sphere>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient> <diffuse>0.4 0.4 0.4 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

    # World Şablonu Kapanış
    world_xml += """
  </world>
</sdf>
"""

    # Dosyaya Yazma
    with open(world_name, "w") as f:
        f.write(world_xml)
    
    print(f"✅ Harita başarıyla oluşturuldu: {world_name}")
    print(f"Toplam {num_bumps} adet yüksek frekanslı, düşük genlikli bozucu eklendi.")

if __name__ == "__main__":
    generate_high_freq_world()