#!/usr/bin/env python3
import os
import math

def create_ramp_model(name, pose_x, pose_y, yaw, width, h, l_up, l_flat, l_down, rgb="1 1 0"):
    t = 0.1 # Kalınlık
    links_sdf = ""
    current_x = 0.0
    
    # 1. Tırmanış Rampası
    if l_up > 0:
        angle_up = math.asin(h / l_up)
        l_up_proj = l_up * math.cos(angle_up)
        x_up = current_x + l_up_proj / 2.0
        z_up = h / 2.0
        
        links_sdf += f"""
        <link name='up_link'>
          <pose>{x_up} 0 {z_up} 0 {-angle_up} 0</pose>
          <collision name='up_col'><geometry><box><size>{l_up} {width} {t}</size></box></geometry></collision>
          <visual name='up_vis'>
            <geometry><box><size>{l_up} {width} {t}</size></box></geometry>
            <material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse></material>
          </visual>
        </link>
        """
        current_x += l_up_proj
        
    # 2. Düz Tepe (Opsiyonel)
    if l_flat > 0:
        x_flat = current_x + l_flat / 2.0
        z_flat = h
        
        links_sdf += f"""
        <link name='flat_link'>
          <pose>{x_flat} 0 {z_flat} 0 0 0</pose>
          <collision name='flat_col'><geometry><box><size>{l_flat} {width} {t}</size></box></geometry></collision>
          <visual name='flat_vis'>
            <geometry><box><size>{l_flat} {width} {t}</size></box></geometry>
            <material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse></material>
          </visual>
        </link>
        """
        current_x += l_flat
        
    # 3. İniş Rampası
    if l_down > 0:
        angle_down = math.asin(h / l_down)
        l_down_proj = l_down * math.cos(angle_down)
        x_down = current_x + l_down_proj / 2.0
        z_down = h / 2.0
        
        links_sdf += f"""
        <link name='down_link'>
          <pose>{x_down} 0 {z_down} 0 {angle_down} 0</pose>
          <collision name='down_col'><geometry><box><size>{l_down} {width} {t}</size></box></geometry></collision>
          <visual name='down_vis'>
            <geometry><box><size>{l_down} {width} {t}</size></box></geometry>
            <material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse></material>
          </visual>
        </link>
        """
        
    model_sdf = f"""
    <model name='{name}'>
      <static>true</static>
      <pose>{pose_x} {pose_y} -0.05 0 0 {yaw}</pose>
      {links_sdf}
    </model>
    """
    return model_sdf

def generate_world():
    sdf = """<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="custom_parkour">
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
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
          <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material>
        </visual>
      </link>
    </model>
"""
    
    # 1A. Sadece Sağ Tekerlek Rampası (Robot Sola Yatar)
    # y = -0.4: Husky'nin sadece sağ tekeri hizasında. Genişlik = 0.4m. Yükseklik = 0.1m
    sdf += create_ramp_model("roll_right_bump", pose_x=3.0, pose_y=-0.4, yaw=0.0, 
                             width=0.4, h=0.1, l_up=1.5, l_flat=0.0, l_down=1.5, rgb="1.0 0.8 0.0")

    # 1B. Sadece Sol Tekerlek Rampası (Robot Sağa Yatar)
    # y = 0.4: Husky'nin sadece sol tekeri hizasında. Genişlik = 0.4m. Yükseklik = 0.1m
    sdf += create_ramp_model("roll_left_bump", pose_x=7.0, pose_y=0.4, yaw=0.0, 
                             width=0.4, h=0.1, l_up=1.5, l_flat=0.0, l_down=1.5, rgb="1.0 0.5 0.0")

    # 2. Klasik Pitch Rampası (Şahlanma)
    # y = 0.0: Tam genişlikte (3.0m). Çıkış (1.5m), Düzlük (1.0m), İniş (1.5m). Yükseklik = 0.12m
    sdf += create_ramp_model("pitch_ramp", pose_x=12.0, pose_y=0.0, yaw=0.0, 
                             width=3.0, h=0.12, l_up=1.5, l_flat=1.0, l_down=1.5, rgb="0.0 0.8 0.0")

    # 3. Roll + Pitch Kombine Rampa
    # 30 derece (0.52 radyan) açıyla yerleştirilmiş standart rampa.
    sdf += create_ramp_model("combined_ramp", pose_x=18.0, pose_y=0.0, yaw=0.5235, 
                             width=3.0, h=0.12, l_up=1.5, l_flat=1.0, l_down=1.5, rgb="0.8 0.0 0.0")

    sdf += """
  </world>
</sdf>
"""
    return sdf

if __name__ == '__main__':
    script_dir = os.path.dirname(os.path.abspath(__file__))
    worlds_dir = os.path.abspath(os.path.join(script_dir, '../../worlds'))
    if not os.path.exists(worlds_dir): os.makedirs(worlds_dir)
        
    world_path = os.path.join(worlds_dir, 'custom_parkour.world')
    with open(world_path, 'w') as f:
        f.write(generate_world())
        
    print(f"✅ Harika! İleri seviye geometrik parkur başarıyla oluşturuldu:\\n{world_path}")
