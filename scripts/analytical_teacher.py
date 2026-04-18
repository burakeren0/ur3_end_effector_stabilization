#!/usr/bin/env python3
import pandas as pd
import numpy as np
from scipy.signal import butter, filtfilt
import os

print("--- Faz 2: Analitik Öğretmen (Veri İşleme) Başlıyor ---")

# 1. Ham Veriyi Yükle
input_path = os.path.expanduser('~/ur3_ws/src/mobile_manipulator/analysis/raw_training_data.csv')
df = pd.read_csv(input_path)
print(f"Okunan Ham Veri: {len(df)} satır. İşleniyor...")

# 2. IMU Gürültüsünü Temizle (Low-Pass Filter)
# Gazebo'nun fizik motorundan gelen mikro-titreşimleri temizleyip pürüzsüz sarsıntıları bırakıyoruz.
b, a = butter(4, 0.04, btype='low', analog=False)
df['wx_filt'] = filtfilt(b, a, df['wx'])
df['wy_filt'] = filtfilt(b, a, df['wy'])
df['wz_filt'] = filtfilt(b, a, df['wz'])

# 3. UR3 Diferansiyel Kinematik (Standart DH Parametreleri)
a_dh = [0, -0.24365, -0.21325, 0, 0, 0]
d_dh = [0.1519, 0, 0, 0.11235, 0.08535, 0.0819]
alpha_dh = [np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0]

def dh_matrix(theta, a, d, alpha):
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0,             np.sin(alpha),               np.cos(alpha),              d],
        [0,             0,                           0,                          1]
    ])

def get_angular_jacobian(q):
    Z = np.zeros((3, 6))
    Z[:, 0] = [0, 0, 1]
    T = np.eye(4)
    for i in range(5):
        T = T @ dh_matrix(q[i], a_dh[i], d_dh[i], alpha_dh[i])
        Z[:, i+1] = T[0:3, 2] # Dönüşüm matrisinin Z eksenini çek
    return Z

# 4. Matematiksel Çözüm (Cevap Anahtarı Oluşturma)
target_dq_list = []

for index, row in df.iterrows():
    q = [row['q1'], row['q2'], row['q3'], row['q4'], row['q5'], row['q6']]
    
    # Hedef Hız: Sarsıntının (filtrelenmiş) tam tersi yönü
    w_target = -np.array([row['wx_filt'], row['wy_filt'], row['wz_filt']])
    
    # Jacobian Sözde-Tersi ile motor hızlarını bul
    J_w = get_angular_jacobian(q)
    J_w_pinv = np.linalg.pinv(J_w)
    dq_ideal = J_w_pinv @ w_target
    
    target_dq_list.append(dq_ideal)

# 5. Yeni Sütunları Ekle ve Sadeleştir
dq_df = pd.DataFrame(target_dq_list, columns=['target_dq1', 'target_dq2', 'target_dq3', 'target_dq4', 'target_dq5', 'target_dq6'])
df_final = pd.concat([df, dq_df], axis=1)

# AI'ın kafasını karıştıracak gereksiz verileri (Quaternion ve ham gürültü) çöpe atıyoruz!
# Sadece Girdiler (X: Açılar ve Filtrelenmiş Hızlar) ve Çıktılar (Y: Motor Hızları) kalıyor.
df_final = df_final[['q1', 'q2', 'q3', 'q4', 'q5', 'q6', 
                     'wx_filt', 'wy_filt', 'wz_filt',
                     'target_dq1', 'target_dq2', 'target_dq3', 'target_dq4', 'target_dq5', 'target_dq6']]

# 6. Final Veri Setini Kaydet
output_path = os.path.expanduser('~/ur3_ws/src/mobile_manipulator/analysis/supervised_training_data_v3.csv')
df_final.to_csv(output_path, index=False)

print(f"BÜYÜK BAŞARI! {len(df_final)} satırlık matris denklemi çözüldü.")
print(f"Yapay Zeka Öğretmen Verisi Kaydedildi:\n{output_path}")
