import pandas as pd
import numpy as np
from scipy.signal import butter, filtfilt
import os
import math

print("Faz 2: Veri Ön İşleme ve Hedef (Target) Hızların Hesaplanması Başlıyor...")

# 1. Alçak Geçiren Filtre (Low-Pass Filter) Fonksiyonu
def lowpass_filter(data, cutoff_freq=5.0, sample_rate=100.0, order=2):
    nyq = 0.5 * sample_rate
    normal_cutoff = cutoff_freq / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    # filtfilt: İleri-geri filtreleme yaparak sinyalde faz kayması (gecikme) olmasını engeller
    return filtfilt(b, a, data)

# 2. UR3 Geometrik Jacobian Hesaplama (MATLAB'deki kodunun Python versiyonu)
def get_ur3_jacobian(q):
    # UR3 DH Parametreleri [a, d, alpha]
    a = [0, -0.24365, -0.21325, 0, 0, 0]
    d = [0.1519, 0, 0, 0.11235, 0.08535, 0.0819]
    alpha = [math.pi/2, 0, 0, math.pi/2, -math.pi/2, 0]

    T = np.eye(4)
    z_axes = [np.array([0, 0, 1])]
    origins = [np.array([0, 0, 0])]

    for i in range(6):
        th = q[i]
        A = np.array([
            [math.cos(th), -math.sin(th)*math.cos(alpha[i]),  math.sin(th)*math.sin(alpha[i]), a[i]*math.cos(th)],
            [math.sin(th),  math.cos(th)*math.cos(alpha[i]), -math.cos(th)*math.sin(alpha[i]), a[i]*math.sin(th)],
            [0,             math.sin(alpha[i]),               math.cos(alpha[i]),              d[i]],
            [0,             0,                                0,                               1]
        ])
        T = T @ A
        z_axes.append(T[0:3, 2])
        origins.append(T[0:3, 3])

    o_n = origins[-1]
    J = np.zeros((6, 6))
    
    # J_v = z_prev X (o_n - o_prev)  |  J_w = z_prev
    for i in range(6):
        J[0:3, i] = np.cross(z_axes[i], (o_n - origins[i]))
        J[3:6, i] = z_axes[i]

    return J

# 3. Veri Okuma
input_file = os.path.expanduser('~/ur3_ws/src/mobile_manipulator/analysis/supervised_training_data_raw.csv')
output_file = os.path.expanduser('~/ur3_ws/src/mobile_manipulator/analysis/supervised_training_data_processed.csv')

df = pd.read_csv(input_file)
print(f"Okunan ham veri boyutu: {df.shape[0]} satır.")

# 4. Gürültü Filtreleme (IMU ve Taban Hızlarını Filtrele)
columns_to_filter = ['ax', 'ay', 'az', 'imu_wx', 'imu_wy', 'imu_wz', 'v_bx', 'v_by', 'v_bz', 'w_bx', 'w_by', 'w_bz']
for col in columns_to_filter:
    df[col] = lowpass_filter(df[col])

print("Sensör verileri Low-Pass Filter ile gürültülerden arındırıldı.")

# 5. İdeal Hız Komutlarının (Target) Hesaplanması
target_q_dots = []

for index, row in df.iterrows():
    # O anki eklem açılarını al
    q_current = [row['q1'], row['q2'], row['q3'], row['q4'], row['q5'], row['q6']]
    
    # O anki taban sarsıntı hızını (V_base) al [v_x, v_y, v_z, w_x, w_y, w_z]
    v_base = np.array([row['v_bx'], row['v_by'], row['v_bz'], row['w_bx'], row['w_by'], row['w_bz']])
    
    # O anki Jacobian'ı hesapla
    J_num = get_ur3_jacobian(q_current)
    
    # Ters Kinematik Hız Denklemi: q_dot = J_pseudo_inverse * (-V_base)
    # (Uç noktayı sabit tutmak için taban sarsıntısının tam zıttı bir hız uygulamalıyız)
    J_pinv = np.linalg.pinv(J_num) # Sözde-Ters (Pseudo-Inverse)
    q_dot_ideal = J_pinv @ (-v_base)
    
    target_q_dots.append(q_dot_ideal)

# 6. Hedefleri (Cevap Anahtarını) Tabloya Ekle
target_columns = ['target_dq1', 'target_dq2', 'target_dq3', 'target_dq4', 'target_dq5', 'target_dq6']
df_targets = pd.DataFrame(target_q_dots, columns=target_columns)
df_final = pd.concat([df, df_targets], axis=1)

# 7. İşlenmiş Veriyi Kaydet
df_final.to_csv(output_file, index=False)
print(f"Mükemmel! Öğretmen cevap anahtarı oluşturuldu ve kaydedildi:\n{output_file}")
