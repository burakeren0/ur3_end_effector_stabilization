#!/usr/bin/env python3
import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.neural_network import MLPRegressor
import joblib
import os

print("--- Faz 3: Yapay Zeka Beyninin Eğitimi (Patlama Çözüldü) ---")

# 1. İşlenmiş Veriyi Yükle
data_path = os.path.expanduser('~/ur3_ws/src/mobile_manipulator/analysis/supervised_training_data_v3.csv')
df = pd.read_csv(data_path)

# İŞTE HAYAT KURTARAN O MÜHENDİSLİK DOKUNUŞU:
# Eklem açılarındaki 10^-14'lük Gazebo gürültülerini yuvarlayıp, Scaler'ın patlamasını önlüyoruz.
df[['q1', 'q2', 'q3', 'q4', 'q5', 'q6']] = df[['q1', 'q2', 'q3', 'q4', 'q5', 'q6']].round(4)

X = df[['q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'wx_filt', 'wy_filt', 'wz_filt']].values
y = df[['target_dq1', 'target_dq2', 'target_dq3', 'target_dq4', 'target_dq5', 'target_dq6']].values

X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# 3. Verileri Ölçeklendirme
scaler = StandardScaler()
X_train_scaled = scaler.fit_transform(X_train)
X_test_scaled = scaler.transform(X_test)

# 4. Modeli Tanımla (Tolerans Düşürüldü, Sabır Artırıldı)
model = MLPRegressor(
    hidden_layer_sizes=(64, 64),
    activation='relu',
    solver='adam',
    alpha=0.0001,
    batch_size=256,
    learning_rate_init=0.001,
    max_iter=2000,
    tol=1e-8,                # HATA BURADAYDI! Varsayılan 0.0001'i iptal edip 0.00000001 yaptık.
    n_iter_no_change=50,     # Ağın pes etmeden önce beklemesi gereken iterasyon sayısını 10'dan 50'ye çıkardık.
    early_stopping=False,
    verbose=True,
    random_state=42
)

# 5. Eğitimi Başlat
print("Derin Öğrenme Başlıyor (Loss değerinin küçüldüğünü izleyin)...")
model.fit(X_train_scaled, y_train)

# 6. Başarı Ölçümü
train_score = model.score(X_train_scaled, y_train)
test_score = model.score(X_test_scaled, y_test)
print(f"\n--- Eğitim Tamamlandı ---")
print(f"Eğitim Verisi R^2 Skoru : {train_score:.4f} (1.0'a ne kadar yakınsa o kadar iyi)")
print(f"Test Verisi R^2 Skoru   : {test_score:.4f} (1.0'a ne kadar yakınsa o kadar iyi)")

# 7. Kayıt
model_path = os.path.expanduser('~/ur3_ws/src/mobile_manipulator/analysis/stabilizer_model_v3.pkl')
scaler_path = os.path.expanduser('~/ur3_ws/src/mobile_manipulator/analysis/feature_scaler_v3.pkl')

joblib.dump(model, model_path)
joblib.dump(scaler, scaler_path)

print(f"Kararlı Beyin Kaydedildi: {model_path}")
