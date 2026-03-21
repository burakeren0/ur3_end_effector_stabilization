import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.neural_network import MLPRegressor
from sklearn.metrics import mean_squared_error, r2_score
import joblib
import os

print("--- YZ Modeli Eğitimi Başlıyor (V2 - Çıkış Ölçeklendirmeli) ---")

# 1. Veriyi Yükle
data_path = os.path.expanduser('~/ur3_ws/src/mobile_manipulator/analysis/supervised_training_data_processed.csv')
df = pd.read_csv(data_path)

X_cols = ['q1','q2','q3','q4','q5','q6', 'dq1','dq2','dq3','dq4','dq5','dq6', 'ax','ay','az', 'imu_wx','imu_wy','imu_wz']
Y_cols = ['target_dq1', 'target_dq2', 'target_dq3', 'target_dq4', 'target_dq5', 'target_dq6']

X = df[X_cols].values
Y = df[Y_cols].values

# 2. Veri Setini Böl
X_train, X_test, Y_train, Y_test = train_test_split(X, Y, test_size=0.2, random_state=42)

# 3. Girişleri (X) Ölçeklendir
scaler_X = StandardScaler()
X_train_scaled = scaler_X.fit_transform(X_train)
X_test_scaled = scaler_X.transform(X_test)

# 4. YENİ: Çıkışları (Y) da Ölçeklendir! (Ağın çok küçük sayıları öğrenmesini kolaylaştırır)
scaler_Y = StandardScaler()
Y_train_scaled = scaler_Y.fit_transform(Y_train)
Y_test_scaled = scaler_Y.transform(Y_test)

# 5. Modeli Biraz Daha Güçlendir (128 Nöron ve Daha Fazla İterasyon)
print("\nSinir Ağı Eğitiliyor... (Lütfen bekleyin)")
model = MLPRegressor(hidden_layer_sizes=(128, 128), 
                     activation='relu', 
                     solver='adam', 
                     max_iter=2000, 
                     learning_rate_init=0.001,
                     random_state=42)

model.fit(X_train_scaled, Y_train_scaled)

# 6. Test ve Değerlendirme (Ölçekleri geri çevirerek gerçek rad/s hatasını bul)
Y_pred_scaled = model.predict(X_test_scaled)
Y_pred = scaler_Y.inverse_transform(Y_pred_scaled) # Gerçek birimlere geri dön

rmse = np.sqrt(mean_squared_error(Y_test, Y_pred))
r2 = r2_score(Y_test, Y_pred)

print("\n--- EĞİTİM SONUÇLARI ---")
print(f"Test Seti Genel RMSE (Hata): {rmse:.5f} rad/s")
print(f"Modelin Başarı Skoru (R^2):  {r2:.4f}")

# 7. Kaydet (Artık Y ölçeklendiricisini de kaydediyoruz)
model_save_path = os.path.expanduser('~/ur3_ws/src/mobile_manipulator/analysis/ur3_stabilization_model.pkl')
scaler_x_save_path = os.path.expanduser('~/ur3_ws/src/mobile_manipulator/analysis/ur3_scaler_X.pkl')
scaler_y_save_path = os.path.expanduser('~/ur3_ws/src/mobile_manipulator/analysis/ur3_scaler_Y.pkl')

joblib.dump(model, model_save_path)
joblib.dump(scaler_X, scaler_x_save_path)
joblib.dump(scaler_Y, scaler_y_save_path)

print(f"\nHarika! Model ve her iki Ölçeklendirici başarıyla kaydedildi.")
