clear; clc;

%% ______1. İleri Kinematik Analizi______
% 1. Eklem açılarını sembolik değişken olarak tanımlayalım (Zamana bağlı değişkenler)
syms th1 th2 th3 th4 th5 th6 real

% 2. UR3 DH Parametreleri Tablosu [a, d, alpha, theta]
dh_params = [
    0,         0.1519,   pi/2,  th1;  % Joint 1
    -0.24365,  0,        0,     th2;  % Joint 2
    -0.21325,  0,        0,     th3;  % Joint 3
    0,         0.11235,  pi/2,  th4;  % Joint 4
    0,         0.08535, -pi/2,  th5;  % Joint 5
    0,         0.0819,   0,     th6   % Joint 6
];

% 3. Dönüşüm matrislerini saklayacağımız hücre dizisi (Cell Array)
T_list = cell(1, 7);
T_list{1} = eye(4); % T_0^0: Taban (Base) referans noktası

fprintf('UR3 İleri Kinematik (Forward Kinematics) Hesaplanıyor...\n');

% 4. Her bir eklem için A_i matrisini hesapla ve ardışık olarak çarp
for i = 1:6
    a = dh_params(i, 1);
    d = dh_params(i, 2);
    alp = dh_params(i, 3);
    th = dh_params(i, 4);
    
    A_i = [cos(th), -sin(th)*cos(alp),  sin(th)*sin(alp), a*cos(th);
           sin(th),  cos(th)*cos(alp), -cos(th)*sin(alp), a*sin(th);
           0,        sin(alp),          cos(alp),         d;
           0,        0,                 0,                1];
       
    T_list{i+1} = simplify(T_list{i} * A_i);
end

% 5. Sonuç: Uç İşlevcinin Toplam Dönüşüm Matrisi (T_6^0)
T_end_effector = T_list{7};
Position = T_end_effector(1:3, 4);

disp('Uç İşlevcinin Sembolik Konumu (x, y, z):');
disp(Position);

%% ______2. Jacobian Matrisi______
fprintf('\nUR3 Geometrik Jakobi Matrisi (Jacobian) Çıkarılıyor...\n');

o_6 = T_list{7}(1:3, 4); 
J = sym(zeros(6, 6)); 

for i = 1:6
    T_prev = T_list{i}; 
    z_prev = T_prev(1:3, 3);
    o_prev = T_prev(1:3, 4);
    
    J_v = cross(z_prev, (o_6 - o_prev));
    J_w = z_prev;
    
    J(:, i) = [J_v; J_w];
end

disp('6x6 Sembolik Jacobian Matrisi Başarıyla Oluşturuldu!');

J_func = matlabFunction(J, 'Vars', {th1, th2, th3, th4, th5, th6});
disp('Jacobian Sayısal Fonksiyonu (J_func) kullanıma hazır.');

%% ______3. Veri Okuma ve Numerik Analiz______
fprintf('\nROS 2 Verileri (CSV) Okunuyor ve Analiz Ediliyor...\n');

% Python'dan kaydettiğimiz csv dosyasını oku
data = readmatrix('robot_kinematics_data.csv');
time = data(:, 1);      % 1. Sütun: Zaman
Q    = data(:, 2:7);    % 2-7. Sütunlar: Eklem Pozisyonları (q)
dQ   = data(:, 8:13);   % 8-13. Sütunlar: Eklem Hızları (dq)
V_meas = data(:, 14:19);% 14-19. Sütunlar: TF'ten ölçülen Uç Nokta Hızları

V_calc = zeros(size(V_meas)); % Teorik hızları kaydedeceğimiz boş matris

% Her bir zaman adımı (milisaniye) için teorik hızı hesapla
for k = 1:size(data, 1)
    % 1. O anki eklem açılarını J_func'a gönder ve anlık Jacobian matrisini bul
    J_num = J_func(Q(k,1), Q(k,2), Q(k,3), Q(k,4), Q(k,5), Q(k,6));
    
    % 2. İleri Hız Kinematiği Formülü: V = J * dq
    % (J_num 6x6'dır, dQ vektörü ile çarpıyoruz)
    V_calc(k, :) = (J_num * dQ(k, :)')';
end

%% ______4. Görselleştirme (Grafikler)______
fprintf('Grafikler Çizdiriliyor...\n');

figure('Name', 'UR3 Hız Kinematiği Doğrulama', 'Units', 'normalized', 'Position', [0.1, 0.05, 0.4, 0.85]);

titles = {'V_x (Lineer Hız)', 'V_y (Lineer Hız)', 'V_z (Lineer Hız)', ...
          '\omega_x (Açısal Hız)', '\omega_y (Açısal Hız)', '\omega_z (Açısal Hız)'};
units = {'m/s', 'm/s', 'm/s', 'rad/s', 'rad/s', 'rad/s'};

for i = 1:6
    subplot(6, 1, i); 
    % Gerçek Ölçüm (Kırmızı Kesik Çizgi)
    plot(time, V_meas(:, i), 'r--', 'LineWidth', 1.5); hold on;
    % Teorik Hesaplama (Mavi Düz Çizgi)
    plot(time, V_calc(:, i), 'b', 'LineWidth', 1);
    
    ylabel(units{i});
    title(titles{i}, 'FontSize', 10);
    grid on;
    
    if i == 1
        legend('Ölçülen (TF) [Gerçek]', 'Hesaplanan (J·dq) [Teori]', 'Location', 'northeast');
    end
    if i < 6
        set(gca, 'XTickLabel', []); % Sadelik için üstteki 5 grafiğin X ekseni yazılarını sil
    else
        xlabel('Zaman (s)');
    end
end

%% ______5. Hata Hesaplama (RMSE)______
% Teorik hesap ile ölçüm arasındaki farkı (Hatayı) ölçüyoruz
total_rmse = sqrt(mean((V_meas - V_calc).^2, 'all', 'omitnan'));

fprintf('\n=== ANALİZ SONUCU ===\n');
fprintf('Genel Kök Ortalama Kare Hata (RMSE): %.6f\n', total_rmse);

if total_rmse < 0.05
    fprintf('BAŞARILI: Hata payı çok düşük! Kurduğunuz Jacobian Modeli kusursuz çalışıyor.\n');
else
    fprintf('DİKKAT: Hata payı yüksek. Eklem limitlerinde veya veri kaydında sorun olabilir.\n');
end