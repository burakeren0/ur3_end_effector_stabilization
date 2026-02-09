clear; clc;

%% 1. Sembolik Değişkenler ve DH Tanımlama
syms th1 th2 th3 th4 th5 th6 real

% UR3 DH Parametreleri: [a, d, alpha, theta]
dh_params = [
    0,          0.1519,   pi/2,   th1;
    -0.24365,   0,        0,      th2;
    -0.21325,   0,        0,      th3;
    0,          0.11235,  pi/2,   th4;
    0,          0.08535, -pi/2,   th5;
    0,          0.0819,   0,      th6
];

T_list = cell(1, 7); 
T_list{1} = eye(4); 

for i = 1:6
    a = dh_params(i, 1); d = dh_params(i, 2);
    alp = dh_params(i, 3); th = dh_params(i, 4);
    
    A_i = [cos(th), -sin(th)*cos(alp),  sin(th)*sin(alp), a*cos(th);
           sin(th),  cos(th)*cos(alp), -cos(th)*sin(alp), a*sin(th);
           0,        sin(alp),          cos(alp),         d;
           0,        0,                 0,                1];
       
    T_list{i+1} = simplify(T_list{i} * A_i);
end

o_6 = T_list{7}(1:3, 4); 
J = sym(zeros(6, 6)); 

for i = 1:6
    T_prev = T_list{i}; 
    z_prev = T_prev(1:3, 3);
    o_prev = T_prev(1:3, 4);
    
    J(:, i) = [cross(z_prev, (o_6 - o_prev)); z_prev];
end

% Hızlı hesaplama için fonksiyona çevir
J_func = matlabFunction(J, 'Vars', {th1, th2, th3, th4, th5, th6});

%% 2. Veri Okuma ve Numerik Analiz
data = readmatrix('robot_kinematics_data.csv');
time = data(:, 1);
Q    = data(:, 2:7);    
dQ   = data(:, 8:13);   
V_meas = data(:, 14:19); 

V_calc = zeros(size(V_meas));

fprintf('Analiz yapılıyor...\n');
for k = 1:size(data, 1)
    J_num = J_func(Q(k,1), Q(k,2), Q(k,3), Q(k,4), Q(k,5), Q(k,6));
    V_calc(k, :) = (J_num * dQ(k, :)')';
end

%% 3. Görselleştirme (Alt Alta 6 Grafik)
figure('Name', 'UR3 Hız Kinematiği Doğrulama (6-DOF)', 'Units', 'normalized', 'Position', [0.1, 0.05, 0.4, 0.85]);

titles = {'V_x (Lineer)', 'V_y (Lineer)', 'V_z (Lineer)', ...
          '\omega_x (Açısal)', '\omega_y (Açısal)', '\omega_z (Açısal)'};
units = {'m/s', 'm/s', 'm/s', 'rad/s', 'rad/s', 'rad/s'};

for i = 1:6
    subplot(6, 1, i); % 6 satır, 1 sütun
    plot(time, V_meas(:, i), 'r--', 'LineWidth', 1.3); hold on;
    plot(time, V_calc(:, i), 'b', 'LineWidth', 1);
    
    ylabel(units{i});
    title(titles{i}, 'FontSize', 10);
    grid on;
    
    if i == 1
        legend('Ölçülen (TF)', 'Hesaplanan (J·dq)', 'Location', 'northeast');
    end
    if i < 6
        set(gca, 'XTickLabel', []); % Son grafik hariç zaman etiketlerini gizle (sadelik için)
    else
        xlabel('Zaman (s)');
    end
end

%% 4. Hata Raporu
total_rmse = sqrt(mean((V_meas - V_calc).^2, 'all', 'omitnan'));
fprintf('\n--- ANALİZ SONUCU ---\n');
fprintf('Genel Ortalama RMSE: %.6f\n', total_rmse);