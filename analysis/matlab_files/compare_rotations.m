%% compare_rotations.m
% Husky ve UR3 uç noktası rotasyon açılarının karşılaştırılması

csv_file = 'matlab_tf_data.csv';

if ~isfile(csv_file)
    error('CSV dosyası bulunamadı: %s', csv_file);
end

T = readtable(csv_file);

time = T.time;

husky_roll = T.husky_roll_world_deg;
husky_pitch = T.husky_pitch_world_deg;
husky_yaw = T.husky_yaw_world_deg;

ee_roll = T.ee_roll_world_deg;
ee_pitch = T.ee_pitch_world_deg;
ee_yaw = T.ee_yaw_world_deg;

ee_x_base = T.ee_x_ur_base;
ee_y_base = T.ee_y_ur_base;
ee_z_base = T.ee_z_ur_base;

figure('Name', 'Husky ve Uç Nokta Yönelim Karşılaştırması', 'NumberTitle', 'off');

subplot(4,1,1);
plot(time, husky_roll, '-b', 'LineWidth', 1.5); hold on;
plot(time, ee_roll, '-r', 'LineWidth', 1.5);
xlabel('Zaman (s)');
ylabel('Roll (deg)');
legend('Husky roll', 'EE roll', 'Location', 'best');
grid on;

title('Roll Açısı Karşılaştırması');

subplot(4,1,2);
plot(time, husky_pitch, '-b', 'LineWidth', 1.5); hold on;
plot(time, ee_pitch, '-r', 'LineWidth', 1.5);
xlabel('Zaman (s)');
ylabel('Pitch (deg)');
legend('Husky pitch', 'EE pitch', 'Location', 'best');
grid on;

title('Pitch Açısı Karşılaştırması');

subplot(4,1,3);
plot(time, husky_yaw, '-b', 'LineWidth', 1.5); hold on;
plot(time, ee_yaw, '-r', 'LineWidth', 1.5);
xlabel('Zaman (s)');
ylabel('Yaw (deg)');
legend('Husky yaw', 'EE yaw', 'Location', 'best');
grid on;

title('Yaw Açısı Karşılaştırması');

subplot(4,1,4);
plot(time, ee_x_base, '-r', 'LineWidth', 1.5); hold on;
plot(time, ee_y_base, '-g', 'LineWidth', 1.5);
plot(time, ee_z_base, '-b', 'LineWidth', 1.5);
xlabel('Zaman (s)');
ylabel('EE pozisyonu (m)');
legend('EE X_{base}', 'EE Y_{base}', 'EE Z_{base}', 'Location', 'best');
grid on;

title('Uç Nokta Konumu (UR Base Frame)');

% Eğer tek bir grafikte görmek isterseniz bu kısmı açabilirsiniz:
% figure('Name', 'Husky vs EE Euler Açıları', 'NumberTitle', 'off');
% plot(time, husky_roll, '-b', time, ee_roll, '--b', time, husky_pitch, '-g', time, ee_pitch, '--g', time, husky_yaw, '-r', time, ee_yaw, '--r', 'LineWidth', 1.2);
% xlabel('Zaman (s)'); ylabel('Açı (deg)');
% legend('Husky roll', 'EE roll', 'Husky pitch', 'EE pitch', 'Husky yaw', 'EE yaw', 'Location', 'best');
% grid on;
