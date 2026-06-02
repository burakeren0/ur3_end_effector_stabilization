% plot_target_angles.m
% analysis klasorundeki read_target_and_actual_angles.py CSV dosyalarini cizer.

clear; % Onceki workspace degiskenlerinin bu analiz sonucunu etkilemesini engeller.
close all; % Eski figurleri kapatarak sadece yeni grafiklerin gorunmesini saglar.
clc; % Komut penceresini bu calistirma icin temizler.

script_file = mfilename('fullpath'); % Scriptin bulundugu klasoru calisma dizininden bagimsiz bulur.
if isempty(script_file) % Script secili satir olarak calistirilirse mfilename bos donebilir.
    script_dir = pwd; % Bu durumda kullanicinin aktif klasorunu script klasoru gibi kullanir.
else % Normal dosya olarak calistirma durumunu ele alir.
    script_dir = fileparts(script_file); % analysis/matlab_files klasorunu elde eder.
end % Script klasoru secimini bitirir.

repo_analysis_dir = '/home/taylan/ur3_ws/src/ur3_end_effector_stabilization/analysis'; % MATLAB Drive kopyasi calissa bile gercek repo CSV klasorunu once dener.
analysis_candidates = {repo_analysis_dir, ... % Aday 1: bu makinedeki asil ROS workspace analysis klasoru.
    fullfile(script_dir, 'analysis'), ... % Aday 2: script MATLAB Drive/UR3 gibi repo kokune kopyalandiysa analysis alt klasorunu dener.
    fullfile(pwd, 'analysis'), ... % Aday 3: MATLAB aktif klasoru repo kokuyse analysis alt klasorunu dener.
    fullfile(script_dir, '..'), ... % Aday 4: script analysis/matlab_files icindeyse bir ust klasor analysis olur.
    pwd}; % Aday 5: MATLAB aktif klasoru dogrudan CSV klasoru ise onu dener.
analysis_dir = find_csv_analysis_dir(analysis_candidates); % CSV iceren ilk uygun analysis klasorunu secer.
if isempty(analysis_dir) % Hicbir aday klasorde CSV bulunamazsa ayrintili hata verir.
    searched_dirs_text = sprintf('  %s\n', analysis_candidates{:}); % Hata mesajinda denenen klasorleri listeler.
    error('analysis klasorunde CSV dosyasi bulunamadi. Aranan klasorler:\n%s', searched_dirs_text); % Yol hatasini kullaniciya net gosterir.
end % analysis klasoru secimini bitirir.
output_dir = fullfile(analysis_dir, 'matlab_files', 'target_angle_figures'); % PNG grafiklerini secilen analysis klasoru altina kaydeder.
if ~exist(output_dir, 'dir') % Grafik klasoru yoksa olusturulmasi gerektigini kontrol eder.
    mkdir(output_dir); % Kayit icin gerekli hedef klasoru olusturur.
end % Grafik klasoru hazirligini tamamlar.

plot_in_degrees = false; % false: radyan cizer, true: dereceye cevirerek cizer.
angle_mode = 'target_relative'; % Actual aciyi hedef aciya en yakin esdeger aci olarak gosterir.
show_figures = true; % true: MATLAB figur pencerelerini acar.
save_png_files = true; % true: Her CSV icin PNG dosyasi kaydeder.

joint_keys = {'shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'}; % CSV eklem adlarini sirayla tanimlar.
joint_labels = {'shoulder pan (q1)','shoulder lift (q2)','elbow (q3)','wrist 1 (q4)','wrist 2 (q5)','wrist 3 (q6)'}; % Grafik etiketlerini okunabilir yapar.
target_cols = strcat('target_', joint_keys); % Hedef aci sutun adlarini olusturur.
actual_cols = strcat('actual_', joint_keys); % Actual aci sutun adlarini olusturur.

csv_files = dir(fullfile(analysis_dir, '*.csv')); % analysis klasorundeki tum CSV dosyalarini listeler.
if isempty(csv_files) % CSV dosyasi bulunamazsa kullaniciyi net hata ile bilgilendirir.
    error('analysis klasorunde CSV dosyasi bulunamadi: %s', analysis_dir); % Eksik veri durumunda scripti durdurur.
end % CSV varlik kontrolunu bitirir.

valid_file_count = 0; % Beklenen target/actual sutunlarina sahip dosya sayisini tutar.
for file_index = 1:numel(csv_files) % Her CSV dosyasini tek tek isler.
    csv_path = fullfile(csv_files(file_index).folder, csv_files(file_index).name); % Okunacak CSV dosyasinin tam yolunu kurar.
    T = readtable(csv_path); % CSV verisini MATLAB tablosu olarak okur.

    if ~has_joint_columns(T, target_cols, actual_cols) % Dosyanin read_target_and_actual_angles.py formatinda olup olmadigini kontrol eder.
        fprintf('Atlandi: %s beklenen target/actual sutunlarina sahip degil.\n', csv_files(file_index).name); % Uyumsuz dosyayi raporlar.
        continue; % Uyumsuz CSV icin grafik uretmeden sonraki dosyaya gecer.
    end % Sutun uyumlulugu kontrolunu bitirir.

    valid_file_count = valid_file_count + 1; % Gecerli dosya sayacini artirir.
    [~, csv_name, ~] = fileparts(csv_files(file_index).name); % Baslik ve kayit adi icin uzantisiz dosya adini alir.
    display_name = strrep(csv_name, '_', ' '); % Grafik basliginda alt cizgileri boslukla degistirir.
    time_s = T.time - T.time(1); % Zaman eksenini ilk ornekten baslayacak sekilde sifirlar.

    fig_visibility = 'on'; % Varsayilan olarak figur penceresini gorunur acar.
    if ~show_figures % Kullanici figurleri gostermek istemezse gorunurlugu kapatir.
        fig_visibility = 'off'; % Figurleri arka planda uretir.
    end % Figur gorunurlugu secimini bitirir.

    fig = figure('Name', ['UR3 target vs actual - ', csv_name], 'NumberTitle', 'off', 'Visible', fig_visibility); % Her CSV icin ayri figur acar.
    ax = zeros(6, 1); % Alt grafik eksenlerini linkaxes icin saklar.

    for joint_index = 1:6 % Alti UR3 eklemini sirayla cizer.
        ax(joint_index) = subplot(6, 1, joint_index); % Ilgili eklem icin alt grafik alani acar.
        hold(ax(joint_index), 'on'); % Target ve actual cizgilerini ayni eksende tutar.

        target_data = T.(target_cols{joint_index}); % Ilgili eklemin hedef aci verisini alir.
        actual_data = T.(actual_cols{joint_index}); % Ilgili eklemin actual aci verisini alir.

        if strcmp(angle_mode, 'target_relative') % Multi-turn acilari hedefe gore en yakin esdegere indirger.
            actual_data = target_data + atan2(sin(actual_data - target_data), cos(actual_data - target_data)); % Actual aciyi hedefin yakininda sarar.
        elseif strcmp(angle_mode, 'wrapped') % Hem hedef hem actual acilari [-pi, pi] araligina sarar.
            target_data = atan2(sin(target_data), cos(target_data)); % Hedef aciyi standart araliga sarar.
            actual_data = atan2(sin(actual_data), cos(actual_data)); % Actual aciyi standart araliga sarar.
        elseif ~strcmp(angle_mode, 'raw') % Sadece tanimli aci modlarina izin verir.
            error('Gecersiz angle_mode: %s', angle_mode); % Hatali ayari erken yakalar.
        end % Aci modu uygulamasini bitirir.

        unit_label = 'rad'; % Varsayilan aci birimini radyan olarak tanimlar.
        if plot_in_degrees % Kullanici derece cizimi isterse donusum yapar.
            target_data = target_data * 180.0 / pi; % Hedef acilari dereceye cevirir.
            actual_data = actual_data * 180.0 / pi; % Actual acilari dereceye cevirir.
            unit_label = 'deg'; % Y ekseni etiketini derece olarak gunceller.
        end % Birim donusumunu bitirir.

        plot(ax(joint_index), time_s, target_data, 'LineWidth', 1.4, 'Color', [0.0 0.45 0.74]); % Hedef aci egrisini mavi cizer.
        plot(ax(joint_index), time_s, actual_data, '--', 'LineWidth', 1.4, 'Color', [0.85 0.33 0.10]); % Actual aci egrisini turuncu kesikli cizer.
        grid(ax(joint_index), 'on'); % Eksen okumayi kolaylastirmak icin izgara acar.
        ylabel(ax(joint_index), sprintf('%s\n(%s)', joint_labels{joint_index}, unit_label), 'Interpreter', 'none'); % Eklemi ve birimi y ekseninde gosterir.

        if joint_index == 1 % Basligi yalnizca ilk alt grafikte verir.
            title(ax(joint_index), ['UR3 Target vs Actual Joint Angles - ', display_name], 'Interpreter', 'none'); % Dosya adini figur basligina ekler.
            legend(ax(joint_index), {'Target', 'Actual'}, 'Location', 'best'); % Cizgi anlamlarini ilk grafikte belirtir.
        end % Ilk ekleme ozel baslik ve legend ayarini bitirir.

        if joint_index == 6 % X ekseni etiketini yalnizca en alttaki grafikte verir.
            xlabel(ax(joint_index), 'Zaman (s)'); % Ortak zaman ekseni birimini yazar.
        end % X ekseni etiketi ayarini bitirir.
    end % Alti eklem grafigini tamamlar.

    linkaxes(ax, 'x'); % Alt grafiklerin zaman eksenlerini birlikte hareket ettirir.

    if save_png_files % PNG kaydi aktifse figuru diske yazar.
        png_path = fullfile(output_dir, [safe_file_name(csv_name), '_target_actual_angles.png']); % Cikti dosyasi adini guvenli bicimde kurar.
        saveas(fig, png_path); % Figuru PNG olarak kaydeder.
        fprintf('Kaydedildi: %s\n', png_path); % Kaydedilen dosyanin yolunu raporlar.
    end % PNG kayit adimini bitirir.
end % Tum CSV dosyalarini islemeyi bitirir.

if valid_file_count == 0 % Hic uyumlu CSV bulunmadiysa bunu hata olarak ele alir.
    error('analysis klasorunde target_*/actual_* sutunlari olan CSV bulunamadi: %s', analysis_dir); % Beklenen veri yoksa scripti durdurur.
end % Gecerli dosya sayisi kontrolunu bitirir.

fprintf('Toplam %d CSV dosyasi icin hedef ve actual aci grafikleri olusturuldu.\n', valid_file_count); % Basarili islenen dosya sayisini raporlar.

function ok = has_joint_columns(T, target_cols, actual_cols) % CSV tablosunun beklenen eklem sutunlarini tasiyip tasimadigini dondurur.
    required_cols = [{'time'}, target_cols, actual_cols]; % Zorunlu time, target ve actual sutunlarini birlestirir.
    ok = all(ismember(required_cols, T.Properties.VariableNames)); % Tum zorunlu sutunlar varsa true dondurur.
end % Sutun kontrolu yardimci fonksiyonunu bitirir.

function safe_name = safe_file_name(name_text) % Dosya adini PNG kaydi icin guvenli hale getirir.
    safe_name = regexprep(name_text, '[^A-Za-z0-9_-]', '_'); % Harf, rakam, tire ve alt cizgi disindaki karakterleri temizler.
    safe_name = regexprep(safe_name, '_+', '_'); % Pes pese gelen alt cizgileri tek alt cizgiye indirir.
    safe_name = regexprep(safe_name, '^_|_$', ''); % Basta veya sonda kalan alt cizgiyi kaldirir.
end % Guvenli dosya adi yardimci fonksiyonunu bitirir.

function selected_dir = find_csv_analysis_dir(candidate_dirs) % CSV iceren ilk mevcut aday klasoru bulur.
    selected_dir = ''; % Baslangicta uygun klasor bulunmadigini isaretler.
    for candidate_index = 1:numel(candidate_dirs) % Tum aday klasorleri sirayla gezer.
        candidate_dir = candidate_dirs{candidate_index}; % Siradaki aday klasoru alir.
        if exist(candidate_dir, 'dir') ~= 7 % Aday klasor yoksa bu secenegi atlar.
            continue; % Mevcut olmayan klasor icin CSV aramasi yapmaz.
        end % Klasor varlik kontrolunu bitirir.
        candidate_csv_files = dir(fullfile(candidate_dir, '*.csv')); % Aday klasordeki CSV dosyalarini listeler.
        if ~isempty(candidate_csv_files) % En az bir CSV varsa bu klasoru kullanir.
            selected_dir = candidate_dir; % Uygun CSV klasorunu sonuc olarak kaydeder.
            return; % Ilk uygun klasor bulundugu icin aramayi bitirir.
        end % CSV varlik kontrolunu bitirir.
    end % Aday klasor taramasini bitirir.
end % CSV analysis klasoru bulma yardimci fonksiyonunu bitirir.
