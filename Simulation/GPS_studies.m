%% sensor variance estimation 1
%
% clear all
gps_pos_001 = h5read("sensor_records.hdf5", "/trajectory_0001/gps/position");
gt_pos1 = h5read("sensor_records.hdf5", "/trajectory_0001/groundtruth/position");
gps_HDOP_01 = h5read("sensor_records.hdf5", "/trajectory_0001/gps/HDOP");
gps_VDOP_01 = h5read("sensor_records.hdf5", "/trajectory_0001/gps/VDOP");
gps_GDOP_01 = h5read("sensor_records.hdf5", "/trajectory_0001/gps/GDOP");
gt_GPS_pos1=zeros(3,length(gps_pos_001(1,:)));
for i=1:length(gps_pos_001)
    gt_GPS_pos1(:,i)=gt_pos1(:,(i-1)*100+1);
end
errorpos_gps1= gps_pos_001- gt_GPS_pos1;

var_gps1 = var(errorpos_gps1')';

errorpos_gpsNorm1=zeros(3,length(gps_pos_001(1,:)));
errorpos_gpsNorm1(1,:)=abs(errorpos_gps1(1,:))./gps_HDOP_01;
errorpos_gpsNorm1(2,:)=abs(errorpos_gps1(2,:))./gps_HDOP_01;
errorpos_gpsNorm1(3,:)=abs(errorpos_gps1(3,:))./gps_VDOP_01;

var_coeff1 = var(errorpos_gpsNorm1')';
mean_coeff1 = mean(errorpos_gpsNorm1')';

%% sensor variance estimation 2
gps_pos_002 = h5read("sensor_records.hdf5", "/trajectory_0002/gps/position");
gt_pos2 = h5read("sensor_records.hdf5", "/trajectory_0002/groundtruth/position");
gt_pos2 = h5read("sensor_records.hdf5", "/trajectory_0002/groundtruth/position");
gps_HDOP_02 = h5read("sensor_records.hdf5", "/trajectory_0002/gps/HDOP");
gps_VDOP_02 = h5read("sensor_records.hdf5", "/trajectory_0002/gps/VDOP");
gps_GDOP_02 = h5read("sensor_records.hdf5", "/trajectory_0002/gps/GDOP");

gt_GPS_pos2=zeros(3,length(gps_pos_002(1,:)));
for i=1:length(gps_pos_002)
    gt_GPS_pos2(:,i)=gt_pos2(:,(i-1)*100+1);
end
errorpos_gps2= gps_pos_002- gt_GPS_pos2;

var_gps2 = var(errorpos_gps2')';

errorpos_gpsNorm2=zeros(3,length(gps_pos_002(1,:)));
errorpos_gpsNorm2(1,:)=abs(errorpos_gps2(1,:))./gps_HDOP_02;
errorpos_gpsNorm2(2,:)=abs(errorpos_gps2(2,:))./gps_HDOP_02;
errorpos_gpsNorm2(3,:)=abs(errorpos_gps2(3,:))./gps_VDOP_02;

var_coeff2 = var(errorpos_gpsNorm2')';
mean_coeff2 = mean(errorpos_gpsNorm2')';

%% sensor variance estimation 3
gps_pos_003 = h5read("sensor_records.hdf5", "/trajectory_0003/gps/position");
gt_pos3 = h5read("sensor_records.hdf5", "/trajectory_0003/groundtruth/position");
gps_HDOP_03 = h5read("sensor_records.hdf5", "/trajectory_0003/gps/HDOP");
gps_VDOP_03 = h5read("sensor_records.hdf5", "/trajectory_0003/gps/VDOP");
gps_GDOP_03 = h5read("sensor_records.hdf5", "/trajectory_0003/gps/GDOP");
gt_GPS_pos3=zeros(3,length(gps_pos_003(1,:)));
for i=1:length(gps_pos_003)
    gt_GPS_pos3(:,i)=gt_pos3(:,(i-1)*100+1);
end
errorpos_gps3= gps_pos_003- gt_GPS_pos3;

errorpos_gpsNorm3=zeros(3,length(gps_pos_003(1,:)));
errorpos_gpsNorm3(1,:)=abs(errorpos_gps3(1,:))./gps_HDOP_03;
errorpos_gpsNorm3(2,:)=abs(errorpos_gps3(2,:))./gps_HDOP_03;
errorpos_gpsNorm3(3,:)=abs(errorpos_gps3(3,:))./gps_VDOP_03;

var_gps3=zeros(6,1);
var_gps3(1,1) = var(errorpos_gps3(1,:));
var_gps3(2,1) = var(errorpos_gps3(2,:));
var_gps3(3,1) = var(errorpos_gps3(3,:));

var_coeff3 = var(errorpos_gpsNorm3')';
mean_coeff3 = mean(errorpos_gpsNorm3')';

%% confronto dati altre traiettorie
clc
clear all
close all

% Leggi i nomi dei dataset all'interno del file H5
info = h5info('sensor_records.hdf5');
dataset_names = {info.Groups.Name};

% Ciclo per sostituire la traiettoria con i dati dei dataset
for i = 1:numel(dataset_names)
    dataset_name = dataset_names{i};
    % Verifica se il nome del dataset è relativo alla traiettoria
    if startsWith(dataset_name, '/trajectory_')
        % Estrai il numero della traiettoria dal nome del dataset
        trajectory_number = sscanf(dataset_name, '/trajectory_%d/');
        % Costruisci il nome del dataset per la posizione GPS
        gps_dataset_name = sprintf('%s/gps/position', dataset_name);
        % Leggi i dati del dataset della posizione GPS
        gps_pos = h5read('sensor_records.hdf5', gps_dataset_name);
        % Sostituisci la traiettoria con i dati del dataset della posizione GPS
        eval(sprintf('gps_pos_%04d = gps_pos;', trajectory_number));
        
        % Leggi i dati del dataset della posizione del groundtruth
        gt_pos = h5read('sensor_records.hdf5', sprintf('%s/groundtruth/position', dataset_name));
        % Leggi i dati del dataset della precisione HDOP del GPS
        gps_HDOP = h5read('sensor_records.hdf5', sprintf('%s/gps/HDOP', dataset_name));
        % Leggi i dati del dataset della precisione VDOP del GPS
        gps_VDOP = h5read('sensor_records.hdf5', sprintf('%s/gps/VDOP', dataset_name));
        
        % Calcola la posizione del groundtruth corrispondente ad ogni punto GPS
        gt_GPS_pos = zeros(3, length(gps_pos(1,:)));
        for j = 1:length(gps_pos)
            gt_GPS_pos(:,j) = gt_pos(:, (j-1)*100 + 1);
        end
        
        % Calcola l'errore tra la posizione GPS e quella del groundtruth
        errorpos_gps = gps_pos - gt_GPS_pos;
        
        % Assegna i risultati alle variabili con nomi appropriati
        eval(sprintf('gt_pos_%04d = gt_pos;', trajectory_number));
        eval(sprintf('gps_HDOP_%04d = gps_HDOP;', trajectory_number));
        eval(sprintf('gps_VDOP_%04d = gps_VDOP;', trajectory_number));
        eval(sprintf('gt_GPS_pos_%04d = gt_GPS_pos;', trajectory_number));
        eval(sprintf('errorpos_gps_%04d = errorpos_gps;', trajectory_number));
    end
end

% Inizializzazione delle figure
figure(1);
hold on;
title('HDOP vs Errore GPS sull''asse X');
xlabel('HDOP');
ylabel('Errore GPS sull''asse X');
grid on;

figure(2);
hold on;
title('HDOP vs Errore GPS sull''asse Y');
xlabel('HDOP');
ylabel('Errore GPS sull''asse Y');
grid on;

figure(3);
hold on;
title('VDOP vs Errore GPS sull''asse Z');
xlabel('VDOP');
ylabel('Errore GPS sull''asse Z');
grid on;

% Ciclo per aggiungere gli scatter plot per ogni traiettoria
for i = 1:numel(dataset_names)
    dataset_name = dataset_names{i};
    % Verifica se il nome del dataset è relativo alla traiettoria
    if startsWith(dataset_name, '/trajectory_')
        % Estrai il numero della traiettoria dal nome del dataset
        trajectory_number = sscanf(dataset_name, '/trajectory_%d/');
        
        % Costruisci i nomi dei dataset per HDOP e errore GPS
        hdop_dataset_name = sprintf('%s/gps/HDOP', dataset_name);
        error_dataset_name = sprintf('errorpos_gps_%04d(1,:)', trajectory_number);
        
        % Leggi i dati dei dataset HDOP e errore GPS
        hdop_data = h5read('sensor_records.hdf5', hdop_dataset_name);
        error_data = eval(error_dataset_name);
        
        % Crea uno scatter plot per l'asse X
        figure(1);
        scatter(hdop_data, error_data, 3,rand(1,3), 'DisplayName', sprintf('tr%d', trajectory_number));
        
        % Crea uno scatter plot per l'asse Y
        error_dataset_name_y = sprintf('errorpos_gps_%04d(2,:)', trajectory_number);
        error_data_y = eval(error_dataset_name_y);
        figure(2);
        scatter(hdop_data, error_data_y,3,rand(1,3), 'DisplayName', sprintf('tr%d', trajectory_number));
        
        % Crea uno scatter plot per l'asse Z
        vdop_dataset_name = sprintf('%s/gps/VDOP', dataset_name);
        vdop_data = h5read('sensor_records.hdf5', vdop_dataset_name);
        error_dataset_name_z = sprintf('errorpos_gps_%04d(3,:)', trajectory_number);
        error_data_z = eval(error_dataset_name_z);
        figure(3);
        scatter(vdop_data, error_data_z, 3,rand(1,3), 'DisplayName', sprintf('tr%d', trajectory_number));
    end
end

% Aggiungi legenda alle figure
figure(1);
legend('Location', 'best');

figure(2);
legend('Location', 'best');

figure(3);
legend('Location', 'best');
%% calcolo varianze errori per range di valori di HDOP
clc

% Definizione dei range di HDOP e VDOP
%hdop_ranges = [0, 2, 2.3:0.3:4, 4.5, 5, 6, 7, 10, 30, 600];
%vdop_ranges = [0, 2, 2.3:0.3:4, 4.5, 5, 6, 7, 10, 30, 600];
hdop_ranges = [0:0.5:7, 10, 30, 600];
vdop_ranges = [0:0.5:7, 10, 30, 600];
% Inizializzazione delle strutture dati per le varianze
variance_x = zeros(size(hdop_ranges,2)-1, 1);
variance_y = zeros(size(hdop_ranges,2)-1, 1);
variance_z = zeros(size(vdop_ranges,2)-1, 1);

% Inizializzazione degli array per i valori dell'errore lungo x, y e z per ciascun range
error_x_cells = cell(size(hdop_ranges,2)-1, 1);
error_y_cells = cell(size(hdop_ranges,2)-1, 1);
error_z_cells = cell(size(vdop_ranges,2)-1, 1);

% Ciclo per riempire gli array con i valori dell'errore lungo x, y e z per ciascun range
for i = 1:numel(dataset_names)
    dataset_name = dataset_names{i};
    % Verifica se il nome del dataset è relativo alla traiettoria
    if startsWith(dataset_name, '/trajectory_')
        % Estrai il numero della traiettoria dal nome del dataset
        trajectory_number = sscanf(dataset_name, '/trajectory_%d/');
        
        % Costruisci il nome del dataset per HDOP, errore GPS lungo x, y e z
        hdop_dataset_name = sprintf('%s/gps/HDOP', dataset_name);
        vdop_dataset_name = sprintf('%s/gps/VDOP', dataset_name);
        error_x_dataset_name = sprintf('errorpos_gps_%04d(1,:)', trajectory_number);
        error_y_dataset_name = sprintf('errorpos_gps_%04d(2,:)', trajectory_number);
        error_z_dataset_name = sprintf('errorpos_gps_%04d(3,:)', trajectory_number);
        
        % Leggi i dati dei dataset HDOP e errore GPS lungo x, y e z
        hdop_data = h5read('sensor_records.hdf5', hdop_dataset_name);
        vdop_data = h5read('sensor_records.hdf5', vdop_dataset_name);
        error_x_data = eval(error_x_dataset_name);
        error_y_data = eval(error_y_dataset_name);
        error_z_data = eval(error_z_dataset_name);
        
        % Ciclo per riempire gli array dell'errore lungo x, y e z per ciascun range
        for j = 1:numel(hdop_ranges)-1
            %costruzione maschera booleana
            hdop_range_mask = hdop_data >= hdop_ranges(j) & hdop_data < hdop_ranges(j+1);
            if any(hdop_range_mask)
                if isempty(error_x_cells{j})
                    error_x_cells{j} = error_x_data(hdop_range_mask);
                else
                    error_x_cells{j} = [error_x_cells{j}, error_x_data(hdop_range_mask)];
                end
                if isempty(error_y_cells{j})
                    error_y_cells{j} = error_y_data(hdop_range_mask);
                else
                    error_y_cells{j} = [error_y_cells{j}, error_y_data(hdop_range_mask)];
                end
                 mean_hdop_values(j) = mean(hdop_data(hdop_range_mask));
            end
        end
        
        for k = 1:numel(vdop_ranges)-1
            vdop_range_mask = vdop_data >= vdop_ranges(k) & vdop_data < vdop_ranges(k+1);
            if any(vdop_range_mask)
                
                if isempty(error_z_cells{k})
                    error_z_cells{k} = error_z_data(vdop_range_mask);
                else
                    error_z_cells{k} = [error_z_cells{k}, error_z_data(vdop_range_mask)];
                end
                mean_vdop_values(k) = mean(vdop_data(vdop_range_mask));
            end
        end
    end
end

% Calcolo delle varianze per ogni array di errori lungo x, y e z
for m = 1:numel(error_x_cells)
    variance_x(m) = var(error_x_cells{m});
    variance_y(m) = var(error_y_cells{m});
end
for n = 1:numel(error_z_cells)
    variance_z(n) = var(error_z_cells{n});
end
for m = 1:numel(error_x_cells)
    variancenorm_x(m) = var(error_x_cells{m}) / mean_hdop_values(m); % Divisione per il valore medio di HDOP
    variancenorm_y(m) = var(error_y_cells{m}) / mean_hdop_values(m); % Divisione per il valore medio di HDOP
end
for n = 1:numel(error_z_cells)
    variancenorm_z(n) = var(error_z_cells{n}) / mean_vdop_values(n); % Divisione per il valore medio di VDOP
end

% Plotta le varianze dell'errore GPS
figure;
plot(hdop_ranges(1:end-1), variance_x,'ro-');
title('Varianza dell''errore lungo x in funzione degli HDOP');
xlabel('HDOP');

figure;
plot(hdop_ranges(1:end-1), variance_y,'ro-');
title('Varianza dell''errore lungo y in funzione degli HDOP');
xlabel('HDOP');

figure;
plot(vdop_ranges(1:end-1), variance_z,'ro-');
title('Varianza dell''errore lungo z in funzione del VDOP');
xlabel('VDOP');

% Plotta le varianze normalizzate riispetto all'HDOP dell'errore GPS
figure;
plot(hdop_ranges(2:end), variancenorm_x, 'ro-');
title('Varianza dell''errore normalizzato in funzione degli HDOP');
xlabel('HDOP');

figure;
plot(hdop_ranges(2:end), variancenorm_y, 'go-');
title('Varianza dell''errore normalizzato in funzione degli HDOP');
xlabel('HDOP');

figure;
plot(vdop_ranges(2:end), variancenorm_z, 'bo-');
title('Varianza dell''errore normalizzato in funzione del VDOP');
xlabel('VDOP');
%% Calcolo coefficienti HDOP utilizzando assieme tutti i dati di tutte le traiettorie senza dividerli per range
% Inizializzazione delle strutture dati per gli errori divisi per HDOP
errors_divided_by_hdop_x = [];
errors_divided_by_hdop_y = [];
errors_divided_by_vdop_z = [];
hdop_values_x = [];
hdop_values_y = [];
vdop_values_z = [];
mean_gps_errors = zeros(3, numel(dataset_names));
% Ciclo per calcolare gli errori divisi per HDOP per ogni traiettoria
for i = 1:numel(dataset_names)
    dataset_name = dataset_names{i};
    % Verifica se il nome del dataset è relativo alla traiettoria
    if startsWith(dataset_name, '/trajectory_')
        % Estrai il numero della traiettoria dal nome del dataset
        trajectory_number = sscanf(dataset_name, '/trajectory_%d/');
        
        % Costruisci il nome del dataset per HDOP e gli errori lungo x, y e z
        hdop_dataset_name = sprintf('%s/gps/HDOP', dataset_name);
        vdop_dataset_name = sprintf('%s/gps/VDOP', dataset_name);
        error_x_dataset_name = sprintf('errorpos_gps_%04d(1,:)', trajectory_number);
        error_y_dataset_name = sprintf('errorpos_gps_%04d(2,:)', trajectory_number);
        error_z_dataset_name = sprintf('errorpos_gps_%04d(3,:)', trajectory_number);
        
        % Leggi i dati dell'HDOP e degli errori lungo x, y e z
        hdop_data = h5read('sensor_records.hdf5', hdop_dataset_name);
        vdop_data = h5read('sensor_records.hdf5', vdop_dataset_name);
        error_x_data = (eval(error_x_dataset_name));
        error_y_data = (eval(error_y_dataset_name));
        error_z_data =(eval(error_z_dataset_name));

        % Calcola la media degli errori del GPS per questa traiettoria
        mean_error_x = mean(error_x_data);
        mean_error_y = mean(error_y_data);
        mean_error_z = mean(error_z_data);
        
        % Salva le medie degli errori del GPS nella matrice
        mean_gps_errors(:, i) = [mean_error_x; mean_error_y; mean_error_z];
        %bisonga togliere BIAS prima di calcolare l'errore con il
        %coeffficiente 
        errorx=error_x_data-mean_error_x;
        errory=error_y_data-mean_error_y;
        errorz=error_z_data-mean_error_z;
        % Calcola gli errori divisi per HDOP e salva gli HDOP corrispondenti
        errors_divided_by_hdop_x = [errors_divided_by_hdop_x, errorx ./ hdop_data];
        errors_divided_by_hdop_y = [errors_divided_by_hdop_y, errory ./ hdop_data];
        errors_divided_by_vdop_z = [errors_divided_by_vdop_z, errorz ./ vdop_data];
        hdop_values_x = [hdop_values_x, hdop_data];
        hdop_values_y = [hdop_values_y, hdop_data];
        vdop_values_z = [vdop_values_z, vdop_data];
    end
end

% Plot degli errori divisi per HDOP per X
figure;
scatter(hdop_values_x, errors_divided_by_hdop_x, 'ro', 'SizeData', 2);
title('Errori divisi per HDOP per X');
xlabel('HDOP');
ylabel('Errore diviso per HDOP');

% Plot degli errori divisi per HDOP per Y
figure;
scatter(hdop_values_y, errors_divided_by_hdop_y, 'go', 'SizeData', 2);
title('Errori divisi per HDOP per Y');
xlabel('HDOP');
ylabel('Errore diviso per HDOP');

% Plot degli errori divisi per VDOP per Z
figure;
scatter(vdop_values_z, errors_divided_by_vdop_z, 'bo', 'SizeData', 2);
title('Errori divisi per VDOP per Z');
xlabel('VDOP');
ylabel('Errore diviso per VDOP');

% Calcolo della varianza degli errori divisi per HDOP per X
variance_errors_divided_by_hdop_x = var(errors_divided_by_hdop_x);

% Calcolo della media degli errori divisi per HDOP per X
mean_errors_divided_by_hdop_x = mean(errors_divided_by_hdop_x);

% Calcolo della varianza degli errori divisi per HDOP per Y
variance_errors_divided_by_hdop_y = var(errors_divided_by_hdop_y);

% Calcolo della media degli errori divisi per HDOP per Y
mean_errors_divided_by_hdop_y = mean(errors_divided_by_hdop_y);

% Calcolo della varianza degli errori divisi per VDOP per Z
variance_errors_divided_by_vdop_z = var(errors_divided_by_vdop_z);

% Calcolo della media degli errori divisi per VDOP per Z
mean_errors_divided_by_vdop_z = mean(errors_divided_by_vdop_z);

%% calcolo di tutte le varianze per traiettoria

% Inizializzazione delle matrici per le varianze lungo x, y e z
variance_x = zeros(numel(dataset_names), 1);
variance_y = zeros(numel(dataset_names), 1);
variance_z = zeros(numel(dataset_names), 1);

% Ciclo per calcolare le varianze lungo x, y e z per ogni traiettoria
for i = 1:numel(dataset_names)
    dataset_name = dataset_names{i};
    % Verifica se il nome del dataset è relativo alla traiettoria
    if startsWith(dataset_name, '/trajectory_')
        % Estrai il numero della traiettoria dal nome del dataset
        trajectory_number = sscanf(dataset_name, '/trajectory_%d/');
        trajectory_number_array = trajectory_number + 1;
        
        % Costruisci il nome del dataset per l'HDOP e gli errori lungo x, y e z
        hdop_dataset_name = sprintf('%s/gps/HDOP', dataset_name);
        vdop_dataset_name = sprintf('%s/gps/VDOP', dataset_name);
        error_x_dataset_name = sprintf('errorpos_gps_%04d(1,:)', trajectory_number);
        error_y_dataset_name = sprintf('errorpos_gps_%04d(2,:)', trajectory_number);
        error_z_dataset_name = sprintf('errorpos_gps_%04d(3,:)', trajectory_number);
        
        % Leggi i dati dell'HDOP e degli errori lungo x, y e z
        hdop_data = h5read('sensor_records.hdf5', hdop_dataset_name);
        vdop_data = h5read('sensor_records.hdf5', vdop_dataset_name);
        error_x_data = eval(error_x_dataset_name);
        error_y_data = eval(error_y_dataset_name);
        error_z_data = eval(error_z_dataset_name);
        
        % Escludi i dati con HDOP maggiore di 10
        hdop_mask = hdop_data <= 7;
        vdop_mask = vdop_data <= 7;
        error_x_data = error_x_data(hdop_mask);
        error_y_data = error_y_data(hdop_mask);
        error_z_data = error_z_data(vdop_mask);
        
        % Calcola le varianze lungo x, y e z
        variance_x(trajectory_number_array) = var(error_x_data);
        variance_y(trajectory_number_array) = var(error_y_data);
        variance_z(trajectory_number_array) = var(error_z_data);
    end
end

% Plot delle varianze lungo x
figure;
plot(1:numel(dataset_names), variance_x, 'ro-');
title('Varianza dell''errore lungo x per traiettoria');
xlabel('Numero Tracciamento');
ylabel('Varianza lungo x');

% Plot delle varianze lungo y
figure;
plot(1:numel(dataset_names), variance_y, 'go-');
title('Varianza dell''errore lungo y per traiettoria');
xlabel('Numero Tracciamento');
ylabel('Varianza lungo y');

% Plot delle varianze lungo z
figure;
plot(1:numel(dataset_names), variance_z, 'bo-');
title('Varianza dell''errore lungo z per traiettoria');
xlabel('Numero Tracciamento');
ylabel('Varianza lungo z');


