clear; close all; clc

% this code is written for justifing the RTK GPS performanace
% author: Haosong Xiao, Department of Mechanical and Aerospace Engineering
% date: 06/15/2025 (v1) 

%% user input
file = input('Enter the path to the data file (e.g., "data.csv"):');
data = readtable(file);

%% NAV
NAV_time = data.NAV_time;
NAV_lat = data.MAV_lat;
NAV_long = data.NAv_long;
NAV_speed = data.NAV_speed;

%% RTK
RTK_time = data.RTK_time;
RTK_lat = data.RTK_lat;
RTK_long = data.RTK_long;
RTK_speed = data.RTK_speed;
err_hor = data.err_lat;
err_vert = data.err_long;

%% plotter
n = length(RTK_time);
figure(1); clf;
hold on; box on; grid on;
plot(1:n, NAV_time - NAV_time(1), 'b.', 'MarkerSize', 5, 'LineWidth', 2);
plot(1:n, RTK_time - RTK_time(1), 'r.', 'MarkerSize', 5, 'LineWidth', 2);
xlabel('Index', 'Interpreter', 'latex', 'FontSize', 18);
ylabel('Timestamp [ms]', 'Interpreter', 'latex', 'FontSize', 18);
legend({'NAV', 'RTK'}, 'Interpreter', 'latex', 'FontSize', 18);
title('Synchronization Check', 'Interpreter', 'latex', 'FontSize', 18);
set(gca, 'FontSize', 18); 

figure(2); clf; 
hold on; box on; grid on;
geoaxes;              
geobasemap streets;    
hold on;              
geoscatter(NAV_lat, NAV_long, 50, 'b', 'filled');
geoscatter(RTK_lat, RTK_long, 50, 'r', 'filled');
legend({'NAV', 'RTK'}, 'Location', 'best');
title('NAV vs RTK');

figure(3); clf; 
hold on; box on; grid on;
plot(NAV_time - NAV_time(1), NAV_speed, 'b', 'MarkerSize', 5, 'LineWidth', 2);
plot(RTK_time - RTK_time(1), RTK_speed, 'r', 'MarkerSize', 5, 'LineWidth', 2);
xlabel('Timestamp [ms]', 'Interpreter', 'latex', 'FontSize', 18);
ylabel('speed [m/s]', 'Interpreter', 'latex', 'FontSize', 18);
legend({'NAV', 'RTK'}, 'Interpreter', 'latex', 'FontSize', 18);
title('Speed Check', 'Interpreter', 'latex', 'FontSize', 18);
% ylim([0,1])
set(gca, 'FontSize', 18); 

figure(4); clf;
hold on; box on; grid on;
plot(RTK_time - RTK_time(1), err_hor, 'k', 'MarkerSize', 5, 'LineWidth', 2);
plot(RTK_time - RTK_time(1), err_vert, 'k--', 'MarkerSize', 5, 'LineWidth', 2);
xlabel('Timestamp [ms]', 'Interpreter', 'latex', 'FontSize', 18);
ylabel('error [m]', 'Interpreter', 'latex', 'FontSize', 18);
legend({'horizontal', 'vertical'}, 'Interpreter', 'latex', 'FontSize', 18);
title('RTK error Check', 'Interpreter', 'latex', 'FontSize', 18);
set(gca, 'FontSize', 18); 
