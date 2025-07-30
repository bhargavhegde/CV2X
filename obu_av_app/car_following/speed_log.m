clc; close all; clear

% Add log files
data = readtable("YYYYMMDDHHMMSSctrl.csv")

D = data.D;
D_nol = data.D_nol;
time = data.ctrl_time - data.ctrl_time(1);
v1 = data.v1 ;
v = data.v;
v_dot = data.v_dot;
flag = data.engaged;

figure(1)
% ---- Subplot 1 ----
subplot(4,1,1)
hold on; grid on; box on
plot(time, v, "LineWidth", 2, "Color", 'b')
plot(time, v1, "LineWidth", 2, "Color", 'r')
xlabel('$\mathrm{time\ [s]}$', 'Interpreter', 'latex', 'FontSize', 18)
ylabel('$\mathrm{Speed\ [m/s]}$', 'Interpreter', 'latex', 'FontSize', 18)
title('Speed vs. Time', 'Interpreter', 'latex', 'FontSize', 18)
legend('$v$','$v_1$', 'Interpreter', 'latex', 'FontSize', 18)
set(gca, 'FontSize', 18)

% ---- Subplot 2 ----
subplot(4,1,2)
hold on; grid on; box on
plot(time, D_nol, "LineWidth", 2, "Color", 'b')
xlabel('$\mathrm{time\ [s]}$', 'Interpreter', 'latex', 'FontSize', 18)
ylabel('$\mathrm{Headway\ [m]}$', 'Interpreter', 'latex', 'FontSize', 18)
title('Headway vs. Time', 'Interpreter', 'latex', 'FontSize', 18)
set(gca, 'FontSize', 18)

% ---- Subplot 3 ----
subplot(4,1,3)
hold on; grid on; box on
plot(time, v_dot, "LineWidth", 2, "Color", 'b')
xlabel('$\mathrm{time\ [s]}$', 'Interpreter', 'latex', 'FontSize', 18)
ylabel('$\dot{v}\ [\mathrm{m/s^2}]$', 'Interpreter', 'latex', 'FontSize', 18)
title('$\dot{v}$ vs. Time', 'Interpreter', 'latex', 'FontSize', 18)
set(gca, 'FontSize', 18)

% ---- Subplot 4 ----
subplot(4,1,4)
hold on; grid on; box on
plot(time, flag, "LineWidth", 2, "Color", 'g')
xlabel('$\mathrm{time\ [s]}$', 'Interpreter', 'latex', 'FontSize', 18)
ylabel('$Engaging$', 'Interpreter', 'latex', 'FontSize', 18)
title('$Engaging$ vs. Time', 'Interpreter', 'latex', 'FontSize', 18)
set(gca, 'FontSize', 18)
