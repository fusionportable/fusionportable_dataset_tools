clc; clear; close all;
color_list = acquire_color_list();

%%
filename = 'example_data/ugv_parking00/RTK.txt';
filename2 = 'example_data/ugv_parking00/CAD.txt';
filename3 = 'example_data/ugv_parking00/Calibrated.txt';
table_data = readtable(filename);
table_data2 = readtable(filename2);
table_data3 = readtable(filename3);

%%
data = parse_table_data(table_data);
data2 = parse_table_data(table_data2);
data3 = parse_table_data(table_data3);

figure(1); 
hold on;
plot3(data.x, data.y, data.z, 'Color', color_list(1, 1:3), 'LineWidth', 2, 'LineStyle', '-');
% plot3(data.x, data.y, data.z, '*', 'Color', color_list(1, 1:3));
% plot3(data.x(1), data.y(1), data.z(1), 'o', 'Color', color_list(2, 1:3), ...
%   'MarkerSize', 20, 'MarkerFaceColor', color_list(3, 1:3));
% plot3(data.x(end), data.y(end), data.z(end), 'd', 'Color', color_list(3, 1:3), ...
%   'MarkerSize', 20, 'MarkerFaceColor', color_list(4, 1:3));

plot3(data2.x, data2.y, data2.z, 'Color', color_list(2, 1:3), 'LineWidth', 2, 'LineStyle', '-');

plot3(data3.x, data3.y, data3.z, 'Color', color_list(4, 1:3), 'LineWidth', 2, 'LineStyle', '--');
% plot3(data.x, data.y, data.z, '*', 'Color', color_list(1, 1:3));
% plot3(data2.x(1), data2.y(1), data2.z(1), 'o', 'Color', color_list(2, 1:3), ...
%   'MarkerSize', 20, 'MarkerFaceColor', color_list(3, 1:3));
% plot3(data2.x(end), data2.y(end), data2.z(end), 'd', 'Color', color_list(3, 1:3), ...
%   'MarkerSize', 20, 'MarkerFaceColor', color_list(4, 1:3));
sprintf('t.secs>=%.9f and t.secs<=%.9f', data.time(1), data.time(end))

legend({'Motion of 3DM-GQ7', 'Motion of Encoders (Before Calib.)', 'Motion of Encoders (After Calib.)'}, 'Location', 'northeast', 'FontSize', 25,'fontname','Times New Roman');
% legend({'Method 2'}, 'Location', 'northwest', 'FontSize', 20);
grid on; ax = gca; ax.GridLineStyle = '--'; ax.GridAlpha = 0.3; box on;
set(gca, 'FontName', 'Times', 'FontSize', 25, 'LineWidth', 2);
xlabel("X [m]"); ylabel("Y [m]"); zlabel("Z [m]");

hold off;
axis on; 
axis equal;
grid on;
xlim([-17, 15]);
ylim([-1, 28]);

%%
function data = parse_table_data(table_data)
  data = struct();  
  data.time = table_data.Var1;
  data.x = table_data.Var2;
  data.y = table_data.Var3;
  data.z = table_data.Var4;
  data.qx = table_data.Var5;
  data.qy = table_data.Var6;
  data.qz = table_data.Var7;
  data.qw = table_data.Var8;
end