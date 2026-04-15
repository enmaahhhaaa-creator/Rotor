% ================= 1. 数据录入 =================
% 升力数据 (col: 5~15)
col_lift = 5:15;
yasim_lift = [2951.168, 3547.781, 4140.559, 4728.898, 5312.210, ...
              5889.921, 6461.476, 7026.344, 7584.017, 8134.007, 8675.845];
rotorlib_lift = [3234.791, 3689.554, 4161.148, 4662.259, 5188.870, ...
                 5726.419, 6280.705, 6856.110, 7435.331, 8029.009, 8626.562];

% 扭矩数据 (col: 0~15)
col_torque = 0:15;
yasim_torque = [-136.31, -143.596, -165.414, -201.733, -252.488, ...
                -317.584, -396.910, -490.318, -597.644, -718.70, ...
                -853.273, -1001.132, -1162.024, -1335.680, -1521.810, -1720.110];
rotorlib_torque = [-572.582, -555.137, -550.214, -554.627, -566.671, ...
                   -606.037, -648.367, -703.187, -778.341, -864.9528, ...
                   -956.2582, -1083.750, -1194.886, -1339.451, -1486.292, -1646.987];

% ================= 2. 多项式拟合 (二次) =================
p_yasim_lift = polyfit(col_lift, yasim_lift, 2);
p_rotorlib_lift = polyfit(col_lift, rotorlib_lift, 2);

p_yasim_torque = polyfit(col_torque, yasim_torque, 2);
p_rotorlib_torque = polyfit(col_torque, rotorlib_torque, 2);

% 计算拟合曲线上的值（为了曲线平滑，可以使用更密集的点来画拟合线）
col_lift_fine = linspace(5, 15, 100);
col_torque_fine = linspace(0, 15, 100);

fit_yasim_lift = polyval(p_yasim_lift, col_lift_fine);
fit_rotorlib_lift = polyval(p_rotorlib_lift, col_lift_fine);
fit_yasim_torque = polyval(p_yasim_torque, col_torque_fine);
fit_rotorlib_torque = polyval(p_rotorlib_torque, col_torque_fine);

% ================= 3. 创建图形窗口 =================
figure('Name', 'Yasim vs Rotorlib 气动特性对比 (完整版)', 'Position', [100, 100, 800, 600]);

% --- 第一个图：升力对比 (区间 5-15) ---
subplot(2, 1, 1);
plot(col_lift, yasim_lift, 'bo', 'MarkerFaceColor', 'b', 'DisplayName', 'Yasim 升力');
hold on;
plot(col_lift, rotorlib_lift, 'rs', 'MarkerFaceColor', 'r', 'DisplayName', 'Rotorlib 升力');
plot(col_lift_fine, fit_yasim_lift, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Yasim 拟合');
plot(col_lift_fine, fit_rotorlib_lift, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Rotorlib 拟合');
hold off;
grid on;
xlim([0 15]); % 统一横坐标范围以便于上下图对比
xlabel('参数 (col)');
ylabel('升力');
title('升力对比 (Yasim vs Rotorlib)');
legend('Location', 'northwest');

% --- 第二个图：扭矩对比 (区间 0-15) ---
subplot(2, 1, 2);
plot(col_torque, yasim_torque, 'bo', 'MarkerFaceColor', 'b', 'DisplayName', 'Yasim 扭矩');
hold on;
plot(col_torque, rotorlib_torque, 'rs', 'MarkerFaceColor', 'r', 'DisplayName', 'Rotorlib 扭矩');
plot(col_torque_fine, fit_yasim_torque, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Yasim 拟合');
plot(col_torque_fine, fit_rotorlib_torque, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Rotorlib 拟合');
hold off;
grid on;
xlim([0 15]);
xlabel('参数 (col)');
ylabel('扭矩');
title('扭矩对比 (Yasim vs Rotorlib) - 包含低桨距区间');
legend('Location', 'southwest');