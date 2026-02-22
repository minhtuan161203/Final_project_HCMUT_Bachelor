%% 1. CHU?N B? D? LI?U (Thay ph?n này b?ng d? li?u th?t c?a b?n)
% -----------------------------------------------------------
% Ví d?: N?u d? li?u th?t c?a b?n tên là 'out.scope_data'
% t = out.scope_data.time;
% set_point = out.scope_data.signals(1).values; % Gi? s? kênh 1 là Setpoint
% response  = out.scope_data.signals(2).values; % Gi? s? kênh 2 là ?áp ?ng

% --- T?O D? LI?U GI? ?? TEST CODE (Xóa ph?n này khi dùng th?t) ---
t = 0:0.0001:0.1; 
set_point = (square(2*pi*100*t) + 1) / 2; % T?o xung vuông (Setpoint)
response = set_point + 0.1*randn(size(t)); % T?o tín hi?u nhi?u (Response)
% -----------------------------------------------------------

%% 2. V? VÀ FORMAT ?? TH?
% T?o figure v?i n?n màu tr?ng (thay vì màu xám m?c ??nh)
fig = figure('Color', 'w', 'Position', [100, 100, 1000, 600]); 

% --- V? TÍN HI?U ---
hold on;
% V? ???ng Set Point (Giá tr? ??t): Màu ??, nét ??t, ??m
p1 = plot(t, set_point, 'r--', 'LineWidth', 2); 

% V? ???ng Response (?áp ?ng): Màu xanh d??ng, nét li?n, m?ng h?n chút
p2 = plot(t, response, 'b-', 'LineWidth', 1.2); 

hold off;

%% 3. TRANG TRÍ (FORMATTING)
% Tiêu ?? và nhãn tr?c
title('?áp ?ng h? th?ng ?i?u khi?n v? trí', 'FontSize', 16, 'FontWeight', 'bold', 'Color', 'k');
xlabel('Th?i gian (s)', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('Biên ?? (Amplitude)', 'FontSize', 14, 'FontWeight', 'bold');

% Chú thích (Legend)
lgd = legend([p1, p2], 'Giá tr? ??t (Set Point)', '?áp ?ng ra (Response)');
set(lgd, 'FontSize', 12, 'Location', 'best'); % T? ??ng ch?n v? trí t?t nh?t
legend boxoff; % B? khung vi?n c?a chú thích cho ??p

% L??i (Grid)
grid on;       % B?t l??i chính
grid minor;    % B?t l??i ph? (l??i nh?)
set(gca, 'GridAlpha', 0.3); % Làm m? l??i chính
set(gca, 'MinorGridAlpha', 0.1); % Làm m? l??i ph?

% Gi?i h?n tr?c (Tùy ch?nh theo d? li?u c?a b?n)
% xlim([0, 0.1]); 
% ylim([-0.2, 1.2]);

% Font ch? chung cho toàn b? tr?c
set(gca, 'FontSize', 12, 'LineWidth', 1.5, 'Box', 'on');

%% 4. L?U ?NH (Tùy ch?n)
% L?u ?nh ch?t l??ng cao ?? chèn vào báo cáo
% exportgraphics(fig, 'DoThi_Dep.png', 'Resolution', 300);