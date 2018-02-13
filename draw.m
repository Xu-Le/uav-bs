clear;
clc;

GE = csvread('user.csv');
SE = csvread('servedUsers.csv');
UAVs = csvread('UAVs.csv');

figure('OuterPosition', [300 40 680 740]);
dummy = [ 0, 0; 1, 0 ];
darkgreen = [ 0/255 128/255 0/255 ];
darkcyan = [ 0/255 128/255 128/255 ];
orange = [ 255/255 128/255 0/255 ];
purple = [ 64/255 0/255 64/255 ];
h1 = plot(dummy(:, 1), dummy(:, 2), 'Color', darkgreen); hold on;
h2 = plot(dummy(:, 1), dummy(:, 2), 'Color', orange); hold on;
h3 = plot(dummy(:, 1), dummy(:, 2), 'Color', purple);
legend([h1, h2, h3], '1930-1950MHz', '1950-1970MHz', '1970-1990MHz');
legend('boxoff');
xlabel('x (m)', 'FontName', 'Times New Roman', 'FontSize', 14);
ylabel('y (m)', 'FontName', 'Times New Roman', 'FontSize', 14);
title('A case with 885 users being served ( N: 1000, K: 10 )', 'FontName', 'Times New Roman', 'FontSize', 14);
hold on;
h = plot(GE(:,1), GE(:,2), 'b.', 'MarkerSize', 8);
axis([0 2000 0 2000]);
hold on;
plot(SE(:,1), SE(:,2), 'g.', 'MarkerSize', 8);
hold on;

K = size(UAVs, 1);
colorS = 'g';
for k = 1:K
    ox = UAVs(k, 1);
    oy = UAVs(k, 2);
    r = UAVs(k, 4);
    % text(ox, oy, int2str(k));
    if UAVs(k, 7) == 1930
        colorS = darkgreen;
    elseif UAVs(k, 7) == 1950
        colorS = orange;
    else
        colorS = purple;
    end
    plot(ox, oy, '+', 'Color', colorS, 'MarkerSize', 8);
    rectangle('Position', [ox - r, oy - r, 2*r, 2*r], 'EdgeColor', colorS, 'LineWidth', 1, 'Curvature', [1, 1]);
end
