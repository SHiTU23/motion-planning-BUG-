clear
clc

%% Rect obstacles
Obst1 = [7, 14 ;8, 14.5; 12, 11.5; 11, 11];


%% start and target points
%%%% choose form the plot
start = [5 , 10]; % x, y
target = [25, 20]; % x, y

xlim([min(start(1), target(1))-2 , max(start(1),target(1))+2]);
ylim([min(start(2), target(2))-2 , max(start(2),target(2))+2]);

% draw a guidline from start to target 
main_path_x = start(1):1:target(1);
T_m = (target(2) - start(2)) / (target(1) - start(1));
T_c = target(2) - T_m * target(1);
main_path_y = T_m * main_path_x + T_c;
scatter(start(1), start(2), 'ko',  'markerfacecolor', 'black', 'displayName', 'start point');
text(start(1), start(2)-1,'start');
scatter(target(1), target(2), 'ko', 'markerfacecolor', 'black', 'displayName', 'target point');
text(target(1), target(2)-1, 'target');
plot(main_path_x, main_path_y, 'k--', 'linewidth', 1);

