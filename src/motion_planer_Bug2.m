clear
clc

figure();
grid on
hold on

%%%%% BUG 2 FOR MOTION PLANNING %%%%%
%% Rect obstacles
Obst1 = [7, 14 ;8, 14.5; 12, 11.5; 11, 11];
Obst3 = rect_generator([15, 15.5], -90);
Obst2 = [17, 20; 18, 20.5; 22, 15.5; 21, 15];
env = {Obst1, Obst2, Obst3};
env_copy = env;

draw_rect(Obst1);
draw_rect(Obst2);
draw_rect(Obst3);
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


%% check for intersection with obstacles
intersection_xy = [];
last_intersection = [0,0];
for obstacle_num = 1:size(env,2)
    
    for i = 1:4 % 4 lines in obstacles
        if i == 4
            next_point = 1;
        else
            next_point = i+1;
        end
        env{obstacle_num}(i,:)
        env{obstacle_num}(next_point,:)
        new_intersection_xy = intersection_point(start, target, env{obstacle_num}(i,:), env{obstacle_num}(next_point,:));
        if (new_intersection_xy(1)~=Inf && new_intersection_xy(1)~=-Inf) && (new_intersection_xy(2)~=Inf && new_intersection_xy(2)~=-Inf)
            if last_intersection ~= new_intersection_xy
                last_intersection = new_intersection_xy;
                intersection_xy = [intersection_xy ; new_intersection_xy, i, next_point, obstacle_num];
                scatter(new_intersection_xy(1), new_intersection_xy(2), 'bo', 'markerfacecolor', 'green');
            end
        end
    end
    i =1;
end
closest_intersections = order_closer_points(start, intersection_xy);

%% Go towards Target point
% Go from start point to first intersection %% FIRST OBSTACLE
current_pose = start;
obstacle_id = 1;
     
for intersections = 1:2:size(closest_intersections,1)
    %%% first intersection
     obstacle_id = closest_intersections(intersections,5);
     plot_line(current_pose, closest_intersections(intersections,1:2));
     current_pose = closest_intersections(intersections,1:2);
     
     %%% nearest corner
     nearest_point = find_nearest_point(current_pose, env{obstacle_id}(closest_intersections(intersections,3),:), env{obstacle_id}(closest_intersections(intersections,4),:));
     plot_line(current_pose, nearest_point(1,1:2));
     current_pose = nearest_point(1,1:2);
    
     %%% find the other two corners
     %%% delete the two points from first line we went through
     env_copy{obstacle_id}(closest_intersections(intersections,3),:) = 0;
     env_copy{obstacle_id}(closest_intersections(intersections,4),:) = 0;
     for i = 1:4
         if env_copy{obstacle_id}(i,1) ~= 0 && env_copy{obstacle_id}(i,2) ~=0
             third_point = i;
             env_copy{obstacle_id}(i,:) = 0;
             break
         end
     end
     for i = 1:4
         if env_copy{obstacle_id}(i,1) ~= 0 && env_copy{obstacle_id}(i,2) ~=0
             forth_point = i;
             env_copy{obstacle_id}(i,:) = 0;
         end
     end
     %%% find the next closest corner
     next_point_togo = find_nearest_point(current_pose, env{obstacle_id}(third_point,:), env{obstacle_id}(forth_point,:));
     next_intersection = closest_intersections(intersections+1,1:2)
     %%% check if intersection is on this side
     intersection_onThisSide = is_onTheLine([current_pose;next_point_togo(1,:)], next_intersection);
     if intersection_onThisSide == 0 %%% intersection is not in this line, go to the next corner
         %%% third corner to go
         plot_line(current_pose, next_point_togo(1,1:2));
         current_pose = next_point_togo(1,1:2);
         
         %%% next intersection
         plot_line(current_pose, next_intersection);
         current_pose = next_intersection;
     else
         %%% next intersection
         plot_line(current_pose, next_intersection);
         current_pose = next_intersection;
     end
end

%% plot to the target
plot_line(current_pose, target);