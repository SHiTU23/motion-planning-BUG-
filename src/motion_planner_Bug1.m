clear
clc

figure();
grid on
hold on

%%%%% BUG 2 FOR MOTION PLANNING %%%%%
%% Rect obstacles
Obst1 = [7, 14 ;8, 14.5; 12, 11.5; 11, 11];
Obst3 = rect_generator([15, 15.5], -180);
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

current_pose = start;
last_intersection = [0,0];
while current_pose(:) ~= target(:)
    % draw a guidline from start to target 
    main_path_x = current_pose(1):1:target(1);
    T_m = (target(2) - current_pose(2)) / (target(1) - current_pose(1));
    T_c = target(2) - T_m * target(1);
    main_path_y = T_m * main_path_x + T_c;
    scatter(current_pose(1), current_pose(2), 'ko',  'markerfacecolor', 'black', 'displayName', 'start point');
    text(current_pose(1), current_pose(2)-1,'start');
    scatter(target(1), target(2), 'ko', 'markerfacecolor', 'black', 'displayName', 'target point');
    text(target(1), target(2)-1, 'target');
    plot(main_path_x, main_path_y, 'k--', 'linewidth', 1);


    %% check for intersection with obstacles
    intersection_xy = [];
    for obstacle_num = 1:size(env,2)

        for i = 1:4 % 4 lines in obstacles
            if i == 4
                next_point = 1;
            else
                next_point = i+1;
            end
            env{obstacle_num}(i,:)
            env{obstacle_num}(next_point,:)
            new_intersection_xy = intersection_point(current_pose, target, env{obstacle_num}(i,:), env{obstacle_num}(next_point,:));
            if (new_intersection_xy(1)~=Inf && new_intersection_xy(1)~=-Inf) && (new_intersection_xy(2)~=Inf && new_intersection_xy(2)~=-Inf)
                intersection_xy = [intersection_xy ; new_intersection_xy, i, next_point, obstacle_num];
            end
        end
    end
    closest_intersections = order_closer_points(current_pose, intersection_xy);
    if size(closest_intersections,1) ~= 0
        last_intersection = closest_intersections(1,1:2);
        scatter(closest_intersections(1,1), closest_intersections(1,2), 'bo', 'markerfacecolor', 'green');


        %% Go towards Target point
        % Go from start point to first intersection %% FIRST OBSTACLE
        obstacle_id = closest_intersections(1,5);
        main_trajectory = [];
        
        %% First go to the nearest intersection
        %%% in the first intersection
        next_pose = closest_intersections(1,1:2);
        plot_line(current_pose, next_pose);
        current_pose = next_pose;

        %%% find closest corner
        next_pose = find_nearest_point(current_pose, env{obstacle_id}(closest_intersections(1,3),1:2), env{obstacle_id}(closest_intersections(1,4),1:2));
        %%% plot the first corner
        plot_line(current_pose, next_pose(1,1:2));
        current_pose = next_pose(1,:);

        %%% first point for the next round 
        end_point_id = point_id(env{obstacle_id}, next_pose(2,:));
        end_point_pose = env{obstacle_id}(end_point_id,:);

        %%% first point to travel in main_trajectory
        in_point_id = point_id(env{obstacle_id}, next_pose(1,:));
        main_trajectory=[main_trajectory;env{obstacle_id}(in_point_id,:)];


        %%% delete the two points from first line we went through
        env_copy{obstacle_id}(closest_intersections(1,3),:) = 0;
        env_copy{obstacle_id}(closest_intersections(1,4),:) = 0;
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

        %%% second corner pose
        next_pose = find_nearest_point(current_pose, env{obstacle_id}(third_point,:), env{obstacle_id}(forth_point,:));
        plot_line(current_pose, next_pose(1,1:2));
        current_pose = next_pose(1,:);
        next_point_id = point_id(env{obstacle_id}, next_pose(1,:));
        main_trajectory=[main_trajectory;env{obstacle_id}(next_point_id,:)];

        %%% third corner
        next_point_id = point_id(env{obstacle_id}, next_pose(2,:));
        next_point = env{obstacle_id}(next_point_id,:);

        %%% go to the next intersection - find the closest point to the target in
        second_corner = current_pose;
        third_corner = next_point;
        x_values = [min(second_corner(1), third_corner(1)):0.1:max(second_corner(1), third_corner(1))];

        %%% from current pose to the other corner
        for x = 1:2:length(x_values)-1
            %%% find the line formula
            m = (third_corner(2)-second_corner(2))/(third_corner(1)-second_corner(1));
            c = third_corner(2) - m * third_corner(1);
            current_y = m*x_values(x)+c;
            next_y = m * x_values(x+1) + c;
            current_xy = [x_values(x), current_y];
            next_xy = [x_values(x+1), next_y];

            closest_point_to_target = find_nearest_point(target, current_xy, next_xy);  
        end
        
        scatter(closest_point_to_target(1,1), closest_point_to_target(1,2),'bo', 'markerfacecolor', 'black');
        main_trajectory=[main_trajectory;closest_point_to_target(1,:)];

        next_pose = third_corner;
        plot_line(current_pose, next_pose);
        current_pose = next_pose;

        int_len = sqrt((target(2)-closest_intersections(2,2))^2 + (target(1)-closest_intersections(2,1))^2);
        new_point_len = sqrt((target(2)-closest_point_to_target(2))^2 + (target(1)-closest_point_to_target(1))^2);


        next_pose = end_point_pose;
        plot_line(current_pose, next_pose);
        current_pose = next_pose;

        for i=1:size(main_trajectory,1)
            next_pose = main_trajectory(i,:);
            plot_line(current_pose, next_pose);
            current_pose = next_pose;
        end  
    else
        %%% go to target
        next_pose = target;
        plot_line(current_pose, next_pose);
        current_pose = next_pose;
    end
end
    
 