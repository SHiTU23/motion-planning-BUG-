%% line1_point1 = [x y]
function Intersection_coordinate = intersection_point(start_xy, target_xy,obstacle_P1_xy, obstacle_P2_xy)
    %% We have 4 points which is 2 lines to find the intersect between them
    x_range =[min(obstacle_P1_xy(1), obstacle_P2_xy(1)) ,max(obstacle_P1_xy(1), obstacle_P2_xy(1))];
    y_range = [min(obstacle_P1_xy(2), obstacle_P2_xy(2)) ,max(obstacle_P1_xy(2), obstacle_P2_xy(2))];
    %% Start to Target line:
    %T_x = start_x:1:target_x;
    trajectory_m = (target_xy(2) - start_xy(2)) / (target_xy(1) - start_xy(1));
    trajectory_c = target_xy(2) - (trajectory_m * target_xy(1));
    % trajectory_line = trajectory_m * T_x + trajectory_c;
    
    %% one line of obstacle
    %O_x = obstacle_P1_x:1:obstacle_P2_x;
    obstacle_m = (obstacle_P2_xy(2) - obstacle_P1_xy(2)) / (obstacle_P2_xy(1) - obstacle_P1_xy(1));
    obstacle_c = obstacle_P2_xy(2) - (obstacle_m * obstacle_P2_xy(1));
    %obstacle_line = obstacle_m * O_x + obstacle_c;
    
    %% Intersection between two lines
    a1 = (trajectory_c * start_xy(2) - trajectory_c*target_xy(2)) / (start_xy(1) * target_xy(2) - target_xy(1) * start_xy(2));
    b1 = (a1*target_xy(1) + trajectory_c) / target_xy(2);
    
    a2 = (obstacle_c * obstacle_P1_xy(2) - (obstacle_c*obstacle_P2_xy(2))) / (obstacle_P1_xy(1) * obstacle_P2_xy(2) - obstacle_P2_xy(1) * obstacle_P1_xy(2));
    b2 = (a2*obstacle_P2_xy(1) + obstacle_c) / obstacle_P2_xy(2);
    
    x_int = (b1 * obstacle_c - b2 * trajectory_c) / (a1*b2 - a2*b1);
    y_int = (a1 * obstacle_c - a2 * trajectory_c) / (a1*b2 - a2*b1);
    
    %% return intersection coordinates
    if (x_int <= x_range(2)) && (x_int>=x_range(1)) && (x_int > start_xy(1) && y_int > start_xy(2))
        Intersection_coordinate = [x_int, y_int];   
    elseif isnan(x_int) && isnan(y_int)
            x = [x_range(1):0.1:x_range(2)];
            for i = 1:1:length(x)
                if x(i)==obstacle_P1_xy(1)
                    y = trajectory_m * x(i) + trajectory_c;
                    if y >= y_range(1) && y <= y_range(2)
                        Intersection_coordinate = [x(i), y]; 
                    else
                        Intersection_coordinate = [Inf, Inf];
                    end
                end
            end
    else
        Intersection_coordinate = [Inf, Inf];   
    end
end

