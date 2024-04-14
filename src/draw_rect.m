function draw_rect(rectangle_points)
    for i = 1:4
        if i == 4
            next_point = 1;
        else
            next_point = i+1;
        end
        %% find the m for points 1 and 2 (1,1) = x (1,2) = y
        if rectangle_points(i,1) < rectangle_points(next_point,1)
            x = rectangle_points(i,1):1:rectangle_points(next_point,1);
        else
            x = rectangle_points(next_point,1):1:rectangle_points(i,1);
        end
        m = (rectangle_points(i,2) - rectangle_points(next_point,2)) / (rectangle_points(i,1) - rectangle_points(next_point,1));
        c = rectangle_points(i,2) - m * rectangle_points(i,1);
        y = m*x+c;
        plot(x,y,'r-', 'linewidth', 0.5);
    end
end

