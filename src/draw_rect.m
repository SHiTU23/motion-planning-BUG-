function draw_rect(rectangle_points)
    plot([rectangle_points(1,1),rectangle_points(2,1)], [rectangle_points(1,2),rectangle_points(2,2)],'r-', 'linewidth', 0.5);
    plot([rectangle_points(2,1),rectangle_points(3,1)], [rectangle_points(2,2),rectangle_points(3,2)],'r-', 'linewidth', 0.5);
    plot([rectangle_points(3,1),rectangle_points(4,1)], [rectangle_points(3,2),rectangle_points(4,2)],'r-', 'linewidth', 0.5);
    plot([rectangle_points(4,1),rectangle_points(1,1)], [rectangle_points(4,2),rectangle_points(1,2)],'r-', 'linewidth', 0.5);
end

