function plot_line(start_point,target_point)
    x = start_point(1):0.2:target_point(1);
    m = (target_point(2) - start_point(2)) / (target_point(1) - start_point(1));
    c = start_point(2) - m * start_point(1);
    y = m * x + c;
    h_line = line(x, y, 'Color', 'r', 'Marker', '*', 'MarkerSize', 10, 'LineWidth', 1);
    for i = 1:length(x)
        set(h_line,'XData',x(i),'YData', y(i));
        drawnow;
        pause (0.5);
    end
end

