function closest_order = order_closer_points(start_point, intersections)
    ordered_list = [];
    shortest_index = 0;
    shortest_line = 100;
    interList_len = size(intersections,1);
    interList_elements = interList_len;
    ordList_len = size(ordered_list,1);
    while ordList_len ~= interList_len
        for i = 1:interList_elements
            % length from start to intersection point
            L = sqrt((start_point(1)-intersections(i,1))^2 + (start_point(2)-intersections(i,2))^2);
            if L < shortest_line
                shortest_line = L;
                shortest_index = i;
            end
        end
        interList_elements = interList_elements-1;
        shortest_line = 100;
        ordered_list = [ordered_list ; intersections(shortest_index, :)]
        intersections(shortest_index, :) = []
        ordList_len = size(ordered_list,1);
    end
    closest_order = ordered_list;
end

