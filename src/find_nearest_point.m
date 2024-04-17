function nearest_point = find_nearest_point(current_pose, point1, point2)
    L1 = sqrt((current_pose(1)-point1(1))^2 + (current_pose(2)-point1(2))^2);
    L2 = sqrt((current_pose(1)-point2(1))^2 + (current_pose(2)-point2(2))^2);
    if L1 <= L2
        nearest_point = [point1; point2];
    else
        nearest_point = [point2; point1];
    end
end

