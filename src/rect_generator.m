function rectangle_points = rect_generator(A,rectangle_angle)
    w = 5;
    h = 1;

    angle_radians = deg2rad(rectangle_angle);
    % Calculate the second point B
    B = [A(1) + w * cos(angle_radians), A(2) + w * sin(angle_radians)];
    % Compute the other two corners (C and D)
    C = [B(1) - h * sin(angle_radians), B(2) + h * cos(angle_radians)];
    D = [A(1) - h * sin(angle_radians), A(2) + h * cos(angle_radians)];
    
    rectangle_points = [A ; B ; C ; D];
end

