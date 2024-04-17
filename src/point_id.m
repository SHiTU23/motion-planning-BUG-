function point_ID = point_id(reference_point_list, checking_point)
    for i = 1: size(reference_point_list,1)
       if  checking_point(1) == reference_point_list(i,1) && checking_point(2) == reference_point_list(i,2)
           point_ID = i;
           break
       end
    end
end