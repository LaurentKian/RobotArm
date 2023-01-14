function newpath = pathsimplfy(path,s)
%% 路径简化
% param: path      初始路径        | n*n*1 double    
%        s         仿真设置        | struct           
%--------------------------------------------------------------------------
% return: newpath  简化路径        | k*n*1 double  

%---------------------Simplify with binary search-------------------------%

end_idx = size(path,2);
right_idx = end_idx; 
left_idx = 1; 
mid_idx = left_idx + floor((right_idx-left_idx)/2);
idx_array = Array([],size(path,2)); 
idx_array.add(right_idx);

dl = deg2rad(2);

while true

    if end_idx == 2 || edgecheck(path(:,1),path(:,end_idx),dl,s)
        idx_array.add(1)
        break
    end

    while left_idx + 1 < right_idx
        if edgecheck(path(:,mid_idx),path(:,end_idx),dl,s)
            right_idx = mid_idx;
        else
            left_idx = mid_idx;
        end
        mid_idx = left_idx + floor((right_idx-left_idx)/2);
    end

    end_idx = right_idx;
    idx_array.add(end_idx)
    left_idx = 1;
end

idxs = idx_array.show();
newpath = path(:,flip(idxs));