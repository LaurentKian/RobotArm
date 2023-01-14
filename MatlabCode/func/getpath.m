function path = getpath(z,VA)
%% 从规划搜索树矩阵中获取规划路径
%--------------------------------------------------------------------------
% param: z          路径终点坐标      | 1*1 double      
%        VA         搜索树矩阵        | n*m double      
%--------------------------------------------------------------------------
% return: path      规划路径          | k*n double 
    POS = 2:7;
    PAR = 9;
    i = 1;
    node = z;
    while node ~= 0
        path(:,i) = VA(node,POS)';
        node = VA(node,PAR);
        i = i + 1;
    end
    path = flip(path,2);
end