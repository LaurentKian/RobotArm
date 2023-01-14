function path_new = pathinter(path,num)
%% 路径插值算法
% param：path： 原始路径 | 6*n double
%        nums:  两点间的插值点数目 | 1*1 double
%--------------------------------------------------------------------------
% return path_new  插值后的路径 | 6*n double
% -------------------------------------------------------------------------
dim = size(path,1);  % 路径节点的维度
n = size(path,2); % 原路径节点数目
path_new = zeros(6,n+(n-1)*num);

%各段路径插值
for i = 1:n-1
    for j = 1:dim
        start = (num+1)*(i-1)+1; % 各段起点位置
        path_new(j,start) = path(j,i);
        path_add = linspace(path(j,i),path(j,i+1),num+2); 
        path_new(j,start+1:start+num) = path_add(2:num+1);  
    end
end

path_new(:,end) = path(:,end); % 补上最后一个路径点

end