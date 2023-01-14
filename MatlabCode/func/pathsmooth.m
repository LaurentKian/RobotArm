function path_new = pathsmooth(path,s,num,dl)
%% 三次B-样条曲线平滑
% param：path： 原始路径                        | d*n double
%        s：    仿真平台接口                    | struct
%        num:   平滑路径节点数目（默认为50）    | 1*1 double
%        dl:    碰撞检测分辨率（默认为2°）      | 1*1 double
%--------------------------------------------------------------------------
% return： path_new:  平滑后路径                | d*n double

MAX_ITER = 5; % 最大迭代次数
INTER_NUM = 2; % 路径插值各段增加点数

if nargin < 4
    dl = 2;           % 默认碰撞检测分辨率为 2°
    if nargin < 3
        num = 50;     % 默认平滑路径节点数目为 50
    end
end

% 三次B-样条参数
t_interval = [0,1];
t_samples = linspace(0,1,num);

% 如果路径点小于三个点，则进行插值
path = pathinter(path,INTER_NUM);

% 以path作为控制点，生成平滑路径
path_new = bsplinepolytraj(path,t_interval,t_samples); 

% 进行碰撞检测并迭代更新平滑路径
for i = 1:MAX_ITER
    
    iter_flag = true;
    
    % 碰撞检测
    for j = 1:size(path_new,1)-1
        if ~edgecheck(path_new(:,i),path_new(:,i+1),dl,s)
            iter_flag = false;
            break
        end
    end
    
    if iter_flag
        return
    end
    
    % 增加原始路径插值点数目，并重新生成平滑路径
    path = pathinter(path,INTER_NUM);
    path_new = bsplinepolytraj(path,t_interval,t_samples);

end

warning("无法得到无碰撞的平滑路径，返回原始路径")
path_new = path ;

end