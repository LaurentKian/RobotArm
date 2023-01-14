function ret = edgecheck(u,v,dl,s)
%% 规划空间拓展枝检测 
% param: u        边的起始节点  |  n*1 double      
%        v        边的终止节点  |  n*1 double      
%        dl       检测分辨率    |  n*1 double      
%        s        仿真设置      |  struct          
%--------------------------------------------------------------------------
% returns： ret 碰撞检测结果    |  bool
%           0: collision free 
%           1: coliision
%--------------------------------------------------------------------------
% return: res bool
        
        ret = true;
        vec = (v - u) / norm(v - u);    % unit vector
        sdl = 0;
        dis = norm(v - u);
        while ~all(u == v)
            if sdl < dis
                sdl = sdl + dl;
                u = u + vec*dl;
            else
                u = v;
            end
            setconfig(s,u);
            if checkcollision(s)
                ret = false;
                break 
            end
        end
end