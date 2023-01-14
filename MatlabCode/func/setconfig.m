function setconfig(s,config)
%% 设置虚拟机械臂的关节变量
%--------------------------------------------------------------------------
% param: s          仿真设置        | struct          
%        config     关节状态        | n*1 double     
%--------------------------------------------------------------------------
% return: none
    for i  = 1:length(s.vjoints)  
        statu = s.sim.simxSetJointPosition(s.clientID,s.vjoints(i), ...
            config(i),s.sim.simx_opmode_oneshot);
        if statu > 1
            warning("Set configuration failed!, returnCode = %d",statu)
        end
    end
end