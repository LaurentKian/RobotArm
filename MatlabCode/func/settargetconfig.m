function settargetconfig(s,config)
%% 设置实际机械臂的目标关节变量
%--------------------------------------------------------------------------
% param: s          仿真设置        | struct          
%        config     关节状态        | n*1 double      
%--------------------------------------------------------------------------
% return: none
    for i = 1:length(s.joints)
        statu = s.sim.simxSetJointTargetPosition(s.clientID,s.joints(i),...
        config(i),s.sim.simx_opmode_oneshot);
        if statu > 1
            warning("Set configuration failed!, returnCode = %d",statu)
        end
    end
end