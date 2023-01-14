function setgripper(s,state)
%% 设置手爪的状态 
%--------------------------------------------------------------------------
% param: s          仿真设置    | struct          
%        state      手爪状态    | bool            
%                   1:  闭合
%                   0:  打开
%--------------------------------------------------------------------------
open_gap = 0.006;
close_gap = -0.003;

if state == 1
    setGripperTargetPos(s,close_gap)
    s.sim.simxSetObjectParent(s.clientID,s.item,s.connector,1,...
        s.sim.simx_opmode_oneshot)
else
    setGripperTargetPos(s,open_gap)
    s.sim.simxSetObjectParent(s.clientID,s.item,-1,1,...
        s.sim.simx_opmode_oneshot)
end
end

function setGripperTargetPos(s,gap)
    statu = s.sim.simxSetJointTargetPosition(s.clientID,s.gripper1,-gap/2,...
        s.sim.simx_opmode_oneshot);
    if statu > 1
            warning("Set configuration failed!, returnCode = %d",statu)
    end
    statu = s.sim.simxSetJointTargetPosition(s.clientID,s.gripper2,gap/2,...
        s.sim.simx_opmode_oneshot);
    if statu > 1
            warning("Set configuration failed!, returnCode = %d",statu)
    end
end