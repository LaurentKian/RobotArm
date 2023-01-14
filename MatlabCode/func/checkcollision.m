function res = checkcollision(s)
%% 碰撞检测
% param: s  接口设置 | stuct  
%--------------------------------------------------------------------------
% returns res  碰撞检测结果  | bool
%              0: 无碰撞
%              1: 产生碰撞
%--------------------------------------------------------------------------
    [statu1,res1] = s.sim.simxCheckCollision(s.clientID,s.robotCollection,...
    s.obstacleCollection,s.sim.simx_opmode_blocking);
    [statu2,res2] = s.sim.simxCheckCollision(s.clientID,s.robotCollection,...
    s.box1,s.sim.simx_opmode_blocking);
    [statu3,res3] = s.sim.simxCheckCollision(s.clientID,s.robotCollection,...
    s.box2,s.sim.simx_opmode_blocking);
    if statu1 > 1 || statu2 > 1 || statu3 > 1
        warning("Collision detection failed!, returnCode = %d",statu1)
%         res = -1;
    end
    if res1 == 0 && (res2 == 0 && res3 == 0)
        res = 0;
    else
        res = 1;
    end
end