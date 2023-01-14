close all
clc
sim=remApi('remoteApi');
sim.simxFinish(-1); % just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID>-1)
    disp('Connected to remote API server');

%---------------------------------初始化----------------------------------%
    
    % 创建仿真平台接口对象
    s = struct("sim",sim,"clientID",clientID);
    
    % 关节指针和关节运动范围
    s.joints = [54, 57, 60, 63, 66, 69];
    s.vjoints = [17, 20, 23, 26, 29, 32];
%     joint_range1 = {deg2rad([-15,0]),...
%                    deg2rad([0,10]),...
%                    deg2rad([-30,0]),...
%                    deg2rad([-5,80]),...
%                    deg2rad([0,60]),...
%                    deg2rad([-200,50])};
    joint_range1 = {deg2rad([-180,180]),...
                   deg2rad([-95,155]),...
                   deg2rad([-180,69]),...
                   deg2rad([-180,180]),...
                   deg2rad([-130,130]),...
                   deg2rad([-180,180])};
    joint_range2 = {deg2rad([-15,25]),...
                   deg2rad([-10,25]),...
                   deg2rad([-25,10]),...
                   deg2rad([0,80]),...
                   deg2rad([10,60]),...
                   deg2rad([-200,20])};

    % 环境对象指针
    s.robotCollection = 2000000;
    s.obstacleCollection = 2000001;
    s.box1 = 107;
    s.box2 = 109;
    s.gripper1 = 75;
    s.gripper2 = 78;
    s.connector = 84;
    s.item = 113;

%----------------------------------采样-----------------------------------%
    s.joint_range = joint_range1;
    s.cspace_lebes = get_cspace_lebes(s);
%     samples1 = sampling(s,500);
%     s.joint_range = joint_range2;
%     s.cspace_lebes = get_cspace_lebes(s);
%     samples2 = sampling(s,500);

%----------------------------------规划-----------------------------------%
   xi1 = [deg2rad(0);      % 第一段路径起始位置
          deg2rad(0);
          deg2rad(0);
          deg2rad(0);
          deg2rad(0);
          deg2rad(0)];

    Xg1 = deg2rad([-1.224e+01, +9.943e-01, -2.106e+01,... % 物体抓取位置
        +6.165e+01, +5.140e+01, -1.151e+02]');

    xi2 = Xg1;  % 第二段路径起始位置

    Xg2 = deg2rad([+1.721e+01, +2.333e+01, +5.581e+00,... % 物体放置位置
        +9.716e+00, +1.842e+01, +9.642e+00]');

    [z1,VA1] = FMT_with_weight(xi1,Xg1,samples1,s);
    setconfig(s,[0,0,0,0,0,0]);
    path1 = getpath(z1,VA1);
    path1 = pathsimplfy(path1,s);   % 路径简化
    path1 = pathsmooth(path1,s);    % 路径平滑

    [z2,VA2] = FMT_with_weight(xi2,Xg2,samples2,s);
    setconfig(s,[0,0,0,0,0,0]);
    path2 = getpath(z2,VA2);
    path2 = pathsimplfy(path2,s);  % 路径简化
    path2 = pathsmooth(path1,s);   % 路径平滑

    path = [path1,path2(:,2:end)]; % 组合两段路径

%-----------------------------执行抓取任务--------------------------------%
    step1 = size(path1,2);
    dt = 0.5;
    % move to the item
    for i = 1:step1
        settargetconfig(s,path(:,i))
        pause(dt)
    end
    % grab the item
    setgripper(s,1)  
    % move to the goal
    for i = step1+1:size(path,2)
        settargetconfig(s,path(:,i))
        pause(dt)
    end
    % place the item
    setgripper(s,0)  
else
    disp('Failed connecting to remote API server');
end

%------------------------------脚本终止处理-------------------------------%

sim.delete(); % call the destructor!

disp('Program ended');

function ret = get_cspace_lebes(s)
    ret = 1;
    for i = 1:size(s.joint_range,2)
        ret = ret*(s.joint_range{i}(2)-s.joint_range{i}(1));
    end
end