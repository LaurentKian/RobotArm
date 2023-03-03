close all
clc
% 初始化
fprintf('Program started\n')
client = RemoteAPIClient();
sim = client.getObject('sim');
defaultIdleFps = sim.getInt32Param(sim.intparam_idle_fps);
sim.setInt32Param(sim.intparam_idle_fps, 0);

% 获得机械臂的Handle
robotBase = sim.getObject('./Frame/VABB_IRB1300');
% 为了能够延续原来的代码我们还是选择保留结构体
% clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);
s = struct("sim",sim,"clientID",0,"robotBase",robotBase);
jointNum = 6;
jointHandles = zeros(6,1);
% 获得Joint的Handles
for i = 1:jointNum
    jointHandles(i) = sim.getObject(strcat('./joint',num2str(i)));
end
tip = sim.getObject('./tip');
% 创建collection
robotCollection = sim.createCollection(0);
sim.addItemToCollection(robotCollection,sim.handle_tree,robotBase,0);
% % 我觉得这里还是要目标物体也给加进去，不然这样还是会碰撞
target=sim.getObject('./Platform/item_respondable');
% sim.addItemToCollection(robotCollection, sim.handle_single,target,0);
% 创建一个新的collection
box=sim.getObject('./box_respondable');
robotCollectionALL = sim.createCollection(0);
sim.addItemToCollection(robotCollectionALL,sim.handle_all,0,0);
sim.addItemToCollection(robotCollectionALL,sim.handle_tree ,robotBase,1);
sim.addItemToCollection(robotCollectionALL,sim.handle_single,target,1);
sim.addItemToCollection(robotCollectionALL,sim.handle_single,box,1);

% 检测分辨率
checkerResol = 3;
% 实例化碰撞检测器
checker = CheckerSim(sim,jointHandles,robotCollection,robotCollectionALL,checkerResol);

% 关节角度
joint_range1 = [deg2rad([-180,180]);
                   deg2rad([-95,155]);
                   deg2rad([-180,69]);
                   deg2rad([-180,180]);
                   deg2rad([-130,130]);
                   deg2rad([-180,180])];
               
joint_range2 = [deg2rad([-15,25]);...
                   deg2rad([-10,25]);...
                   deg2rad([-25,10]);...
                   deg2rad([0,80]);...
                   deg2rad([10,60]);...
                   deg2rad([-200,20])];

     s.joint_range = joint_range1;
%     传换成Cspace下
    s.cspace_lebes = get_cspace_lebes(s);

%  采样器
%     sampler = SamplerUni(checker,joint_range1);
    sampler = SamplerUniSim(checker,joint_range1,sim);
%     采样点
    n=100;
%     采样的问题！！
    samples1 = sampler.samplingValid(n);    
%     samples1 = sampling(s,jointHandles,1000);
    s.joint_range = joint_range2;

      sampler2 = SamplerUniSim(checker,joint_range2,sim);
      samples2 = sampler2.samplingValid(n); 
        
        xi1 = [deg2rad(0);      % 第一段路径起始位置
          deg2rad(0);
          deg2rad(0);
          deg2rad(0);
          deg2rad(0);
          deg2rad(0)];

    Xg1 = deg2rad([-1.224e+01, +9.943e-01, -2.106e+01,... % 物体抓取位置
        +6.165e+01, +5.140e+01, -1.151e+02]');
    [z1,VA1] = FMT_with_weight(xi1,Xg1,samples1,s,checker);

    xi2 = Xg1;  % 第二段路径起始位置
    Xg2 = deg2rad([+1.721e+01, +2.333e+01, +5.581e+00,... % 物体放置位置
        +9.716e+00, +1.842e+01, +9.642e+00]');
    
%     setconfig(s,[0,0,0,0,0,0]);
    path1 = getpath(z1,VA1);
    path1 = pathsimplfy(path1,s);   % 路径简化
    path1 = pathsmooth(path1,s,checker);    % 路径平滑

    [z2,VA2] = FMT_with_weight(xi2,Xg2,samples2,s,checker);
%     setconfig(s,[0,0,0,0,0,0]);
    path2 = getpath(z2,VA2);
    path2 = pathsimplfy(path2,s,checker);  % 路径简化
    path2 = pathsmooth(path2,s,checker);   % 路径平滑

    path = [path1,path2(:,2:end)]; % 组合两段路径
    
    step1 = size(path1,2);
    dt = 0.05;
    % move to the item
    for i = 1:step1
        setconfig(sim,jointHandles,path(:,i))
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
    
    
