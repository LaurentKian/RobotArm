close all
clc
sim=remApi('remoteApi');
sim.simxFinish(-1); % just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID>-1)
    disp('Connected to remote API server');

%------------------------------Initialization-----------------------------%
    
    % setting
    s = struct("sim",sim,"clientID",clientID);
    
    % joint handles
    s.joints = [52, 55, 58, 61, 64, 67];
    s.vjoints = [18, 21, 24, 27, 30, 33];
    joint_range1 = {deg2rad([-10,0]),...
                   deg2rad([0,5.5]),...
                   deg2rad([-15,0]),...
                   deg2rad([-5,0]),...
                   deg2rad([0,20]),...
                   deg2rad([-5,0])};
    joint_range2 = {deg2rad([-7,25]),...
                   deg2rad([-5,25]),...
                   deg2rad([-15,-10]),...
                   deg2rad([-5,16]),...
                   deg2rad([15,35]),...
                   deg2rad([-5,10])};

    % objects handles
    s.robotCollection = 2000000;
    s.obstacleCollection = 2000001;
    s.box1 = 112;
    s.box2 = 114;
    s.gripper1 = 73;
    s.gripper2 = 76;
    s.connector = 82;
    s.item = 116;

%--------------------------------Sampling---------------------------------%
    s.joint_range = joint_range1;
    s.cspace_lebes = get_cspace_lebes(s);
    samples1 = sampling(s,500);
    s.joint_range = joint_range2;
    s.cspace_lebes = get_cspace_lebes(s);
    samples2 = sampling(s,500);
%--------------------------------Planning---------------------------------
    xi1 = [deg2rad(0);
          deg2rad(0);
          deg2rad(0);
          deg2rad(0);
          deg2rad(0);
          deg2rad(0)];

    Xg1 = [deg2rad(-5.86);
          deg2rad(6.20);
          deg2rad(-13.74);
          deg2rad(-3.35);
          deg2rad(17.60);
          deg2rad(-3.23)];

    xi2 = Xg1;

    Xg2 = [deg2rad(22.34);
    deg2rad(20.84);
    deg2rad(-15.29);
    deg2rad(16.27);
    deg2rad(33.78);
    deg2rad(7.225)];

    [z1,VA1] = FMT_with_weight(xi1,Xg1,samples1,s);
    setconfig(s,[0,0,0,0,0,0]);
    path1 = getpath(z1,VA1);
    path1 = pathsimplfy(path1,s);

    [z2,VA2] = FMT_with_weight(xi2,Xg2,samples2,s);
    setconfig(s,[0,0,0,0,0,0]);
    path2 = getpath(z2,VA2);
    path2 = pathsimplfy(path2,s);

    % merge paths
    path = [path1,path2(:,2:end)];
%---------------------------------Working----------------------------------%
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

sim.delete(); % call the destructor!

disp('Program ended');

function ret = get_cspace_lebes(s)
    ret = 1;
    for i = 1:size(s.joint_range,2)
        ret = ret*(s.joint_range{i}(2)-s.joint_range{i}(1));
    end
end