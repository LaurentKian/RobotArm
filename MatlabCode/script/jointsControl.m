clear
close all
clc
sim=remApi('remoteApi');
sim.simxFinish(-1); % just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID>-1)
    disp('Connected to remote API server');

%--------------------------set joint poses--------------------------------%
    joint_poses = zeros(3,7);
    joint_poses(1,:) = [-2*pi/3,0,0,0,pi/4,0,0];
    joint_poses(2,:) = [0,-pi/4,pi/2,0,0,0,0];
    joint_poses(3,:) = [-2*pi/3,0,0,0,pi/4,0,0];


%--------------------------get joints handles-----------------------------%
    h = zeros(1,7);

    % assign joints' name
    link_names = repelem("UR5_jointnum",1,7);
    for i = 1:7
        link_names(i) = replace(link_names(i),"num",string(i));
    end

    for i = 1:7
	    [r,h(i)] = sim.simxGetObjectHandle(clientID,char(link_names(i)),...
                sim.simx_opmode_blocking);
    end

% ------------------------- control the joints ---------------------------%
    while true
        for i = 1:3
            for j = 1:7
            	sim.simxSetJointTargetPosition(clientID,h(j),...
                    joint_poses(i,j),sim.simx_opmode_streaming);
            end
            pause(2)
        end
    end
else
    disp('Failed connecting to remote API server');
end
sim.delete(); % call the destructor!

disp('Program ended');