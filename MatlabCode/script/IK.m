clear
close all
clc
sim=remApi('remoteApi');
sim.simxFinish(-1); % just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);
load traj.mat % load trajectory
if (clientID>-1)
    disp('Connected to remote API server');
    [~,target_h] = sim.simxGetObjectHandle(clientID,'target',...
        sim.simx_opmode_blocking);
    
    % set interval
    dt = 0.025;

    % show the trajectory
    showtraj(traj)

    % set the position of the target
    for i = 1:size(traj,2)
        sim.simxSetObjectPosition(clientID,target_h,-1,traj(:,i),...
        sim.simx_opmode_oneshot);
        pause(dt)
    end
    

else
    disp('Failed connecting to remote API server');
end
sim.delete(); % call the destructor!

disp('Program ended');

function showtraj(traj)
% show the trajectory
aniline  =  animatedline("Color","r","LineWidth",3);
axis([-0.2,0.2,-0.2,0.2,0.6,1.0])
grid on
axis equal
view(-52,22)
for i = 1:size(traj,2)
    addpoints(aniline,traj(1,i),traj(2,i),traj(3,i));
    drawnow
end

end