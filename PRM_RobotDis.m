%% khoi tao mobile robot
R = 0.1; % ban kinh banh xe [dv: m] wheelRadius
L = 0.3; % chieu dai co so  [dv: m] trackwidth = wheelbase
dd =  DifferentialDrive(R,L); % dinh nghia xe
%[v w] = forwardKinematics(dd,wL,wR); % tinh v & w dua vao wL & wR
%[wL,wR] = inverseKinematics(dd,v,w); % tinh wL & wR dua vao v & w
d = 0; %quang duong di duoc 
%% cac tham so mo phong (Simulation parameters)
SampleTime = 0.1; % khoang thoi gian lay mau [dv: s ?]
tVec = 0:SampleTime:30; % vecto thoi gian, 0->30 chia lam 0.1(SampleTime)
                        % phan bang nhau
initPose = [1;0;0]; % vi tri ban dau (x0,y0& theta0)
pose = zeros(3,length(tVec)); % ma tran chua cac vi tri ung voi cac gia tri thoi gian cua tVec
pose(:,1) = initPose; % gan vi tri ban dau vao ma tran pose
robotCurrentPose = initPose;
robot = differentialDriveKinematics("TrackWidth", 0.3, "VehicleInputs", "VehicleSpeedHeadingRate");
%% tao duong di (create possible pathway)
close all
load complexMap
mapInflated = copy(compMap);
inflate(mapInflated, robot.TrackWidth/2);
%inflate(compMap,R); % thoi phong tung vi tri bi chiem dong
% tao duong di roadmap bang PRM
planner = mobileRobotPRM(mapInflated);
planner.NumNodes = 450;
planner.ConnectionDistance = 1; %[dv: m];
% ve duong di dua vao diem dau va diem cuoi
startPoint = [initPose(1),initPose(2)];
goalPoint  = [3.0, 1.0];
waypoints = findpath(planner,startPoint,goalPoint);
%{
% phan tim so nut toi thieu de tim duong
while isempty(waypoints)
        planner.NumNodes = planner.NumNodes +10;
        update(planner);
        waypoints = findpath(planner,startPoint,goalPoint);
end
%}
show(planner)
%% Pure Pursuit Controller
controller = controllerPurePursuit;
release(controller); %de dung lai controller ma khong can clear het data
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.8; %khoang cach robot nhin de tinh toan
                                     %dieu khien [dv: m]
controller.DesiredLinearVelocity = 0.75;    % [dv: m/s]
controller.MaxAngularVelocity = 1.5;        % [dv: rad/s]
%% Create visualizer 
load complexMap % load map chua inflate
%inflate=0;
viz = Visualizer2D;
viz.hasWaypoints = true;
viz.mapName = 'compMap';
%{
%% Chay lap phan mo phong
r = rateControl(1/SampleTime); %khoang thuc hien cac code de control robot
frameSize = robot.TrackWidth/0.8;
for i = 2:length(tVec) 
    % Chuyen doi tu controller sang toc do quay 2 banh xe
    [vRef,wRef] = controller(pose(:,i-1));
    [wL,wR] = inverseKinematics(dd,vRef,wRef);
    
    % Tinh van toc
    [v,w] = forwardKinematics(dd,wL,wR); %phuong trinh dong hoc robot
    vel = [v*cos(pose(3,i-1));v*sin(pose(3,i-1));w];
    pose(:,i) = pose(:,i-1) + vel*SampleTime; % vi tri tiep theo 
    % Xac dinh vi tri hien tai
    CurrPose = [pose(1,i-1);pose(2,i-1)];
    dist = norm(goalPoint'-CurrPose);  %Khoang cach tu diem cuoi den diem
                                       %hien tai cua robot
    if i>2
       d = d +norm(pose(1:2,i-1)-pose(1:2,i-2)); 
    end
    if (dist < .2)       %[m]             
            disp("Goal position reached")
            disp('DistanceTravelled: ')
            d  
            disp('time: ') 
            t = tVec(i-1)
            disp('Node Needed: ')
            planner.NumNodes
            break;
    end
    % cap nhat hien thi visualizer
    viz(pose(:,i),waypoints)
    waitfor(r); 
end
%}
%% PRM WITH ROBOT DISPLAY
robotInitialLocation = waypoints(1,:);
robotGoal = waypoints(end,:);
initialOrientation = 0; % gia su goc xoay theta ban dau la 0
%vi tri cua robot [x,y,theta] theo ma tran dang 3 1
robotCurrentPose = [robotInitialLocation initialOrientation]';
%khoang cach tu robot den diem dich
dist = norm(robotInitialLocation - robotGoal);
%cho truoc khoang ve dich
goalRadius = 0.2;
%% CHAY LAP MO PHONG
r = rateControl(1/SampleTime);
% Determine vehicle frame size to most closely represent vehicle with plotTransforms
frameSize = robot.TrackWidth;
reset(r);
figure
while( dist > goalRadius )
    
    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = controller(robotCurrentPose);
    
    % Get the robot's velocity using controller inputs
    vel = derivative(robot, robotCurrentPose, [v omega]);
    
    % Update the current pose
    robotCurrentPose = robotCurrentPose + vel*SampleTime; 
    
    % Re-compute the distance to the goal
    dist = norm(robotCurrentPose(1:2) - robotGoal(:));
    
    % Update the plot
    hold off
    show(compMap);
    hold all

    % Plot path each instance so that it stays persistent while robot mesh
    % moves
    %plot(waypoints(:,1), waypoints(:,2),"k--d")
    plot(waypoints(:,1), waypoints(:,2))
    
    % Plot the path of the robot as a set of transforms
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, 'MeshFilePath', 'groundvehicle.stl', 'Parent', gca, "View","2D", "FrameSize", frameSize);
    light;
    xlim([0 10])
    ylim([0 8])
    
    waitfor(r);
end
if (dist < goalRadius)
    disp("Goal position reached")
end

