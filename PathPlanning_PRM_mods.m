%% khoi tao mobile robot
R = 0.1; % ban kinh banh xe [dv: m]
L = 0.5; % chieu dai co so  [dv: m]
dd =  DifferentialDrive(R,L); % dinh nghia xe
%[v w] = forwardKinematics(dd,wL,wR); % tinh v & w dua vao wL & wR
%[wL,wR] = inverseKinematics(dd,v,w); % tinh wL & wR dua vao v & w
d = 0; %quang duong di duoc 
%% cac tham so mo phong
SampleTime = 0.1; % khoang thoi gian lay mau [dv: s]
tVec = 0:SampleTime:30; % vecto thoi gian, 0->30 chia lam 0.1(SampleTime)
                        % phan bang nhau
initPose = [1;0;0]; % vi tri ban dau (x0,y0& theta0)
pose = zeros(3,length(tVec)); % ma tran chua cac vi tri ung voi cac gia tri thoi gian cua tVec
pose(:,1) = initPose; % gan vi tri ban dau vao ma tran pose

%% tao duong di (Create possible pathway)
close all
load complexMap
mapInflated = copy(compMap);
inflate(mapInflated,R);
%inflate(compMap,R); % thoi phong tung vi tri bi chiem dong
% tao duong di roadmap bang PRM
planner = mobileRobotPRM(mapInflated);
planner.NumNodes = 20;
planner.ConnectionDistance = 1.0; %[dv: m];
% ve duong di dua vao diem dau va diem cuoi
startPoint = [initPose(1),initPose(2)];
goalPoint  = [4.2, 1.0];
waypoints = findpath(planner,startPoint,goalPoint);
% phan tim so nut toi thieu de tim duong
while isempty(waypoints)
        planner.NumNodes = planner.NumNodes +10;
        update(planner);
        waypoints = findpath(planner,startPoint,goalPoint);
end
show(planner)
%% Pure Pursuit Controller
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.56; %khoang cach robot nhin de tinh toan
                                     %dieu khien [dv: m]
controller.DesiredLinearVelocity = 0.75;    % [dv: m/s]
controller.MaxAngularVelocity = 1.5;        % [dv: rad/s]
%% Create visualizer 
load complexMap % load map chua inflate
%inflate=0;
viz = Visualizer2D;
viz.hasWaypoints = true;
viz.mapName = 'compMap';
%% Chay lap phan mo phong
r = rateControl(1/SampleTime); %khoang thuc hien cac code de control robot
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
            disp('DistanceTravelled:  (m)')
            d  
            disp('time:  (s)') 
            t = tVec(i-1)
            disp('Node Needed: ')
            planner.NumNodes
            break;
    end
    % cap nhat hien thi visualizer
    viz(pose(:,i),waypoints)
    waitfor(r); 
end