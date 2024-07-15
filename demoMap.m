%TAO MAP (BINARY O-GRIDS)
myMap = binaryOccupancyMap(10,10,5) %scale map len 5 de phu hop size cua rb
%TAO MANG VOI KICH THUOC THICH HOP VOI MAP
walls = zeros(50,50);   %tao ma tran 50x50 voi gia tri 0
walls(1,:) = 1;         %vach tuong tren
walls(end,:) = 1;       %vach tuong duoi
walls(:,1) = 1;         %vach tuong trai
walls(:,end) = 1;       %vach tuong phai
walls(1:20,25) = 1;     %vach ngan 1
walls(20,20:25) = 1;    %vach ngan 2
walls(30,1:10) = 1;     %vach ngan 3
walls(30:end,22) = 1;   %vach ngan 4
walls(30,22:25) = 1;    %vach ngan 5
walls(30,40:end) = 1;   %vach ngan 6
walls(40,40:end) = 1;   %vach ngan 7
walls(42:50,14:22) = 1; %vach ngan 8
%DUA GIA TRI OCCUPANCY VAO MANG
setOccupancy(myMap,[1 1],walls,"grid")
%show(myMap)
%INFLATE MAP TO ACCOUNT FOR ROBOT SIZE
inflateRadius = 0.05 
inflate(myMap,inflateRadius)
show(myMap)
%CHECK OCCUPANCY IN WORLD COORDINATE (METERS)
%ccupied = checkOccupancy(myMap, [6 5])



