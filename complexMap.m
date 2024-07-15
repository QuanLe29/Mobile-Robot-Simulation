%% map phuc tap
compMap = binaryOccupancyMap(10,8,5); 
%% TAO MANG VOI KICH THUOC THICH HOP VOI MAP
walls = zeros(50,40);   %tao ma tran 50x40 voi gia tri 0
walls(:,1) = 1 ;        %vach tuong trai
walls(:,50) = 1;        %vach tuong phai
walls(1,:) = 1;         %vach tuong tren
walls(40,11:50) = 1;    %vach tuong duoi 
walls(1:20,26) = 1;     %vach tuong 1
walls(17,1:18) = 1;     %vach tuong 2
walls(1:5,1:15) = 1;    %ban hoc 
walls(9:10,7:8) = 1;    %ghe ngoi
walls(26,6) = 1;        %vat can ngay cua vao
walls(7,26:38) = 1;     %vach tuong 3
walls(13,34:50) = 1;    %vach tuong 4
walls(20,26:34) = 1;    %vach tuong 5
walls(28:40,11) = 1;    %vach tuong 6
walls(28,11:36) = 1;    %vach tuong 7
walls(28:34,36) = 1;    %vach tuong 8
walls(33:40,18) = 1;    %vach tuong 9
walls(33:40,26) = 1;    %vach tuong 10
walls(32,42) = 1;       %vat can goc phai ben duoi
walls(22,20) = 1;       %vat can gan tuong 1
%DUA GIA TRI OCCUPANCY VAO MANG
setOccupancy(compMap,[1 1],walls,"grid")
show(compMap)

