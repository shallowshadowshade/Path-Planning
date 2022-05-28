 function map = obstacle_map(xStart,yStart,xTarget,yTarget,MAX_X,MAX_Y)%建障碍物函数
%This function returns a map contains random distribution obstacles.
    rand_map = rand(MAX_X,MAX_Y);%返回一个MAX_X * MAX_Y的矩阵
    map = [];
    % 第一个障碍物为起点
    map(1,1) = xStart; %第一个障碍物的坐标
    map(1,2) = yStart; %第二个障碍物的坐标
    k=2;
    % 地图中的障碍物比例
    obstacle_ratio = 0.45;
    for i = 1:1:MAX_X
        for j = 1:1:MAX_Y
            if( (rand_map(i,j) < obstacle_ratio) && (i~= xStart || j~=yStart) && (i~= xTarget || j~=yTarget))
                map(k,1) = i; %假如比例足百分之45的话，就继续扩张障碍物，直到满足百分之45的障碍物比例为止
                map(k,2) = j;
                k=k+1;
            end    
        end
    end
    % 最后一个障碍物为终点
    map(k,1) = xTarget; %第二个标识表示第几个障碍物
    map(k,2) = yTarget;
 end