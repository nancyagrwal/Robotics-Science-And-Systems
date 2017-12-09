function e = checkConvexHull(q1,q2)
% q1 = [240 120 240 120 240 60];
% q2 =  [0 0 0 0 0 90] ;
global map convexhull_map;

state_count = 20;
thet1diff = q1(1):((q2(1)-q1(1))/(state_count-1)):q2(1);
thet2diff = q1(2):((q2(2)-q1(2))/(state_count-1)):q2(2);
thet3diff = q1(3):((q2(3)-q1(3))/(state_count-1)):q2(3);
thet4diff = q1(4):((q2(4)-q1(4))/(state_count-1)):q2(4);
thet5diff = q1(5):((q2(5)-q1(5))/(state_count-1)):q2(5);
thet6diff = q1(6):((q2(6)-q1(6))/(state_count-1)):q2(6);

if size(thet1diff,2) < 20
   thet1diff = [thet1diff, q2(3)];
end

if size(thet2diff,2) < 20
    thet2diff = [thet2diff, q2(4)];
end

if size(thet3diff,2) < 20
      thet3diff = [thet3diff, q2(4)];
end

if size(thet4diff,2) < 20
    thet4diff = [thet4diff, q2(4)];
end

if size(thet5diff,2) < 20
    thet5diff = [thet5diff, q2(4)];
end

if size(thet6diff,2) < 20
     thet6diff = [thet6diff, q2(4)];
end

convexhull_map = [];
nodes = zeros(2*state_count,2);
start_index = 1;
end_index = 2*state_count;
   
for i = 1:state_count
     q1 = [thet1diff(i),thet2diff(i),thet3diff(i),thet4diff(i),thet5diff(i),thet6diff(i)];
     rob = CreatePumaRobot(q1);
%      for plotting states
%      p = patch(rob);
%      p.FaceColor = 'red';
%      p.EdgeColor = 'black';
     
     f= rob.vertices;
     pt1 = [(f(1,1) + f(2,1))/2,(f(1,2) + f(2,2))/2];
     pt2 = [(f(23,1) + f(24,1))/2,(f(23,2) + f(24,2))/2];
     
     nodes(start_index,:) = pt1;
     nodes(end_index,:) = pt2;
     start_index = start_index + 1;
     end_index = end_index - 1;
     
end
     [x1,y1] = poly2cw(nodes(:,1),nodes(:,2));
     convexhull_map = [x1,y1];
     plot(x1,y1,'.r');
     H = convhull(x1,y1);
     plot(x1(H),y1(H),'-')
      
% now check for convex hull collision
 e = 1;
 map_size = size(map,2);
 
 for i = 1:map_size-1
     ostacle = map{i};
     [x1,y1] = poly2cw(ostacle(:,1),ostacle(:,2));
     ostacle = [x1,y1];
     [x1,~] = polybool('intersection',convexhull_map(:,1),convexhull_map(:,2),ostacle(:,1),ostacle(:,2));
     
     if isempty(x1)
         e = e*1;
     else
         e = e*0;
     end
     
end



