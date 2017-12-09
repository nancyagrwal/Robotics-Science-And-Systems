function ComputePRM()
% Give a start and an end configuration: (Thet1 Thet2 Thet3 Thet4 Thet5 Thet6)
figure(1);
axis_size = 40;
axis equal;
axis (axis_size*[-1 1 -1 1]);

% PUMA 560 is a 6-link robot. 

% config1:
% start_config =[-120 120 -120 120 -120 60];
% goal_config = [0 0 0 0 0 180];

% config2:
% start_config = [0 0 0 -45 45 0];
% goal_config = [-35 -30 -25 20 20 -20];

% config3:
start_config = [240 120 240 120 240 60];
goal_config = [0 0 0 0 0 180];
 
start = CreatePumaRobot(start_config);
s_config = patch (start);
s_config.FaceColor = 'red';
s_config.EdgeColor = 'black';

goal = CreatePumaRobot(goal_config);
g_config = patch(goal);
g_config.FaceColor = 'green';
g_config.EdgeColor = 'black';

%% Adding 3 path obstacles
% any of the below configurations can be used:
% for experiments 1 and 2:
obstacle = createStructure(10, 20, 10, 20);
obstacle = appendStructures (obstacle, createStructure(-20, 0, -20, -10));
obstacle = appendStructures (obstacle, rotateStruct(createStructure(-10, 10, -10, 10), 30, [-20 20]));
% obstacle = appendStructures(obstacle, rotateStruct(createStructure(-18, 10, 13, 11), 40, [-19 , 19]));
% obstacle = appendStructures(obstacle, createStructure(-44, -35, -20, -49));
% obstacle = appendStructures(obstacle, rotateStruct(createStructure(-6, 6, -5, 4), 45, [15 -15]));
% for experiment 3 and 4:
% obstacle = createStructure(-20,20,40,5);
% obstacle = appendStructures(obstacle, createStructure(-20,20,-40,-5));
obst = patch(obstacle);
obst.FaceColor = 'blue';
obst.EdgeColor = 'black';

% Build a roadmap:
sample_count = 50;
k = 5;

% pass functions as arguments like: q1 = integral(@log,a,b)
tic
init_roadmap = PRM (@()(RandomSPaceConfiguration(obstacle)), @CalculateDistance, @(x,y)(LocalPlanner(x,y,obstacle)), sample_count, k);
toc

%for plotting roadmap:
init_roadmap.pts(sample_count+1,1) = start.vertices(1,1);
init_roadmap.pts(sample_count+1,2) = start.vertices(1,2);
init_roadmap.pts(sample_count+2,1) = start.vertices(1,1);
init_roadmap.pts(sample_count+2,2) = start.vertices(1,2);
drawnow
%disp(init_roadmap.pts);

% Add nodes
goal_roadmap = AddNewNode (start_config', init_roadmap, @CalculateDistance, @(x,y)(LocalPlanner(x,y,obstacle)), k);
goal_roadmap = AddNewNode (goal_config', goal_roadmap, @CalculateDistance, @(x,y)(LocalPlanner(x,y,obstacle)), k);

tic
adjacency_matrix = zeros(sample_count+2);
cost_matrix = zeros(sample_count+2);
sid = sample_count + 1;
fid = sample_count + 2;
for i= 1:length(goal_roadmap.edge_set)
   x = goal_roadmap.edge_set(i,1);
   y = goal_roadmap.edge_set(i,2);
   adjacency_matrix(x,y) = 1;
   adjacency_matrix(y,x) = 1;
   cost_matrix(x,y) = goal_roadmap.costs_edge(i);
   cost_matrix(y,x) = goal_roadmap.costs_edge(i);
end
[shortest_Path,cost] = dijkstra(adjacency_matrix,cost_matrix,sid,fid);
toc
% tic
% shorty = AStar(adjacency_matrix,cost_matrix,sid,fid);
% toc
% tic
% % Using dijkastra algorithm to plan a shortest_Path
% shortest_Path_old = ShortestPathDijkstra(goal_roadmap.edges, goal_roadmap.edge_lengths, sample_count+1, sample_count+2);
% toc
disp(shortest_Path);
figure(2);
gplot(adjacency_matrix,init_roadmap.pts,'r*:'); hold on;
plot(init_roadmap.pts(shortest_Path,1),init_roadmap.pts(shortest_Path,2),'ko-','LineWidth',3); hold off
title(sprintf('Distance from %d to %d = %1.3f',1,sample_count+2,cost))

for i = 2:length(shortest_Path)
    x1 = goal_roadmap.random_samples(:,shortest_Path(i-1));
    x2 = goal_roadmap.random_samples(:,shortest_Path(i));
    diff = x2 - x1;
    t = diff > 180;
    diff(t) = diff(t) - 360;
    t = diff < -180;
    diff(t) = diff(t) + 360;
    itr = ceil(sum(abs(diff))/10);
    
    for t = 0:itr
        x = mod(x1 + (t/itr)*diff, 360);
        rob = CreatePumaRobot(x);
        s_config.Vertices = rob.vertices;
        drawnow;
        if (CheckForCollision(rob, obstacle))
            fprintf (1, 'collision!!\n');
        end
       
    end
    
end
end
function out = CalculateDistance (config1, config2)
% Compute the distance between two configurations
%Metric by Comparing Angles:
%p(theta1, theta2) = min {|theta1 - theta2|, 2*pi - |theta1 - theta2|}	
angle_dif = abs(bsxfun(@minus, config2, config1));
out = sum(min(angle_dif,360-angle_dif));
end

function out = LocalPlanner (config1, config2, obstacle)
% Local planning checks whether there is a local path between two milestones, 
% which corresponds to an edge on the roadmap.
diff = config2 - config1;
t = diff > 180;
diff(t) = diff(t) - 360;
t = diff < -180;
diff(t) = diff(t) + 360;
sample_count = ceil(sum(abs(diff)) / 10);
for i = 1:sample_count
    config = mod(config1+ (i/sample_count)*diff, 360);
    rob = CreatePumaRobot(config);
    if (CheckForCollision(rob,obstacle))
        out = false;
        return
    end
end
out = true;
end


function conf = RandomSPaceConfiguration(obstacle)
while true
    conf = rand(6,1)*360;
    rob = CreatePumaRobot(conf);
    if (~CheckForCollision(rob, obstacle))
        return
    end
end
end


