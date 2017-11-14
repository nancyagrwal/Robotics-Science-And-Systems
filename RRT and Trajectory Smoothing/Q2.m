% Calculate a path from qStart to xGoal
% input: qStart -> 1x4 joint vector describing starting configuration of
%                   arm
%        xGoal -> 3x1 position describing desired position of end effector
%        sphereCenter -> 3x1 position of center of spherical obstacle
%        sphereRadius -> radius of obstacle
% output -> qMilestones -> 4xn vector of milestones. A straight-line interpolated
%                    path through these milestones should result in a
%                    collision-free path. You may output any number of
%                    milestones. The first milestone should be qStart. The
%                    last milestone should place the end effector at xGoal.
function qMilestones = Q2(rob,sphereCenter,sphereRadius,qStart,xGoal)
	
    step_size = 0.1;      
    error  = 0.01;    
    plotGraph = true;
    qMilestones=[];
    
    % Add buffer space
    radiusBuffer = sphereRadius * 1.2;
    
    % create a tree:
    tree = [qStart] ; 
    nodeParents = [-1];
    
    % Calculate goal positions:
    qGoal = rob.ikine(transl(xGoal),zeros(1,4),[1,1,1,0,0,0]);
    % plot filled points for goal config: 
    scatter3(qGoal(1), qGoal(2),qGoal(3), 'filled');
    
    while(true)
        collision = true;
        % Generate a random node:
        qrand = generateRandomGoal(qGoal);
        
        % Generate the nearest node to random node in tree:
        qnear = findClosestNode(tree , qrand);    
        
        %qNext = new Node EPSILON away from qNear in direction of qRandom;
        qnext = tree(qnear,:);
        
        if(pdist([qnext;qrand])<error) 
            continue;
        end
        
        % check if there is a collision:
        if(qrand == qGoal)
            collision = Q1(rob, qnext, qGoal, sphereCenter, radiusBuffer);
        end
        
        % Keep on moving towards goal if distance > step_size
        if(collision && pdist([qnext;qrand]) > step_size)
            segment = max(40 , ceil(pdist([qnext;qrand])/step_size));
            [~ , k] = size(qnext); 
            qgen = zeros(k , segment);
            for i=1:k
                qgen(i,:) = linspace(qnext(i), qrand(i), segment);
            end
            qrand = qgen(:,2);
            qrand = qrand';
        end
        
        % check for collision with updated qrand
        collision = Q1(rob, qnext , qrand, sphereCenter, radiusBuffer);
        if(collision) 
            continue; 
        end
        
        %update tree and parent:
        tree = [tree; qrand];
        nodeParents = [nodeParents; qnear];
        
        if(plotGraph)
            newpoint = rob.fkine(qrand);
            newpoint = newpoint(1:3,4);
            scatter3(newpoint(1), newpoint(2), newpoint(3));
        end
        
        % if we reach goal , then break:
        if(qGoal == qrand) 
            break; 
        end
    end
    
    % add nodes to qMilestones=[]:
    [qnear , ~] = size(tree);
    while(qnear ~= -1)
        qMilestones = [tree(qnear , : ) ; qMilestones];
        qnear = nodeParents(qnear);
    end
            
    % plotting as in file hw2:
    if(plotGraph)
        for b = 1:size(qMilestones)
            newplot = rob.fkine(qMilestones(b,:));
            newplot = newplot(1:3,4);
            scatter3(newplot(1), newplot(2), newplot(3), '+');
        end
    end
    qMilestones
end

function qnear = findClosestNode(node1 , node2)
    [s , ~] = size(node1);
    Y = repmat(node2, s, 1);
    distance  = node1 - Y;
    sme = sum(distance.^2,2);
    distance  = sqrt(sme);
    [~ , qnear] = min(distance);
end

function randomNode = generateRandomGoal(qGoal)
    p = rand;
    R = rand(size(qGoal)); 
    if(p < 0.8) 
        randomNode = mod(R*15, 4*pi);
        randomNode = randomNode - 2*pi;
    else 
        randomNode = qGoal;
    end
end




