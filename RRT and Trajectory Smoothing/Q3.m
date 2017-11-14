% Smooth path given in qMilestones
% input: qMilestones -> nx4 vector of n milestones. 
%        sphereCenter -> 3x1 position of center of spherical obstacle
%        sphereRadius -> radius of obstacle
% output -> qMilestones -> 4xm vector of milestones. A straight-line interpolated
%                    path through these milestones should result in a
%                    collision-free path. You should output a number of
%                    milestones m<=n.
function smoothedMileStone = Q3(rob,qMilestones,sphereCenter,sphereRadius)
    smoothedMileStone = qMilestones;
    [goalposit,~] = size(smoothedMileStone);
    radiusBuffer = sphereRadius * 1.2;
   
    while(true)
        goalPos = smoothedMileStone(goalposit,:);
        for pt=1:goalposit
            qnext = smoothedMileStone(pt,:);
            % Check for collision and move until free path:Keep moving
            % source
            collision = Q1(rob, qnext, goalPos, sphereCenter, radiusBuffer);
            if(collision) 
                continue; 
            end
            
            % if collision is false:
            smoothedMileStone(pt+1:goalposit-1,:) = [];
            goalposit = pt+1; 
            break;
        end
        
        % new goal position:
        goalposit = goalposit - 1;
        if(goalposit<3) 
            break; 
        end
    end
    smoothedMileStone
end
