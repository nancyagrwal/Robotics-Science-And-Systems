% 
% This function takes two joint configurations and the parameters of the
% obstacle as input and calculates whether a collision free path exists
% between them.
% 
% input: q1, q2 -> start and end configuration, respectively. Both are 1x4
%                  vectors.
%        sphereCenter -> 3x1 position of center of sphere
%        r -> radius of sphere
%        rob -> SerialLink class that implements the robot
% output: collision -> binary number that denotes whether this
%                      configuration is in collision or not.

function collision = Q1(rob,q1,q2,sphereCenter,r)
	SEG_SIZE = 20;
    collision = false;
    [~ , s] = size(q1); 
    str = zeros(s , SEG_SIZE);
    %generating pts:    
    for j=1:s
        str(j , :) = linspace(q1(j), q2(j), SEG_SIZE);
    end
    
    str = str';
    [n , ~] = size(str);
    
    for j=1:n
        collision = robotCollision(rob, str(j , :),sphereCenter,r);
        % if collision becomes true, break:
        if(collision) 
            break; 
        end
    end
end

