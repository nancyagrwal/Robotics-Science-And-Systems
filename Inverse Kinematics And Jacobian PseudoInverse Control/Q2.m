% TODO: You write this function!
% input: f -> a 9-joint robot encoded as a SerialLink class
%        qInit -> 1x9 vector denoting current joint configuration
%        posGoal -> 3x1 vector denoting the target position to move to
% output: q -> 1x9 vector of joint angles that cause the end
%                     effector position to reach <position>
%                     (orientation is to be ignored)
function q = Q2(f,qInit,posGoal)

	% Define scaling factor and rotation matrix:
    R = [1 0 0; 0 -1 0 ; 0 0 1];
    a = 0.5;
    % Find goal position:
    q = qInit;
    T = [R posGoal; 0 0 0 1];
    
	% Algorithm:
    for i=1:50
        pos = f.fkine(q);
		% Finding jacobian in world frame:
		J = f.jacob0(q);
		% Calculating jacobian inverse:
		Jinv = pinv(J);
        % Converting homogenous transform dX = (posGoal - p); to differential motion: 
		deltaX = tr2delta(pos,T);
        % Calculate vector of joint angles:
        deltaQ = a * Jinv * deltaX;
        q = deltaQ' + q;
    end

end


