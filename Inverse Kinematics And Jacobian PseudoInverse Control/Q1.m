% TODO: You write this function!
% input: f -> an 9-joint robot encoded as a SerialLink class
%        position -> 3x1 vector denoting the target position to move to
% output: q -> 1x9 vector of joint angles that cause the end
%              effector position to reach <position>
%              (orientation is to be ignored)
function q = Q1(f,position)
	% Define the rotation matrix:
    R = [1 0 0;0 1 0; 0 0 1];
    % Find goal position:
    T = [R position;0 0 0 1];
    % Calculate vector of joint angles:
    q = f.ikine(T);
end
