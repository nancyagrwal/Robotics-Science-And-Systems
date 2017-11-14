% TODO: You write this function!
% input: f1 -> an 9-joint robot encoded as a SerialLink class for one
%              finger
%        f2 -> an 9-joint robot encoded as a SerialLink class for one
%              finger
%        qInit -> 1x11 vector denoting current joint configuration.
%                 First six joints are the arm joints. Joints 8,9 are
%                 finger joints for f1. Joints 10,11 are finger joints
%                 for f2.
%        f1Target, f2Target -> 3x1 vectors denoting the target positions
%                              each of the two fingers.
% output: q -> 1x11 vector of joint angles that cause the fingers to
%              reach the desired positions simultaneously.
%              (orientation is to be ignored)

function q = Q3(f1,f2,qInit,f1Target,f2Target)
% Define scaling matrix and rotation matrix:
    R = [1 0 0; 0 1 0; 0 0 1];
    a = 0.05;
    EMPTY = [0 0;0 0;0 0;0 0;0 0;0 0];
    q = qInit;
	
    % Finding goal positions for fingers F1 and F2:
    gpF1 = [R f1Target; 0 0 0 1];
    gpF2 = [R f2Target; 0 0 0 1];
    
	% Algorithm:
    for i=1:50
        % Find current joint angles for fingers F1 and F2:
        q1 = [q(:,1:7) q(:,8:9)];  
        q2 = [q(:,1:7) q(:,10:11)];  
        % Calculate current positions 
        T1 = f1.fkine(q1);
        T2 = f2.fkine(q2);
        % Calculating Jacobian in world frame:
        J1 = f1.jacob0(q1);              
		J2 = f2.jacob0(q2);   
		%Filling Jacobians for 12*11 matrix:	
        J1 = [J1(:,1:7) J1(:,8:9) EMPTY]; 
        J2 = [J2(:,1:7) EMPTY J2(:,8:9)];  
        FinalJ = [J1;J2];                  
        % Calculating Jacobian inverse:
        Jinv = pinv(FinalJ);                     
		% Calculate current and final delta 
        deltaF1 = tr2delta(T1,gpF1);    
        deltaF2 = tr2delta(T2,gpF2);     
        deltaX   = [deltaF1; deltaF2];           
        % Calculate vector of joint angles:
        deltaQ = a *  Jinv * deltaX;
        q = deltaQ' + q;
    end
end
    
