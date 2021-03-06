% **************************************************************
% ************** You don't need to modify this function.
% **************************************************************
function [xmat,umat] = Q3(A,B,QT,Q,R,Rdelta,T,x0)

    % calculate new synthetic dynamics for the system given underlying
    % dynamics.
    [Abar,Bbar,QTbar,Qbar,x0bar] = getSyntheticDynamics(A,B,QT,Q,R,x0);
    
    % calculate value function
    Pseq = FH_DT_Riccati(Abar,Bbar,QTbar,Qbar,Rdelta,T);

    % integrate forward the vehicle dynamics
    [xmat,umat] = integrate_dynamics(Abar,Bbar,Rdelta,x0bar,Pseq,T);

end


% **************************************************************
% ************** You don't need to modify this function.
% **************************************************************
% Get control action and integrate dynamics foward one step in time. This
% function calls <getControl>, a function that you must implement. 
% input: A,B,R -> parameters of system and cost function
%        x0 -> 4x1 initial state
%        Pseq -> cell array of 4x4 P matrices. Cell array goes from 1 to T
%        T -> time horizon
% output: xmat -> 4xT matrix of state as a function of time
%         umat -> 2x(T-1) matrix of control actions as a function of time
function [xmat, umat] = integrate_dynamics(A,B,R,x0,Pseq,T)
    i=0;
    x = x0;
    for i=1:T
        
        % get control action
        u = getControl(A,B,R,Pseq,x,i);

        % integrate dynamics forward one time step
        x = A*x + B*u;
        
        xmat(:,i) = x;
        umat(:,i) = u;

    end
end

% output: Abar,Bbar,QTbar,Qbar,x0bar -> net parameters
function [Abar,Bbar,QTbar,Qbar,x0bar] = getSyntheticDynamics(A,B,QT,Q,R,x0)
    Bbar = [B;R];
    %disp(Bbar);
    Abar  = [[A;zeros(size(R,1),size(A,2))] Bbar];
    Qbar  = [Q zeros(size(Q,1),size(R,2)); zeros(size(R,1), size(Q,2)) R];
    QTbar = [QT zeros(size(Q,1),size(R,2));zeros(size(R,1),size(Q,2)+size(R,2))];
    x0bar = [x0;zeros(size(R,2),1)];
   
end

% Calculate control action to take from the current state.
% input: A,B,R -> parameters of system and cost function
%        P -> P matrix for this time step
%        x -> current state
%        i -> current time step of controller
% output: u -> control action to take
function u = getControl(A,B,R,Pseq,x,i)
    % same as before:
    % at current time step:
    N1 = Pseq{i};
    u = (-(R+B'*N1*B)^-1)*B'*N1*A*x;
end


% Calculate time-varying value function using riccati eqn. (i.e.
% compute the sequence of P matrices.)
% input: A,B,Qf,Q,R -> parameters of dynamics and cost function
%        T -> time horizon
% output: Pseq -> cell array of 4x4 P matrices. Cell array goes from 1 to T
function Pseq = FH_DT_Riccati(A,B,Qf,Q,R,T)
    % same as before:
    % define Pseq as a cell array:
    Pseq = cell(1,T);
    % define final cost:
    Pseq{T} = Qf;
    for t=(T-1):-1:1
        N1 = Pseq{t+1};
        temp = A'*N1*B*((R+B'*N1*B)^-1)*B'*N1*A;
        Pseq{t} = Q+A'*N1*A-temp;
    end
end


