function [] = lynxServo(th1, th2, th3, th4, th5, th6)
% Commands the Lynx to the angles defined by the input (in radians)
% Has a change in joint angle limit of 5 degrees between successive calls
% The function cannot be called successively without a gap of 100 ms
%
% INPUTS:
%   th1...th6 : Joint variables for the six DOF Lynx arm (radians and
%   mm)
% OUTPUT:
%   Calls lynxServoSim.m and displays the Lynx in the configuration specified by 
%   the input parameters, or calls lynxServoPhysical.m to move the real
%   robot.

    % Ensure the Lynx has been initialized
    if(~evalin('base','exist(''lynx'',''var'')'))
        error('lynxServo:lynxInitialization','Lynx has not been initialised. Run lynxStart first');
    end
    
    % Import global variables
    global lynx qs time_prev time_prev_vel delay_ms
    
    if isempty(time_prev)
        time_prev = tic;
    end
    if isempty(time_prev_vel)
        time_prev_vel = tic;
    end
    
    % Check inputs
    if nargin == 6 && length(th1) == 1 
        q = [th1 th2 th3 th4 th5 th6];
    elseif nargin == 1 && length(th1) == 6 && numel(th1) == 6
        q = th1;
        % Ensure that q is a row vector
        if size(q, 1) > 1
            q = q';
        end
    else
        error('Input must be 6 scalars or a 6 element vector representing desired joint angles followed by the robot name.')
    end
    
    if any(isnan(q))
        error('lynxServo:nanInput',['Input contains a value of NaN. Input was: [' num2str(q) ']']);
    end
    
    % Set delay between function calls
    if isempty(delay_ms)
        if lynx.hardware
            delay_ms = 10;
        else
            delay_ms = 0;
        end
    end
    delay = delay_ms/1000;
    
    % Check that angular limits are satisfied
    lowerLim = [-1.4 -1.2 -1.8 -1.9 -2 -15]; % Lower joint limits in radians (grip in inches)
    upperLim = [1.4 1.4 1.7 1.7 1.5 30]; % Upper joint limits in radians (grip in inches)
    
    for i=1:length(q)
        if q(i) < lowerLim(i)
            q(i) = lowerLim(i);
            fprintf('Joint %d was sent below lower limit, moved to boundary %0.2f\n',i,lowerLim(i))
        elseif q(i) > upperLim(i)
            q(i) = upperLim(i);
            fprintf('Joint %d was sent above upper limit, moved to boundary %0.2f\n',i,upperLim(i))
        end
    end

    %% Collision Detection

    % Preventing forearm from hitting base of robot
    if (q(3) > (-0.135 * q(2) + 1.15))
         q(3) = (-0.135 * q(2)) + 1.15;
         %disp('Position would have caused collision, moved to closest safe position')
    end
			  
    % Check that angular velocity limits are satisfied
    dt = toc(time_prev_vel); %time since velocity limit was last checked
    time_prev_vel = tic;

    maxOmegas = [1 1 1 2 3 20];
    
    if dt > 0
        omega = abs((q - qs)/dt);
        speedind = find(omega>maxOmegas);	
        if ~isempty(speedind)
            jointErrStr = [];
            for i = 1:length(speedind)
                jointErrStr = [jointErrStr sprintf('\nJoint %d has approached an angular velocity of %.3f rad/s. The max angular velocity for joint %d is %0.3f rad/s', speedind(i), omega(speedind(i)),speedind(i),maxOmegas(speedind(i)))];
            end
           error('lynxServo:VelocityLimit','%s',jointErrStr);
        end
    end
    
    qs = q;

    % Send the angles to the Lynx
    if lynx.hardware
        
        lynxServoPhysical(q(1),q(2),q(3),q(4),q(5),q(6));
        
    else
        lynxServoSim(q);
    end
    
    while 1
        if toc(time_prev) > delay
            time_prev = tic;
            break;
        end
    end

end
