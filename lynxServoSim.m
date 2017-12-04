function [ ] = lynxServoSim( th1, th2, th3, th4, th5, th6)
% lynxMove  Moves the simulated Lynx to the new configuration specified by the input (in radians)

%   INPUTS:
%    th1...th6 : Joint variables for the six DOF Lynx arm in radians
    
    % Ensure that the Lynx has been initialized
    if(~evalin('base','exist(''lynx'',''var'')'))
        error('lynxMove:lynxInitialization','Lynx has not been initialised. Run lynxStart first');
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
        error('lynxMove:argumentCheck','Input must be 6 scalars or a 6 element vector representing desired joint angles.')
    end
    
    
    if any(isnan(q))
        error('lynxMove:nanInput',['Input contains a value of NaN. Input was: [' num2str(q) ']']);
    end
    
    % Changes the Lynx config to that specified by input and plots it there
    global lynx frameSkip frameCounter  
    skipFrame = false;
    if ~isempty(frameSkip) && ~isempty(frameCounter)
        if mod(frameCounter, frameSkip) == 0
            skipFrame = false;
        else
            skipFrame = true;
        end
        frameCounter = frameCounter + 1;
    end
    
    
    plotrobot(lynx, q, skipFrame);
       
end

