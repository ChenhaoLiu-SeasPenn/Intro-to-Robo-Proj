function lynxStart(varargin)
% lynxStart  Loads kinematic data for a Lynx AL5D manipulator and sets
%   variables.
%
%   If using hardware:
%   Starts the Lynx and moves the Lynx to its home position. 
%
%   WARNING: Be aware of the Lynx's position before turning on the 
%   controller. To prevent damage, remove any objects in the path between 
%   the current position and the home position. If the Lynx is likely to 
%   slam into the table surface or other immovable object, manually move 
%   the Lynx towards the home position before using this command.
%
%     Options         Values {Default Value}
%       'Hardware'      'Legend' | 'Lucky' | 'Lyric' | {'off'} Sets up the code to use the Lynx
%                       hardware.  If hardware is enabled then there
%                       is no visualization.  Default is simulation.
%                       Be careful to read the above warning before 
%                       enabling hardware.
%
%       'Joints'        {'on'} | 'off' Display the joints of the robot
%
%       'Jointaxis'     'on' | {'off'} Display the joint axes of the robot
%
%       'Shadow'        {'on'} | 'off' Plot the shadow of the robot on the
%                       ground plane
%       
%       'PlotEveryNFrames'  {1} Plots every N positions of the robot in
%                       simulation, speeding up visualizations
%
%       'Delay_ms'      {0 sim, 10 hardware} Delay, is milliseconds,
%                       between calls to lynxServo.  
%
%       'LEDMarkerStyle' {'*'} Style of the LED marker in the animation
%
%       'LEDMarkerSize' {6} Size of the point plotted for the LED
%
%   Philip Dames & Arunkumar Byravan & Mike Shomin
%   University of Pennsylvania
%   based on puma260.m

% Lynx ADL5 constants in mm
a = 3*25.4; % base height
b = 5.75*25.4; % shoulder to elbow length
c = 7.375*25.4; %elbow to wrist length
d = 1.75*25.4; %wrist1 to wrist2
e = 1.25*25.4; %wrist2 to base of gripper
f = 1.125*25.4; % gripper length

off = [ 0 b c 0 0 0]; % link length
D = [ a 0 0 0 d+e 0]; % link offset
alph = [ -pi/2 0 0 -pi/2 0 0] ; % link twist
th = [ 0 -pi/2 +pi/2 -pi/2 0 0]; % joint angle

% Initialize the robot links
%             alpha / a / theta / d / 0
L{1} = link([ alph(1) off(1) th(1) D(1) 0 ]);
L{2} = link([ alph(2) off(2) th(2) D(2) 0 ]); %, 'ActuatorDisplayOffset', [0, 0, -b/2]');
L{3} = link([ alph(3) off(3) th(3) D(3) 0 ]); %, 'ActuatorDisplayOffset', [0, 0, -d/2]');
L{4} = link([ alph(4) off(4) th(4) D(4) 0 ]);%, 'ActuatorDisplayOffset', [d, 0, 0]);
L{5} = link([ alph(5) off(5) th(5) D(5) 0 ]);%, 'ActuatorDisplayOffset', [0, 0, d]);
L{6} = link([ alph(6) off(6) th(6) D(6) 0 ]);

global lynx

% Set time delay
global delay_ms frameSkip frameCounter robotName qs
delay_ms = 10;
frameSkip = 1;
frameCounter = 0;

% zero angles, L shaped pose
qs = [0,0,0,0,0,0];

% Check property inputs
if mod(size(varargin,2), 2) == 1
    error('Must be a value for each property set')
end
optsin = {};
tool_on = false;
hardware_on = false;
for j = 1:2:size(varargin,2)
    % User specified time delay
    if strcmpi(varargin{1,j}, 'delay') || strcmpi(varargin{1,j}, 'delay_ms')
        delay_ms = varargin{1,j+1};
    
    % Use the Lynx hardware or simulation
    elseif strcmpi(varargin{1,j}, 'Hardware')
        if strcmpi(varargin{1,j+1}, 'off')
            optsin{end+1} = varargin{1,j};
            optsin{end+1} = varargin{1,j+1};
            
        elseif strcmpi(varargin{1,j+1}, 'Legend') || ...
                strcmpi(varargin{1,j+1}, 'Lucky') || ...
                strcmpi(varargin{1,j+1}, 'Lyric')
                optsin{end+1} = varargin{1,j};
                optsin{end+1} = 'on';
                robotName = varargin{1,j+1};
				hardware_on = true;
		else
			error('Invalid value for Hardware property');
        end
        
    elseif strcmpi(varargin{1,j}, 'PlotEveryNFrames')
        if mod(varargin{1,j+1}, 1) ~= 0
            error('PlotEveryNFrames must be an integer')
        end
        frameSkip = varargin{1,j+1};
        frameCounter = 0;
            
    elseif strcmpi(varargin{1,j}, 'Port')
        serialPort = varargin{1,j+1};
        
    % Else pass to robot initialization
    else
        optsin{end+1} = varargin{1,j};
        optsin{end+1} = varargin{1,j+1};
        
    end
    
    
end

% Initialize the Lynx variable
lynx = robot(L, optsin{:});
lynx.name = 'Lynx AL5D';
lynx.manuf = 'Lynxmotion';

% Initialize the Lynx if using hardware
if hardware_on
    lynx.hardware = hardware_on;
    % Check that delay is set properly
    if delay_ms < 0.5
        error('Delay cannot be less than 0.5 ms');
    end
    % Display warning message
    str = input(['Warning: Be aware of the Lynx''s position before continuing.\n'...
        'Hold the lynx as close to the home position as possible.\n' ...
        'Do you want to continue (y/n)?\n'],'s');
    if isempty(str)
        str = 'n';
    end
    if strcmpi(str, 'y') || strcmpi(str, 'yes')
        lynxInitializeHardware(serialPort);
    else
        disp('Start cancelled.');
    end
    
% Initialize the plot if using simulation
else
    % zero angles, L shaped pose
    plotrobot(lynx, qs);
    
end

% Set global variables in the base workspace
evalin('base', 'global lynx delay_ms qs')

%Send the robot to a home configuration
lynxServo(0,0,0,0,0,0);
