%PLOT Graphical robot animation
%
%	PLOT(ROBOT, Q)
%	PLOT(ROBOT, Q, options)
%
% Displays a graphical animation of a robot based on the
% kinematic model.  A stick figure polyline joins the origins of
% the link coordinate frames.
% The robot is displayed at the joint angle Q, or if a matrix it is
% animated as the robot moves along the trajectory.
%
% The graphical robot object holds a copy of the robot object and
% the graphical element is tagged with the robot's name (.name method).
% This state also holds the last joint configuration which can be retrieved,
% see PLOT(robot) below.
%
% If no robot of this name is currently displayed then a robot will
% be drawn in the current figure.  If hold is enabled (hold on) then the
% robot will be added to the current figure.
%
% If the robot already exists then that graphical model will be found
% and moved.
%
% MULTIPLE VIEWS OF THE SAME ROBOT
% If one or more plots of this robot already exist then these will all
% be moved according to the argument Q.  All robots in all windows with
% the same name will be moved.
%
% MULTIPLE ROBOTS
% Multiple robots can be displayed in the same plot, by using "hold on"
% before calls to plot(robot).
%
% options is a list of any of the following:
% 'workspace' [xmin, xmax ymin ymax zmin zmax]
% 'perspective' 'ortho'		controls camera view mode
% 'erase' 'noerase'		controls erasure of arm during animation
% 'loop' 'noloop'		controls endless loop mode
% 'base' 'nobase'		controls display of base 'pedestal'
% 'wrist' 'nowrist'		controls display of wrist
% 'name', 'noname'		display the robot's name
% 'shadow' 'noshadow'		controls display of shadow
% 'xyz' 'noa'			wrist axis label
% 'joints' 'nojoints'		controls display of joints
% 'mag' scale			annotation scale factor
%
% The options come from 3 sources and are processed in the order:
% 1. Cell array of options returned by the function PLOTBOTOPT
% 2. Cell array of options returned by the .plotopt method of the
%    robot object.  These are set by the .plotopt method.
% 3. List of arguments in the command line.
%
% GRAPHICAL ANNOTATIONS:
%
% The basic stick figure robot can be annotated with
%  shadow on the floor
%  XYZ wrist axes and labels
%  joint cylinders and axes
%
% All of these require some kind of dimension and this is determined
% using a simple heuristic from the workspace dimensions.  This dimension
% can be changed by setting the multiplicative scale factor using the
% 'mag' option above.
%
% GETTING GRAPHICAL ROBOT STATE:
% q = PLOT(ROBOT)
% Returns the joint configuration from the state of an existing
% graphical representation of the specified robot.  If multiple
% instances exist, that of the first one returned by findobj() is
% given.
%
% MOVING JUST ONE INSTANCE oF A ROBOT:
%
%  r = PLOT(robot, q)
%
% Returns a copy of the robot object, with the .handle element set.
%
%  PLOT(r, q)
%
% will animate just this instance, not all robots of the same name.
%
% See also: PLOTBOTOPT, DRIVEBOT, FKINE, ROBOT.


% HANDLES:
%
%  A robot comprises a bunch of individual graphical elements and these are
% kept in a structure which can be stored within the .handle element of a
% robot object:
%	h.robot		the robot stick figure
%	h.shadow	the robot's shadow
%	h.x		wrist vectors
%	h.y
%	h.z
%	h.xt		wrist vector labels
%	h.yt
%	h.zt
%
%  The plot function returns a new robot object with the handle element set.
%
% For the h.robot object we additionally:
%	- save this new robot object as its UserData
%	- tag it with the name field from the robot object
%
%  This enables us to find all robots with a given name, in all figures,
% and update them.

% Copyright (C) 1993-2008, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for Matlab (RTB).
%
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
%
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
%
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.

function rnew = plotrobot(robot, tg, skipFrame)

%
% q = PLOT(robot)
% return joint coordinates from a graphical robot of given name
%
if nargin < 2
    rh = findobj('Tag', robot.name);
    if ~isempty(rh)
        r = get(rh(1), 'UserData');
        rnew = r.q;
        % Input changed to inches, no need to convert
        %             rnew(6) = rnew(6)*25.4; % Convert inches to mm (r.q returns in, but joing variable defined in mm)
        rnew = rnew + [0 pi/2 -pi/2 pi/2 0 0]; % r.q returns total theta, but many joints have an offset, adjust for offset
    end
    return
end

if nargin < 3
    skipFrame = false;
end

% process options
opt = plot_options(robot);

np = size(tg,1);
n = robot.n;

%if numcols(tg) ~= n,
if size(tg,2) ~= n
    error('Insufficient columns in q')
end

if ~isempty(robot.handle)
    % handles provided, animate just that robot
    for r=1:opt.repeat
        for p=1:np
            animate( robot, tg(p,:));
            pause(opt.delay)
        end
    end
    
    return;
end

% Do the right thing with figure windows.
ax = gca;

% if this figure has no robot in it, create one
if isempty( findobj(ax, 'Tag', robot.name) )
    
    h = create_new_robot(robot, opt);
    
    % save the handles in the passed robot object, and
    % attach it to the robot as user data.
    robot.handle = h;
    set(h.robot, 'Tag', robot.name);
    set(h.robot, 'UserData', robot);
end

% get handle of any existing robot of same name
rh = findobj('Tag', robot.name);

% now animate all robots tagged with this name
for rep=1:opt.repeat
    for p=1:np
        for r = rh'
            if ~skipFrame
                animate( get(r, 'UserData'), tg(p,:));
            end
        end
    end
end

% save the last joint angles away in the graphical robot
for r=rh',
    rr = get(r, 'UserData');
    rr.q = tg(end,:);
    set(r, 'UserData', rr);
end

if nargout > 0,
    rnew = robot;
end

%PLOT_OPTIONS
%
%	o = PLOT_OPTIONS(robot, options)
%
% Returns an options structure

function o = plot_options(robot)
%%%%%%%%%%%%%% process options
o.erasemode = 'normal';
o.joints = robot.joints;
o.repeat = 1;
o.shadow = robot.shadow;
o.dims = [];
o.magscale = 1;
o.name = 1;
o.jointaxis = robot.jointaxis;
if isempty(o.dims),
    %
    % simple heuristic to figure the maximum reach of the robot
    %
    L = robot.link;
    reach = 0;
    for i=1:robot.n,
        reach = reach + abs(L{i}.A) + abs(L{i}.D);
    end
    o.dims = [-reach reach -reach reach 0 reach*5/3]*0.6;
    o.mag = reach/10;
else
    reach = min(abs(dims));
end
o.mag = o.magscale * reach/10;

%CREATE_NEW_ROBOT
%
%	h = CREATE_NEW_ROBOT(robot, opt)
%
% Using data from robot object and options create a graphical robot in
% the current figure.
%
% Returns a structure of handles to graphical objects.
%
% If current figure is empty, draw robot in it
% If current figure has hold on, add robot to it
% Otherwise, create new figure and draw robot in it.
%

function h = create_new_robot(robot, opt)
if ~isempty(robot.toolhandle)
    h.tool = robot.toolhandle;
    set(h.tool, 'erasemode', opt.erasemode);
end

h.mag = opt.mag;

%
% setup an axis in which to animate the robot
%
figure(1)
axis(opt.dims);

xlabel('X (mm)')
ylabel('Y (mm)')
zlabel('Z (mm)')
set(gca,'xtick',-1000:100:1000, 'ytick',-1000:100:1000,'ztick',-1000:100:1000)
grid on
view(80,20)

axis equal vis3d
xlim([-400 400]);
ylim([-400 400]);
zlim([-300 600]);

zlims = get(gca, 'ZLim');
h.zmin = zlims(1);

% create a line which we will
% subsequently modify.
%
h.robot = line(robot.lineopt{:}, ...
    'Color', [0 0 0]);
if opt.shadow == 1,
    h.shadow = line(robot.shadowopt{:}, ...
        'Color', 0.6*ones(1,3));
end

% Create vectors for the end effector frame
h.endeffectorX = line([0,0],[0,0],'Color','r','LineWidth',2);
h.endeffectorY = line([0,0],[0,0],'Color','g','LineWidth',2);
h.endeffectorZ = line([0,0],[0,0],'Color','b','LineWidth',2);

%
% display spheres at each joint, as well as axis centerline.
%

L = robot.link;
for i=(1:robot.n-1),
    if opt.joints == 1,
        % sphere
        if L{i}.sigma == 0,
            N = 20;
        elseif L{i}.sigma == 1
            N = 4;
        end
        
        [xc, yc, zc] = sphere(N);
        xc = opt.mag/3 * xc;
        yc = opt.mag/3 * yc;
        zc = opt.mag/3 * zc;
        
        actuatoroffset = L{i}.actuatoroffset;
        xc = xc + actuatoroffset(1);
        yc = yc + actuatoroffset(2);
        zc = zc + actuatoroffset(3);
        
        % add the surface object and color it
        h.joint(i) = surface(xc,yc,zc);
        
        if length(h.joint) == i
            set(h.joint(i), 'FaceColor', 0.3*[1 1 1], 'EdgeColor', 'none');
            
            % build a matrix of coordinates so we
            % can transform the spheres in animate()
            % and hang it off the cylinder
            xyz = [xc(:)'; yc(:)'; zc(:)'; ones(1,length(xc(:)))];
            set(h.joint(i), 'UserData', xyz);
        end
    end
    
    if opt.jointaxis == 1
        % add a dashed line along the axis
        h.jointaxis(i) = line('xdata', [0;0], ...
            'ydata', [0;0], ...
            'zdata', [0;0], ...
            'color', 'blue', ...
            'linestyle', '--', ...
            'erasemode', 'xor');
    end
end

%ANIMATE   move an existing graphical robot
%
%	animate(robot, q)
%
% Move the graphical robot to the pose specified by the joint coordinates q.
% Graphics are defined by the handle structure robot.handle.

function animate(robot, q)

n = robot.n;
h = robot.handle;
L = robot.link;

mag = h.mag;

% compute the link transforms, and record the origin of each frame
% for the animation.

[X, T] = updateQ(q);

t = robot.base;
Tn = t;
for j=1:length(T),
    Tn(:,:,j) = t;
    t = t * T(:,:,j);
end

t = t *robot.tool;

x = X(:,1);
y = X(:,2);
z = X(:,3);
eeAxisLength = 50;
eex = t(1:3,1:3)*[eeAxisLength;0;0] + X(end,:)';
eey = t(1:3,1:3)*[0;eeAxisLength;0] + X(end,:)';
eez = t(1:3,1:3)*[0;0;eeAxisLength] + X(end,:)';

xs = x;
ys = y;
zs = h.zmin*ones(size(x));

%
% draw the robot stick figure and the shadow
%
set(h.robot, 'xdata', x, 'ydata', y, 'zdata', z);
if isfield(h, 'shadow')
    set(h.shadow, 'xdata', xs, 'ydata', ys, 'zdata', zs);
end

%
% Update the end effector frame
%
set(h.endeffectorX,'xdata',[x(end),eex(1)],'ydata',[y(end),eex(2)],'zdata',[z(end),eex(3)]);
set(h.endeffectorY,'xdata',[x(end),eey(1)],'ydata',[y(end),eey(2)],'zdata',[z(end),eey(3)]);
set(h.endeffectorZ,'xdata',[x(end),eez(1)],'ydata',[y(end),eez(2)],'zdata',[z(end),eez(3)]);

if isfield(h, 'tool')
    xyz = get(h.tool, 'UserData');
    p = t*xyz;
    set(h.tool, 'XData', p(1,:),  'YData', p(2,:),  'ZData', p(3,:));
end

%
% display the joints as spheres with rotation axes
%
for j=1:(length(X)-1),
    if isfield(h, 'joint'),
        % get coordinate data from the spheres
        xyz = get(h.joint(j), 'UserData');
        xyz = bsxfun(@plus,xyz,[X(j,:)';1]);
        
        ncols = sqrt(size(xyz,2));
        xc = reshape(xyz(1,:), [], ncols);
        yc = reshape(xyz(2,:), [], ncols);
        zc = reshape(xyz(3,:), [], ncols);
        set(h.joint(j), 'Xdata', xc, 'Ydata', yc, 'Zdata', zc);
        
    end
end

if isfield(h, 'tool')
    xyz = get(h.tool, 'UserData');
    xyz = t * xyz;
    
    set(h.tool, 'XData', xyz(1,:), 'YData', xyz(2,:), 'ZData', xyz(3,:));
end

drawnow
