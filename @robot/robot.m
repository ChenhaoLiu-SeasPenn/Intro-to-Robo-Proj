%ROBOT Robot object constructor
%
%	ROBOT			create a ROBOT object with no links
%	ROBOT(robot)		create a copy of an existing ROBOT object
%	ROBOT(robot, LINK)	replaces links for robot object
%	ROBOT(LINK, ...)	create from a cell array of LINK objects
%	ROBOT(DH, ...)		create from legacy DH matrix
%	ROBOT(..., 'PropertyName', PropertyValue)   create with plotting options
%
% Optional properties are:
% 	Name			'string' robot type or name
% 	Manufacturer	'string' who built it
% 	Comment			'string' general comment
%   Shadow          {'on'} | 'off' plot the shadow of the robot on the
%                       ground plane.
%   Joints          'on' | {'off'} display the joints of the robot
%   Jointaxis       'on' | {'off'} display the joint axes of the robot
%
% If the legacy matrix forms are used the default name is the workspace
% matrix that held the data.
%
% See also: ROBOT/SUBSREF, ROBOT/SUBSASGN, LINK.

% Copyright (C) 1999-2008, by Peter I. Corke
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

function r = robot(L, varargin)

	if nargin == 0
		r.name = 'noname';
		r.manuf = '';
		r.comment = '';
		r.link = {};
		r.n = 0;
		r.mdh = 0;
		r.gravity = [0; 0; 9.81];
		r.base = eye(4,4);
		r.tool = eye(4,4);
		r.handle = [];	% graphics handles
		r.q = [];	% current joint angles
		r.plotopt = {};
		r.lineopt = {'Color', 'black', 'Linewidth', 3};
		r.shadowopt = {'Color', 'black', 'Linewidth', 4};
		r = class(r, 'robot');
        r.hardware = false;
        r.LEDOn = false;
        r.LEDColor = zeros(1,3);
        r.LEDMarkerStyle = '*';
        r.LEDMarkerSize = 6;
        
        
	elseif isa(L, 'robot')
		r = L;
        if nargin == 2,
			r.link = varargin{1,1};
        end
        
    else
		if isa(L, 'double')
			% legacy matrix
			dh_dyn = L;
			clear L
			for j=1:size(dh_dyn,1)
				L{j} = link(dh_dyn(j,:));
			end
			% get name of variable
			r.name = inputname(1);
			r.link = L;
		elseif iscell(L) && isa(L{1}, 'link')
			r.link = L;
		else
			error('unknown type passed to robot');
		end
		r.n = length(L);

		% set the robot object mdh status flag
		mdh = [];
		for j = 1:length(L)
			mdh = [mdh L{j}.mdh];
		end
		if all(mdh == 0)
			r.mdh = mdh(1);
		elseif all (mdh == 1)
			r.mdh = mdh(1);
		else
			error('robot has mixed D&H link conventions');
		end

		% fill in default base and gravity direction
		r.gravity = [0; 0; 9.81];
		r.base = eye(4,4);
		r.tool = eye(4,4);
		r.handle = [];
        r.toolhandle = [];
		r.q = [];
		r.plotopt = {};
		r.lineopt = {'Color', 'black', 'Linewidth', 3};
		r.shadowopt = {'Color', 'black', 'Linewidth', 4};
        
        % set default options
        r.name = 'noname';
        r.manuf = '';
        r.comment = '';
        r.shadow = true;
        r.joints = true;
        r.jointaxis = false;
        r.sphericalwrist = [];
        r.hardware = false;
        
        r.LEDOn = false;
        r.LEDColor = zeros(1,3);
        r.LEDMarkerStyle = '*';
        r.LEDMarkerSize = 6;
        
        if mod(size(varargin,2), 2) == 1
            error('must enter a value for each property input');
        end
        for j = 1:2:size(varargin, 2)
            % Name
            if strcmpi(varargin{1,j}, 'name')
                r.name = varargin{1,j+1};
            
            % manufacturer
            elseif strcmpi(varargin{1,j}, 'manufacturer')
                r.manuf = varargin{1,j+1};
            
            % Name
            elseif strcmpi(varargin{1,j}, 'comment')
                r.comment = varargin{1,j+1};

            % Shadow
            elseif strcmpi(varargin{1,j}, 'shadow')
                if strcmpi(varargin{1,j+1}, 'on')
                    r.shadow = true;
                elseif strcmpi(varargin{1,j+1}, 'off')
                    r.shadow = false;
                else
                    error('invalid option for shadow')
                end
            
            % Joints
            elseif strcmpi(varargin{1,j}, 'joints')
                if strcmpi(varargin{1,j+1}, 'on')
                    r.joints = true;
                elseif strcmpi(varargin{1,j+1}, 'off')
                    r.joints = false;
                else
                    error('invalid option for joints')
                end
            
            % Jointaxis
            elseif strcmpi(varargin{1,j}, 'jointaxis')
                if strcmpi(varargin{1,j+1}, 'on')
                    r.jointaxis = true;
                elseif strcmpi(varargin{1,j+1}, 'off')
                    r.jointaxis = false;
                else
                    error('invalid option for jointaxis')
                end
                
            % LED Marker Style
            elseif strcmpi(varargin{1,j}, 'LEDMarkerStyle')
                if ~ischar(varargin{1, j+1})
                    error('LEDMarker must be a string')
                end
                r.LEDMarkerStyle = varargin{1, j+1};
                
            % LED Marker Size
            elseif strcmpi(varargin{1,j}, 'LEDMarkerSize')
                if ~isnumeric(varargin{1, j+1})
                    error('LEDMarkerSize must be a number')
                end
                r.LEDMarkerSize = varargin{1, j+1};
            
            % Tool
            elseif strcmpi(varargin{1,j}, 'tool')
                r.tool = varargin{1,j+1};
                
            % ToolHandle
            elseif strcmpi(varargin{1,j}, 'ToolHandle')
                r.toolhandle = varargin{1,j+1};
                
            % Spherical wrist
            elseif strcmpi(varargin{1,j}, 'SphericalWrist')
                r.sphericalwrist = varargin{1,j+1};
                
            % Hardware
            elseif strcmpi(varargin{1,j}, 'Hardware')
                if strcmpi(varargin{1,j+1}, 'on')
                    r.hardware = true;
                else
                    r.hardware = false;
                end
                
            else
                error('Invalid property for robot')
            end
        end

		r = class(r, 'robot');
	end
