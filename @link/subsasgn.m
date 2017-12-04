%SUBSASGN assignment methods on a LINK object
%
%	LINK.alpha = alpha	kinematic parameters
%	LINK.A = A
%	LINK.theta = theta
%	LINK.D = D
%	LINK.sigma = sigma	1 if joint is prismatic
%
%   LINK.points = points
%
%	LINK.offset = q0	joint coordinate offset

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

function l = subsasgn(l, s, v)

	if s(1).type  ~= '.'
		error('only .field supported')
	end
	switch s(1).subs,
	case 'alpha',
		l.alpha = v;
	case 'A',
		l.A = v;
	case 'theta',
		l.theta = v;
	case 'D',
		l.D = v;
	case 'offset',
		l.offset = v;
    case 'actuatoroffset',
		l.actuatoroffset = v;
	case 'sigma',
        if ischar(v)
            l.sigma = lower(v) == 'p';
        else
            l.sigma = v;
        end
    case 'points'
        l.points = v;
	otherwise, error('Unknown method')
	end
