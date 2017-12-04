%SUBSREF Reference methods on a LINK object
%
%	LINK(q)		return link transform (A) matrix
%
%	LINK.alpha	return DH parameters
%	LINK.A
%	LINK.theta
%	LINK.D
%	LINK.sigma	return prismatic flag
%	LINK.RP		return 'R' or 'P'
%	LINK.mdh	0 if standard D&H, else 1
%
%   LINK.points return points to plot
%
%	LINK.offset	return joint variable offset
%	LINK.dh		return legacy DH row

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

function v = subsref(l, s)
	if strcmp(s(1).type, '()')
		if l.mdh == 0,
			v = linktran([l.alpha l.A l.theta l.D l.sigma], ...
				s(1).subs{1}+l.offset);
		else
			v = mlinktran([l.alpha l.A l.theta l.D l.sigma], ...
				s(1).subs{1}+l.offset);
		end
	elseif s(1).type  ~= '.'
		error('only .field supported')
	else

		% NOTE WELL:  the following code can't use getfield() since
		% getfield()  uses this, and Matlab will crash!!

		el = char(s(1).subs);
		switch el,
		%%%%%%% kinematic parameters
		case 'alpha',
			v = l.alpha;
		case 'A',
			v = l.A;
		case 'theta',
			v = l.theta;
		case 'D',
			v = l.D;
		case 'offset',
			v = l.offset;
		case 'sigma',
			v = l.sigma;
        case 'points',
			v = l.points;
        case 'actuatoroffset',
			v = l.actuatoroffset;
		case 'RP',
			if l.sigma == 1,
				v = 'P';
			else
				v = 'R';
			end
		case 'mdh',
			v = l.mdh;

		%%%%%%% legacy parameters
		case 'dh',
			v = [l.alpha l.A l.theta l.D l.sigma];
		otherwise, disp('Unknown method')
		end
	end


%LINKTRAN	Compute the link transform from kinematic parameters
%
%	LINKTRAN(alpha, an, theta, dn)
%	LINKTRAN(DH, q) is a homogeneous 
%	transformation between link coordinate frames.
%
%	alpha is the link twist angle
%	an is the link length
%	theta is the link rotation angle
%	dn is the link offset
%	sigma is 0 for a revolute joint, non-zero for prismatic
%
%	In the second case, q is substitued for theta or dn according to sigma.
%
%	Based on the standard Denavit and Hartenberg notation.

%	Copyright (C) Peter Corke 1993
function t = linktran(a, b, c, d)

	if nargin == 4,
		alpha = a;
		an = b;
		theta = c;
		dn = d;
	else
		if size(a,2) < 4,
			error('too few columns in DH matrix');
		end
		alpha = a(1);
		an = a(2);
		if size(a,2) > 4,
			if a(5) == 1,	% prismatic
                theta = a(3);
				dn = b;
			else		% revolute
				theta = b;
				dn = a(4);
			end
		else
			theta = b;	% assume revolute if sigma not given
			dn = a(4);
		end
	end
	sa = sin(alpha); ca = cos(alpha);
	st = sin(theta); ct = cos(theta);

	t =    [	ct	-st*ca	st*sa	an*ct
			st	ct*ca	-ct*sa	an*st
			0	sa	ca	dn
			0	0	0	1];

