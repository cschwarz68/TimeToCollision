% Copyright (c) 2013, cschwarz68
% All rights reserved.
% 
% Redistribution and use in source and binary forms, with or without modification,
% are permitted provided that the following conditions are met:
% 
%   Redistributions of source code must retain the above copyright notice, this
%   list of conditions and the following disclaimer.
% 
%   Redistributions in binary form must reproduce the above copyright notice, this
%   list of conditions and the following disclaimer in the documentation and/or
%   other materials provided with the distribution.
% 
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
% ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
% WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
% DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
% ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
% (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
% LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
% ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
% (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
% SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

% a two dimensional line class that has two points and a type: line, ray, 
% or segment.  The line2d object holds one point in time and doesn't yet
% support vectorized operations.  The class has static methods that
% calculate minimum distances between two line2d's of various types.  
% Vectors and points are both represented by 1x2 arrays.
classdef line2d
    % class properties
    properties
        point1
        point2
        type = 'line'
    end
    
    % hidden properties for the implicit equation of a line
    properties (Hidden = true, Dependent = true)
        a
        b
        c
        n
    end
        
    % methods
    methods
        % constructor takes type points and a type.  the first point is
        % understood to be closer to the 'beginning' of the line than the
        % second point...in other words, line2d's have direction.  the type
        % of a line can be 'line', 'ray', or 'segment'.  points must be 1x2
        % arrays
        function obj = line2d(p1,p2,type)
            if nargin>0
                if nargin<2
                    error('must supply 2 points for a line')
                elseif nargin<3
                    obj.type = 'line';
                else
                    obj.type = type;
                    switch obj.type
                        case 'line'
                            % ok
                        case 'segment'
                            % ok
                        case 'ray'
                            % ok
                        otherwise
                            error('only line types supported are line, segment, and ray')
                    end
                end
                if length(p1)~=2
                    error('point 1 must be of length 2')
                end
                if length(p2)~=2
                    error('point 2 must be of length 2')
                end
                obj.point1 = p1;
                obj.point2 = p2;
            else
                % empty object
            end
        end
        
        % coefficients of implicit equation
        % ax + by + c = 0
        function a = get.a(obj)
            a = obj.point1(2) - obj.point2(2);
        end
        function b = get.b(obj)
            b = obj.point2(1) - obj.point1(1);
        end
        function c = get.c(obj)
            c = obj.point1(1) * obj.point2(2) - ...
                obj.point2(1) * obj.point1(2);
        end
        function n = get.n(obj)
            % normalization factor
            n = sqrt(obj.a^2 + obj.b^2);
        end
    
        % get length of line segment
        function len = length(obj)
            len = norm(toVector(obj));
        end
        
        % get angle of line in radians
        function ang = angleRad(obj)
            vec = toVector(obj);
            ang = atan2(vec(2),vec(1));
        end
        
        % get angle of line in degrees
        function ang = angleDeg(obj)
            ang = angleRad(obj)*180/pi;
        end
       
        % flip directionality of a line
        function obj = flipDirection(obj)
            tmp = obj.point2;
            obj.point2 = obj.point1;
            obj.point1 = tmp;
        end
        
        % rotate line about it's start point in radians
        function obj = rotateRad(obj,theta)
            vec = toVector(obj);
            c = cos(theta);
            s = sin(theta);
            R = [c -s; s c];
            vp = R * vec';
            obj.point2 = vp' + obj.point1;
        end
        
        % rotate line about it's start point in degrees
        function obj = rotateDeg(obj,theta)
            obj = rotateRad(obj,theta*pi/180);
        end
        
        % rotate line about it's end point in radians
        function obj = rotate2Rad(obj,theta)
            obj = flipDirection(rotateRad(flipDirection(obj),theta));
        end
        
        % rotate line about it's end point in degrees
        function obj = rotate2Deg(obj,theta)
            obj = flipDirection(rotateDeg(flipDirection(obj),theta));
        end
        
        % convert line to a vector
        function vec = toVector(obj)
            vec = obj.point2 - obj.point1;
        end
        
        % what side of a line does a point fall?
        % positive if to the left (CCW)
        % negative if to the right (CW)
        function s = side(obj,point)
            cp = ((obj.point2(1) - obj.point1(1))*(point(2) - obj.point1(2)) - ...
                (obj.point2(2) - obj.point1(2))*(point(1) - obj.point1(1)));
            s = sign(cp);
        end
                
        % find intersection of two perpendicular lines
        % use parameteric equations so that
        % A+r(B-A) = C+s(D-C)
        % and solve for parameter r
        % return the perpendicular line from the point to the line
        function pline = intersectionPerp(obj,point1)
            vec = obj.toVector;
            perp = fliplr(vec);
            perp(1) = -1*perp(1);
            point2 = point1 + perp;
            num = (point1(2)-obj.point1(2)) * vec(1) - ...
                (point1(1)-obj.point1(1)) * vec(2);
            den = (point2(1)-point1(1)) * vec(2) - ...
                (point2(2)-point1(2)) * vec(1);
            if abs(den)<1e-6
                error('lines should not be parallel')
            end
            r = num / den;
            pointI = point1 + r*(point2-point1);
            pline = line2d(point1,pointI);
        end
        
        % find a point on a line parametrically starting from point 1
        function point = getPoint(obj,p)
            point = obj.point1 + p*obj.toVector;
        end
        
        % find the distance between two points of parametric lines given
        % the same parameter, p
        function dist = distToLineParametric(obj,line,p)
            if ~isa(line,'line2d')
                error('must supply line2d object as input')
            end
            point1 = obj.point1 + p*obj.toVector;
            point2 = line.point1 + p*line.toVector;
            dline = line2d(point1,point2);
            dist = dline.length;
        end
        
        % distance to a point, depends on line type
        function [dist,endp] = distToPoint(obj,point)
            switch obj.type
                case 'line'
                    dist = distPointToLine(obj,point);
                    endp = NaN;
                case 'segment'
                    [dist,endp] = distPointToSegment(obj,point);
                case 'ray'
                    [dist,endp] = distPointToRay(obj,point);
                otherwise
                    error('unknown line type')
            end
        end
        
        % perpendicular distance from a point to a line
        function dist = distPointToLine(obj,point)
            dist = abs(obj.a * point(1) + obj.b * point(2)  + obj.c) / obj.n;
        end
        
        % distance from a point to a ray
        function [dist,endp] = distPointToRay(obj,point)
            v = [obj.point2(1)-obj.point1(1) obj.point2(2)-obj.point1(2)];
            w = [point(1)-obj.point1(1) point(2)-obj.point1(2)];
            c1 = dot(w,v);
            if ( c1 <= 0 ) % point is beyond point 1 of line
                dist = norm(obj.point1-point);
                endp = obj.point1;
            else % point's projection falls on line segment
                dist = distPointToLine(obj,point);
                endp = NaN;
            end
        end
        
        % distance from a point to a line segment
        function [dist,endp] = distPointToSegment(obj,point)
            v = [obj.point2(1)-obj.point1(1) obj.point2(2)-obj.point1(2)];
            w = [point(1)-obj.point1(1) point(2)-obj.point1(2)];
            c1 = dot(w,v);
            c2 = dot(v,v);
            if ( c1 <= 0 ) % point is beyond point 1 of line
                dist = norm(obj.point1-point);
                endp = obj.point1;
            elseif ( c2 <= c1 ) % point is beyond point 2 of line
                dist = norm(obj.point2-point);
                endp = obj.point2;
            else % point's projection falls on line segment
                dist = distPointToLine(obj,point);
                endp = NaN;
            end
        end 
        
        % plot a line, showing a square at point 1 and a star at point 2
        function plot(obj,varargin)
            plot([obj.point1(1); obj.point2(1)], ...
                [obj.point1(2); obj.point2(2)],varargin{:}, ...
                obj.point1(1),obj.point1(2),'gs', ...
                obj.point2(1),obj.point2(2),'g*')
        end
        
        % plot a line up to a point that is specified parametrically
        function plotpp(obj,p,varargin)
            v = obj.toVector;
            point = obj.point1 + p*v;
            plot(point(1),point(2),varargin{:})
        end
    end
    
    % static methods
    methods (Static)        
        % angle between lines in radians
        function ang = angleIncludedRad(line1,line2)
            if ~isa(line1,'line2d') || ~isa(line2,'line2d')
                error('must supply line2d objects as input')
            end
            ang = acos(dot(toVector(line1),toVector(line2)) / ...
                (line1.length*line2.length));
        end
        
        % angle between lines in degrees
        function ang = angleIncludedDeg(line1,line2)
            if ~isa(line1,'line2d') || ~isa(line2,'line2d')
                error('must supply line2d objects as input')
            end
            ang = line2d.angleIncludedRad(line1,line2)*180/pi;
        end
        
        % are lines in same or opposite direction?
        function samedir = compareDirection(line1,line2)
            if ~isa(line1,'line2d') || ~isa(line2,'line2d')
                error('must supply line2d objects as input')
            end
            samedir = abs(line2d.angleIncludedRad(line1,line2))<1e-6;
        end
        
        % are lines parallel?
        function parallel = parallel(line1,line2)
            if ~isa(line1,'line2d') || ~isa(line2,'line2d')
                error('must supply line2d objects as input')
            end
            parallel = line2d.compareDirection(line1,line2) || ...
                line2d.compareDirection(flipDirection(line1),line2);
        end
        
        % are lines equal?
        function eq = equals(line1,line2)
            if ~isa(line1,'line2d') || ~isa(line2,'line2d')
                error('must supply line2d objects as input')
            end
            eq = all(line1.point1==line2.point1) && ...
                    all(line1.point2==line2.point2);
        end
        
        % distance from line2d to line2d
        function [dist,end1,end2] = distParallel(line1,line2)
            if ~isa(line1,'line2d') || ~isa(line2,'line2d')
                error('must supply line2d objects as input')
            end
            if ~line2d.parallel(line1,line2)
                error('must be parallel')
            end
            switch line1.type
                case 'line'
                    [dist,end1] = distToPoint(line1,line2.point1);
                    end2 = line2.point1;
                case 'ray'
                    switch line2.type
                        case 'line'
                            [dist,end1] = distToPoint(line1,line2.point1);
                            end2 = line2.point1;
                        case 'ray'
                            [dist,end1,end2] = line2d.distRayToRayParallel(line1,line2);
                        case 'segment'
                            [dist,end1,end2] = line2d.distRayToSegmentParallel(line1,line2);
                    end
                case 'segment'
                    switch line2.type
                        case 'line'
                            [dist,end1] = distToPoint(line1,line2.point1);
                            end2 = line2.point1;
                       case 'ray'
                            [dist,end1,end2] = line2d.distSegmentToRayParallel(line1,line2);
                        case 'segment'
                            [dist,end1,end2] = line2d.distSegmentToSegmentParallel(line1,line2);
                    end
            end
        end
        
        % distance from ray to ray
        function [dist,end1,end2] = distRayToRayParallel(ray1,ray2)
            if ~isa(ray1,'line2d') || ~isa(ray2,'line2d')
                error('must supply line2d objects as input')
            end
            if ~line2d.parallel(ray1,ray2)
                error('must be parallel')
            end
            v = [ray1.point2(1)-ray1.point1(1) ray1.point2(2)-ray1.point1(2)];
            w1 = [ray2.point1(1)-ray1.point1(1) ray2.point1(2)-ray1.point1(2)];
            w2 = [ray2.point2(1)-ray1.point1(1) ray2.point2(2)-ray1.point1(2)];
            d11 = dot(w1,v);
            d12 = dot(w2,v);
            samedir = line2d.compareDirection(ray1,ray2);
            % if rays are same direction, find perpendicular length and return
            if samedir
                dist = distPointToLine(ray1,ray2.point1);
                end1 = NaN;
                end2 = ray2.point1;
                return
            end
            % rays are in opposite direction
            % segment is beyond point 1 of line
            if (d11 <= 0)
                if d11 <= d12 % rays overlap
                    %  2<--------1
                    %         1-------->2
                    dist = distPointToLine(ray1,ray2.point1);
                    end1 = NaN;
                    end2 = ray2.point1;
                else % rays don't overlap
                    %  2<--------1
                    %              1-------->2
                    dist = norm(ray1.point1-ray2.point1);
                    end1 = ray1.point1;
                    end2 = ray2.point1;
                end
            else
                if d11 <= d12 % rays don't overlap
                    %              1-------->2
                    %  2<--------1
                    dist = norm(ray1.point1-ray2.point1);
                    end1 = ray1.point1;
                    end2 = ray2.point1;
                else % rays overlap
                    %         1-------->2
                    %  2<--------1
                    dist = distPointToLine(ray1,ray2.point1);
                    end1 = NaN;
                    end2 = ray2.point1;
                end
            end
        end
        
        % distance from ray to segment
        function [dist,end1,end2] = distRayToSegmentParallel(ray1,segment2)
            if ~isa(ray1,'line2d') || ~isa(segment2,'line2d')
                error('must supply line2d objects as input')
            end
            if ~line2d.parallel(ray1,segment2)
                error('must be parallel')
            end
            v = [ray1.point2(1)-ray1.point1(1) ray1.point2(2)-ray1.point1(2)];
            w1 = [segment2.point1(1)-ray1.point1(1) segment2.point1(2)-ray1.point1(2)];
            w2 = [segment2.point2(1)-ray1.point1(1) segment2.point2(2)-ray1.point1(2)];
            d11 = dot(w1,v);
            d12 = dot(w2,v);
            w3 = [segment2.point1(1)-ray1.point2(1) segment2.point1(2)-ray1.point2(2)];
            d21 = dot(w3,v);
            c2 = dot(v,v);
            if (d11 <= 0) && (d12 <= 0) && (d21 <= d11)
                if d11 <= d12 % point 2 is closest to line
                    %  1--------2
                    %             1-------->2
                    dist = norm(ray1.point1-segment2.point2);
                    end1 = ray1.point1;
                    end2 = segment2.point2;
                else % point 1 is closest to line
                    %  2--------1
                    %             1-------->2
                    dist = norm(ray1.point1-segment2.point1);
                    end1 = ray1.point1;
                    end2 = segment2.point1;
                end
            elseif (c2 <= d11) && (c2 <= d12) && (d11 <= d21)
                if d12 <= d11 % point 2 is closest to line
                    %              2--------1
                    %  2<--------1
                    dist = norm(ray1.point1-segment2.point2);
                    end1 = ray1.point1;
                    end2 = segment2.point2;
                else % point 1 is closest to line
                    %              1--------2
                    %  2<--------1
                    dist = norm(ray1.point1-segment2.point1);
                    end1 = ray1.point1;
                    end2 = segment2.point1;
                end
            else % the ray and segment overlap so find perpendicular length
                dist = distPointToLine(ray1,segment2.point1);
                end1 = NaN;
                end2 = segment2.point1;
            end
        end
        
        % distance from segment to ray
        function [dist,end1,end2] = distSegmentToRayParallel(segment1,ray2)
            if ~isa(segment1,'line2d') || ~isa(ray2,'line2d')
                error('must supply line2d objects as input')
            end
            if ~line2d.parallel(segment1,ray2)
                error('must be parallel')
            end
            v = [segment1.point2(1)-segment1.point1(1) segment1.point2(2)-segment1.point1(2)];
            w1 = [ray2.point1(1)-segment1.point1(1) ray2.point1(2)-segment1.point1(2)];
            w2 = [ray2.point2(1)-segment1.point1(1) ray2.point2(2)-segment1.point1(2)];
            d11 = dot(w1,v);
            d12 = dot(w2,v);
            w3 = [ray2.point1(1)-segment1.point2(1) ray2.point1(2)-segment1.point2(2)];
            d21 = dot(w3,v);
            c2 = dot(v,v);
            if ((d11 <= 0) && (d21 <= 0) && (d12 <= d11))
                if d11 <= d21 % point 2 is closest to line
                    %  2<--------1
                    %              2--------1
                    dist = norm(segment1.point2-ray2.point1);
                    end1 = segment1.point2;
                    end2 = ray2.point1;
                else % point 1 is closest to line
                    %  2<--------1
                    %              1--------2
                    dist = norm(segment1.point1-ray2.point1);
                    end1 = segment1.point1;
                    end2 = ray2.point1;
                end
            elseif (c2 <= d11) && (c2 <= d21) && (d11 <= d12)
                if c2p1 <= c1p1 % point 2 is closest to line
                    %              1-------->2
                    %  1--------2
                    dist = norm(segment1.point2-ray2.point1);
                    end1 = segment1.point2;
                    end2 = ray2.point1;
                else % point 1 is closest to line
                    %              1-------->2
                    %  2--------1
                    dist = norm(segment1.point1-ray2.point1);
                    end1 = segment1.point1;
                    end2 = ray2.point1;
                end
            else % the ray and segment overlap so find perpendicular length
                dist = distPointToLine(segment1,ray2.point1);
                end1 = NaN;
                end2 = ray2.point1;
            end
        end
        
        % distance from line segment to line segment
        function [dist,end1,end2] = distSegmentToSegmentParallel(segment1,segment2)
            if ~isa(segment1,'line2d') || ~isa(segment2,'line2d')
                error('must supply line2d objects as input')
            end
            if ~line2d.parallel(segment1,segment2)
                error('must be parallel')
            end
            v = [segment1.point2(1)-segment1.point1(1) segment1.point2(2)-segment1.point1(2)];
            w1 = [segment2.point1(1)-segment1.point1(1) segment2.point1(2)-segment1.point1(2)];
            w2 = [segment2.point2(1)-segment1.point1(1) segment2.point2(2)-segment1.point1(2)];
            d11 = dot(w1,v);
            d12 = dot(w2,v);
            w3 = [segment2.point1(1)-segment1.point2(1) segment2.point1(2)-segment1.point2(2)];
            w4 = [segment2.point2(1)-segment1.point2(1) segment2.point2(2)-segment1.point2(2)];
            d21 = dot(w3,v);
            d22 = dot(w4,v);
            c2 = dot(v,v);
            if (d11 <= 0) && (d12 <= 0) && (d21 <= d11)
                if d11 <= d12 % point 2 is closest to segment
                    %  1--------2
                    %             1--------2
                    dist = norm(segment1.point1-segment2.point2);
                    end1 = segment1.point1;
                    end2 = segment2.point2;
                else % point 1 is closest to segment
                    %  2--------1
                    %             1--------2
                    dist = norm(segment1.point1-segment2.point1);
                    end1 = segment1.point1;
                    end2 = segment2.point1;
                end
            elseif (d21 <= 0) && (d22 <= 0) && (d11 <= d21)
                if d21 <= d22 % point 2 is closest to segment
                    %  1--------2
                    %             2--------1
                    dist = norm(segment1.point2-segment2.point2);
                    end1 = segment1.point2;
                    end2 = segment2.point2;
                else % point 1 is closest to segment
                    %  2--------1
                    %             2--------1
                    dist = norm(segment1.point2-segment2.point1);
                    end1 = segment1.point2;
                    end2 = segment2.point1;
                end
            elseif (c2 <= d11) && (c2 <= d12) && (d11 <= d21)
                if d12 <= d11 % point 2 is closest to segment
                    %             2--------1
                    %  2--------1
                    dist = norm(segment1.point1-segment2.point2);
                    end1 = segment1.point1;
                    end2 = segment2.point2;
                else % point 1 is closest to segment
                    %             1--------2
                    %  2--------1
                    dist = norm(segment1.point1-segment2.point1);
                    end1 = segment1.point1;
                    end2 = segment2.point1;
                end
            elseif (c2 <= d21) && (c2 <= d22) && (d21 <= d11)
                if d22 <= d21 % point 2 is closest to segment
                    %             2--------1
                    %  1--------2
                    dist = norm(segment1.point2-segment2.point2);
                    end1 = segment1.point2;
                    end2 = segment2.point2;
                else % point 1 is closest to segment
                    %             1--------2
                    %  1--------2
                    dist = norm(segment1.point2-segment2.point1);
                    end1 = segment1.point2;
                    end2 = segment2.point1;
                end
            else % the segments overlap so find perpendicular length
                dist = distPointToLine(segment1,segment2.point1);
                end1 = NaN;
                end2 = segment2.point1;
            end
        end        
    end
end