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
        
    properties (Constant, Hidden)
        TOL = 0.01;
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
                if size(p1,2)~=2
                    error('point 1 must have 2 columns')
                end
                if size(p2,2)~=2
                    error('point 2 must have 2 columns')
                end
                % allow for a list of lines
                [Nmax,Imax] = max([size(p1,1) size(p2,1)]);
                [Nmin,Imin] = min([size(p1,1) size(p2,1)]);
                if Nmax~=Nmin
                    if Nmin>1
                        error('error. trying to create a list of line2d with different number of points')
                    else
                        switch Imin
                            case 1
                                p1 = repmat(p1,Nmax,1);
                            case 2
                                p2 = repmat(p2,Nmax,1);
                        end
                    end
                end
                % assign point 1 and point 2 to object
                obj.point1 = p1;
                obj.point2 = p2;
            else
                % empty object
            end
        end
        
        % coefficients of implicit equation
        % ax + by + c = 0
        function a = get.a(obj)
            a = obj.point1(:,2) - obj.point2(:,2);
        end
        function b = get.b(obj)
            b = obj.point2(:,1) - obj.point1(:,1);
        end
        function c = get.c(obj)
            c = obj.point1(:,1) .* obj.point2(:,2) - ...
                obj.point2(:,1) .* obj.point1(:,2);
        end
        function n = get.n(obj)
            % normalization factor
            n = sqrt(obj.a.^2 + obj.b.^2);
        end
    
        % convert line to a vector
        function vec = toVector(obj)
            vec = obj.point2 - obj.point1;
        end
        
        % get length of line segment
        function len = length(obj)
            vec = toVector(obj);
            len = sqrt(vec(:,1).^2 + vec(:,2).^2);
        end
        
        % get number of entries in an array of lines
        function c = count(obj)
            c = size(obj.point1,1);
        end
        
        % get a subset of the object
        function one = getone(obj,idx)
            if idx>obj.count
                error('index greater than object count')
            end
            one = line2d(obj.point1(idx,:),obj.point2(idx,:),obj.type);
        end
        
        % get a normalized vector
        function nvec = normvec(obj)
            vec = toVector(obj);
            len = length(obj);
            hasLength = len>line2d.TOL;
            nvec = vec;
            nvec(hasLength,:) = ...
                [vec(hasLength,1)./len(hasLength) vec(hasLength,2)./len(hasLength)];
        end
        
        % get angle of line in radians
        function ang = angleRad(obj)
            nvec = normvec(obj);
            ang = atan2(nvec(:,2),nvec(:,1));
        end
        
        % get angle of line in degrees
        function ang = angleDeg(obj)
            ang = angleRad(obj)*180/pi;
        end
       
        % flip directionality of a line
        function obj = flipDirection(obj,selection)
            if nargin<2
                selection = true(obj.count,1);
            end
            tmp = obj.point2(selection,:);
            obj.point2(selection,:) = obj.point1(selection,:);
            obj.point1(selection,:) = tmp;
        end
        
        % rotate line about it's start point in radians
        function obj = rotateRad(obj,theta,selection)
            if nargin<3
                selection = true(obj.count,1);
            end
            nvec = normvec(obj);
            c = cos(theta);
            s = sin(theta);
%             R = [c -s; s c];
%             vp = R * vec';
            vecrot = [c.*nvec(:,1)-s.*nvec(:,2) s.*nvec(:,1)+c.*nvec(:,2)];
            % denormalize the rotated vector
            len = length(obj);
            hasLength = len>line2d.TOL;
            vecrot(hasLength,:) = ...
                [vecrot(hasLength,1).*len(hasLength) vecrot(hasLength,2).*len(hasLength)];
            obj.point2(selection,:) = vecrot(selection,:) + obj.point1(selection,:);
        end
        
        % rotate line about it's start point in degrees
        function obj = rotateDeg(obj,theta,selection)
            if nargin<3
                selection = true(obj.count,1);
            end
            obj = rotateRad(obj,theta*pi/180,selection);
        end
        
        % rotate line about it's end point in radians
        function obj = rotateEndRad(obj,theta)
            obj = flipDirection(rotateRad(flipDirection(obj),theta));
        end
        
        % rotate line about it's end point in degrees
        function obj = rotateEndDeg(obj,theta)
            obj = flipDirection(rotateDeg(flipDirection(obj),theta));
        end
        
        % translate the line by a vector offset
        function obj = translate(obj,offset)
            obj.point1(:,1) = obj.point1(:,1) + offset(:,1);
            obj.point1(:,2) = obj.point1(:,2) + offset(:,2);
            obj.point2(:,1) = obj.point2(:,1) + offset(:,1);
            obj.point2(:,2) = obj.point2(:,2) + offset(:,2);
        end
        
        % what side of a line does a point fall?
        % positive if to the left (CCW)
        % negative if to the right (CW)
        function s = side(obj,point)
            cp = ( (obj.point2(:,1) - obj.point1(:,1)) .* ...
                   (point(:,2) - obj.point1(:,2)) - ...
                   (obj.point2(:,2) - obj.point1(:,2)) .* ...
                   (point(:,1) - obj.point1(:,1)) );
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
            perp(:,1) = -1*perp(:,1);
            point2 = point1 + perp;
            num = (point1(:,2)-obj.point1(:,2)) .* vec(:,1) - ...
                  (point1(:,1)-obj.point1(:,1)) .* vec(:,2);
            den = (point2(:,1)-point1(:,1)) .* vec(:,2) - ...
                  (point2(:,2)-point1(:,2)) .* vec(:,1);
            if any(abs(den)<1e-3)
                error('lines should not be parallel')
            end
            r = num ./ den;
            pointI = point1 + r.*(point2-point1);
            pline = line2d(point1,pointI);
        end
        
        % find a point on a line parametrically starting from point 1
        function point = getPoint(obj,p)
            vec = obj.toVector;
            point = obj.point1 + [p.*vec(:,1) p.*vec(:,2)];
        end
        
        % find the distance between two points of parametric lines given
        % the same parameter, p
        function dist = distToLineParametric(obj,line,p)
            if ~isa(line,'line2d')
                error('must supply line2d object as input')
            end
            point1 = getPoint(obj,p);
            point2 = line.getPoint(p);
            dline = line2d(point1,point2);
            dist = dline.length;
        end
        
        % distance to a point, depends on line type
        % returns the distance, the line whos length is the distance, and a
        % boolean, inseg, that show if the perpendicular distance falls
        % without the line segment.  If not, and a segment or ray is specified,
        % then the distance would not be perpendicular and would go to one
        % of the endpoints of the segment/ray
        function [dist,dline,inseg] = distToPoint(obj,point,type)
            % check for incompatible array sizes in the inputs
            c1 = obj.count;
            c2 = size(point,1);
            if c1~=c2 && c1>1 && c2>1
                error('point counts are not compatible in the array inputs')
            end
            % if one is a scalar that needs to be replicated, use the
            % line2d constructor to accomplish it
            if c1~=c2
                ln = line2d(obj.point1,point);
                obj = line2d(ln.point1,obj.point2,obj.type);
            end
            % delegate the distance calculation to the appropriate function
            if nargin<3
                type = obj.type;
            end
            switch type
                case 'line'
                    [dist,dline,inseg] = distPointToLine(obj,point);
                case 'segment'
                    [dist,dline,inseg] = distPointToSegment(obj,point);
                case 'ray'
                    [dist,dline,inseg] = distPointToRay(obj,point);
                otherwise
                    error('unknown line type')
            end
        end
                
        % plot a line, showing a square at point 1 and a star at point 2
        function plot(obj,varargin)
            c1 = obj.count;
            if c1==1
                plot([obj.point1(1); obj.point2(1)], ...
                    [obj.point1(2); obj.point2(2)],varargin{:}, ...
                    obj.point1(1),obj.point1(2),'gs', ...
                    obj.point2(1),obj.point2(2),'g*')
            else
                hold on
                for idx = 1:c1
                    plot([obj.point1(idx,1); obj.point2(idx,1)], ...
                        [obj.point1(idx,2); obj.point2(idx,2)],varargin{:}, ...
                        obj.point1(idx,1),obj.point1(idx,2),'gs', ...
                        obj.point2(idx,1),obj.point2(idx,2),'g*')
                end
            end
        end
        
        function plotn(obj,n,varargin)
            n1 = obj.count;
            if n1==1
                idx = 1;
            elseif n1<n
                idx = 1:n1;
            else
                factor = int32(n1/n);
                idx = 1:factor:n*factor;
            end
            hold on
            for i = 1:length(idx)
                plot([obj.point1(idx(i),1); obj.point2(idx(i),1)], ...
                    [obj.point1(idx(i),2); obj.point2(idx(i),2)],varargin{:}, ...
                    obj.point1(idx(i),1),obj.point1(idx(i),2),'gs', ...
                    obj.point2(idx(i),1),obj.point2(idx(i),2),'g*')
            end
        end
        
        function ploti(obj,i,varargin)
            n1 = obj.count;
            if islogical(i)
                idx = find(i);
            else
                idx = i;
            end
            if max(idx)>n1
                error('ploti index out of range')
            end
            hold on
            for i = 1:length(idx)
                plot([obj.point1(idx(i),1); obj.point2(idx(i),1)], ...
                    [obj.point1(idx(i),2); obj.point2(idx(i),2)],varargin{:}, ...
                    obj.point1(idx(i),1),obj.point1(idx(i),2),'gs', ...
                    obj.point2(idx(i),1),obj.point2(idx(i),2),'g*')
            end
        end
        
        % plot a line up to a point that is specified parametrically
        function plotpp(obj,p,varargin)
            v = obj.toVector;
            point = obj.point1 + p.*v;
            plot(point(1),point(2),varargin{:})
        end
    end
    
    
    methods (Access = private)
        % perpendicular distance from a point to a line, implicit equation
        function dist = distPointToLineImplicit(obj,point)
            dist = abs(obj.a .* point(:,1) + obj.b .* point(:,2)  + obj.c) ./ obj.n;
        end
        
        % perpendicular distance from a point to a line, parametric
        function [dist,dline,inseg] = distPointToLine(obj,point)
            w = line2d(obj.point1,point);
            num = line2d.dotProduct(w,obj);
            den = line2d.dotProduct(obj,obj);
            s = num./den;
            p = getPoint(obj,s);
            dline = line2d(p,point,'segment');
            dist = length(dline);
            inseg = s>=0 & s<=1;
        end
        
        % distance from a point to a ray
        function [dist,dline,inseg] = distPointToRay(obj,point)
            w = line2d(obj.point1,point);
            d = line2d.dotProduct(w,obj);
            isBeyondEnd = d<=0;
            % point's projection falls on line segment
            [dist,dline] = distToPoint(obj,point,'line');
            % point is beyond point 1 of line
            dist2 = length(w);
            % assign appropriate value(s) for dist and endp
            dist(isBeyondEnd) = dist2(isBeyondEnd);
            dline.point1(isBeyondEnd,:) = w.point1(isBeyondEnd,:);
            dline.point2(isBeyondEnd,:) = w.point2(isBeyondEnd,:);
            inseg = ~isBeyondEnd;
        end
        
        % distance from a point to a line segment
        function [dist,dline,inseg] = distPointToSegment(obj,point)
            w1 = line2d(obj.point1,point);
            w2 = line2d(obj.point2,point);
            d1 = line2d.dotProduct(w1,obj);
            isBeyondEnd1 = d1<=0;
            d2 = line2d.dotProduct(obj,obj);
            isBeyondEnd2 = d2<=d1;
            % point's projection falls on line segment
            [dist,dline] = distToPoint(obj,point,'line');
            % point is beyond point 1 of line
            dist1 = length(w1);
            % point is beyond point 2 of line
            dist2 = length(w2);
            % assign appropriate value(s) for dist and endp
            dist(isBeyondEnd2) = dist2(isBeyondEnd2);
            dline.point1(isBeyondEnd2,:) = w2.point1(isBeyondEnd2,:);
            dline.point2(isBeyondEnd2,:) = w2.point2(isBeyondEnd2,:);
            dist(isBeyondEnd1) = dist1(isBeyondEnd1);
            dline.point1(isBeyondEnd1,:) = w1.point1(isBeyondEnd1,:);
            dline.point2(isBeyondEnd1,:) = w1.point2(isBeyondEnd1,:);
            inseg = ~isBeyondEnd1 & ~isBeyondEnd2;
        end 
    end
    
    % static methods
    methods (Static)      
        % dot product between lines
        function d = dotProduct(line1,line2)
            if ~isa(line1,'line2d') || ~isa(line2,'line2d')
                error('must supply line2d objects as input')
            end
            vec1 = line1.toVector();
            vec2 = line2.toVector();
            d = vec1(:,1).*vec2(:,1) + vec1(:,2).*vec2(:,2);
        end
        
        % angle between lines in radians
        function ang = angleIncludedRad(line1,line2)
            if ~isa(line1,'line2d') || ~isa(line2,'line2d')
                error('must supply line2d objects as input')
            end
            nvec1 = line1.normvec;
            nvec2 = line2.normvec;
            d = nvec1(:,1).*nvec2(:,1) + nvec1(:,2).*nvec2(:,2);
            ang = real( acos(d) );
        end
        
        % angle between lines in degrees
        function ang = angleIncludedDeg(line1,line2)
            if ~isa(line1,'line2d') || ~isa(line2,'line2d')
                error('must supply line2d objects as input')
            end
            ang = line2d.angleIncludedRad(line1,line2)*180/pi;
        end
        
        % are lines in same direction?
        function samedir = compareDirection(line1,line2)
            if ~isa(line1,'line2d') || ~isa(line2,'line2d')
                error('must supply line2d objects as input')
            end
            samedir = abs(line2d.angleIncludedRad(line1,line2))<line2d.TOL;
        end
        
        % are lines parallel?
        function parallel = parallel(line1,line2)
            if ~isa(line1,'line2d') || ~isa(line2,'line2d')
                error('must supply line2d objects as input')
            end
            parallel = line2d.compareDirection(line1,line2) | ...
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
        
        % find intersection between parametric lines
        % line1: (x1,y1),(x2,y2),t1
        % line2: (x3,y3),(x4,y4),t2
        % if the intersection is on a line its parameter will be in [0,1]
        function [p,t1,t2] = intersection(line1,line2)
            if ~isa(line1,'line2d') || ~isa(line2,'line2d')
                error('must supply line2d objects as input')
            end
            x1 = line1.point1(:,1);
            y1 = line1.point1(:,2);
            x2 = line1.point2(:,1);
            y2 = line1.point2(:,2);
            x3 = line2.point1(:,1);
            y3 = line2.point1(:,2);
            x4 = line2.point2(:,1);
            y4 = line2.point2(:,2);
            
            % common denominator for the calculations
            den = (x4-x3).*(y2-y1) - (x2-x1).*(y4-y3);
            
            % line 1 parameter at intersection
            t1num = (x4-x3).*(y3-y1) - (x3-x1).*(y4-y3);
            t1 = t1num./den;
            
            % line 2 parameter at intersection
            t2num = (x2-x1).*(y3-y1) - (x3-x1).*(y2-y1);
            t2 = t2num./den;
            
            % solve for the intersection point
            x = x1 + (x2-x1).*t1;
            y = y1 + (y2-y1).*t1;
            p = [x y];
        end
        
        % distance from line2d to line2d
        function [dist,dline] = distParallel(line1,line2)
            if ~isa(line1,'line2d') || ~isa(line2,'line2d')
                error('must supply line2d objects as input')
            end
            isParallel = line2d.parallel(line1,line2);
            if any(~isParallel)
                error('must be parallel')
            end
            switch line1.type
                case 'line'
                    [dist,dline] = distToPoint(line1,line2.point1);
                case 'ray'
                    switch line2.type
                        case 'line'
                            [dist,dline] = distToPoint(line1,line2.point1);
                        case 'ray'
                            [dist,dline] = line2d.distRayToRayParallel(line1,line2);
                        case 'segment'
                            [dist,dline] = line2d.distRayToSegmentParallel(line1,line2);
                    end
                case 'segment'
                    switch line2.type
                        case 'line'
                            [dist,dline] = distToPoint(line1,line2.point1);
                       case 'ray'
                            [dist,dline] = line2d.distSegmentToRayParallel(line1,line2);
                        case 'segment'
                            [dist,dline] = line2d.distSegmentToSegmentParallel(line1,line2);
                    end
            end
        end
        
        % distance from ray to ray
        function [dist,dline] = distRayToRayParallel(ray1,ray2)
            if ~isa(ray1,'line2d') || ~isa(ray2,'line2d')
                error('must supply line2d objects as input')
            end
            isParallel = line2d.parallel(ray1,ray2);
            if any(~isParallel)
                error('must be parallel')
            end
            samedir = line2d.compareDirection(ray1,ray2);
            % default is to assume same direction and compute line to line
            % distance
            [dist,dline] = distToPoint(ray1,ray2.point1,'line');
            % cases where not same direction
            [dist1a,dline1a] = distToPoint(ray1,ray2.point1,'ray');
            dist(~samedir) = dist1a(~samedir);
            dline.point1(~samedir,:) = dline1a.point1(~samedir,:);
            dline.point2(~samedir,:) = dline1a.point2(~samedir,:);
        end
        
        % distance from ray to segment
        function [dist,dline] = distRayToSegmentParallel(ray1,segment2)
            if ~isa(ray1,'line2d') || ~isa(segment2,'line2d')
                error('must supply line2d objects as input')
            end
            isParallel = line2d.parallel(ray1,segment2);
            if any(~isParallel)
                error('must be parallel')
            end
            [dist,dline,isInSegment] = distToPoint(segment2,ray1.point1,'segment');
            % cases where ray end is not in segment.  in other words, the
            % segment lies to one side or the other of the ray endpoint
            distc = cell(1,2);
            [distc{1},dlinep1] = distToPoint(ray1,segment2.point1,'ray');
            [distc{2},dlinep2] = distToPoint(ray1,segment2.point2,'ray');
            % which end of segment is closer to point 1 of ray?
            [distmin,Imin] = min([distc{1} distc{2}],[],2);
            use1 = Imin==1;
            use2 = Imin==2;
            dist(~isInSegment) = distmin(~isInSegment);
            dline.point1(~isInSegment & use1,:) = dlinep1.point1(~isInSegment & use1,:);
            dline.point2(~isInSegment & use1,:) = dlinep1.point2(~isInSegment & use1,:);
            dline.point1(~isInSegment & use2,:) = dlinep2.point1(~isInSegment & use2,:);
            dline.point2(~isInSegment & use2,:) = dlinep2.point2(~isInSegment & use2,:);
        end
        
        % distance from segment to ray
        function [dist,dline] = distSegmentToRayParallel(segment1,ray2)
            if ~isa(segment1,'line2d') || ~isa(ray2,'line2d')
                error('must supply line2d objects as input')
            end
            isParallel = line2d.parallel(segment1,ray2);
            if any(~isParallel)
                error('must be parallel')
            end
            [dist,dline] = line2d.distRayToSegmentParallel(ray2,segment1);
            dline = dline.flipDirection;
        end
        
        % distance from line segment to line segment
        function [dist,dline] = distSegmentToSegmentParallel(segment1,segment2,selection)
            if ~isa(segment1,'line2d') || ~isa(segment2,'line2d')
                error('must supply line2d objects as input')
            end
            if nargin<3
                selection = true(size(segment1.point1,1),1);
            end
            isParallel = line2d.parallel(segment1,segment2);
            if any(~isParallel & selection)
                error('must be parallel')
            end
            dist1c = cell(1,4);
            [dist1c{1},dline] = distToPoint(segment1,segment2.point1,'segment');
            [dist1c{2},dlinep22] = distToPoint(segment1,segment2.point2,'segment');
            [dist1c{3},dlinep11] = distToPoint(segment2,segment1.point1,'segment');
            dlinep11 = dlinep11.flipDirection;
            [dist1c{4},dlinep12] = distToPoint(segment2,segment1.point2,'segment');
            dlinep12 = dlinep12.flipDirection;
            [dist,Imin] = min([dist1c{1} dist1c{2} dist1c{3} dist1c{4}],[],2);
            use2 = Imin==2;
            dline.point1(use2,:) = dlinep22.point1(use2,:);
            dline.point2(use2,:) = dlinep22.point2(use2,:);
            use3 = Imin==3;
            dline.point1(use3,:) = dlinep11.point1(use3,:);
            dline.point2(use3,:) = dlinep11.point2(use3,:);
            use4 = Imin==4;
            dline.point1(use4,:) = dlinep12.point1(use4,:);
            dline.point2(use4,:) = dlinep12.point2(use4,:);
        end        
    end
end