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

% a two dimensional box class that has corners, sides, a center, angle, and
% velocity.  The box2d object holds one point in time and doesn't yet
% support vectorized operations.  The class has a static method that
% calculates minimum distance between two box2d's using the rotating
% calipers method.  Points are ordered in the clockwise direction, and
% sides are line2d objects with type 'segment'.  Vectors and points are
% both represented by 1x2 arrays.
classdef box2d
    % class properties
    properties
        c1
        c2
        c3
        c4
        center
        poi
        velocity = [0 0];
    end
    
    properties (Hidden)
        usepoi = false;
    end
    
    properties (Dependent)
        angle
    end
    
    properties (Constant)
        TOL = 0.03;
    end
    
    properties (Hidden, Dependent)
        side12
        side23
        side34
        side41
    end
    
    % class methods
    methods
        % constructor takes two points that are interpreted to be any two
        % opposite corners of the box.  If the two points are on a vertical
        % or horizontal line then an error is thrown.  The initial box has
        % orientation 0 degrees.
        function obj = box2d(p1,p2)
            if nargin>0
                if nargin<2
                    error('must supply 2 points for a line')
                else
                    if p1(1)==p2(1) || p1(2)==p2(2)
                        error('x and y coordinates of points must be distinct')
                    end
                    xmin = min([p1(1) p2(1)]);
                    xmax = max([p1(1) p2(1)]);
                    ymin = min([p1(2) p2(2)]);
                    ymax = max([p1(2) p2(2)]);
                    obj.c1 = [xmax ymin];
                    obj.c2 = [xmin ymin];
                    obj.c3 = [xmin ymax];
                    obj.c4 = [xmax ymax];
                    obj.center = [(xmin+xmax)/2 (ymin+ymax)/2];
                    obj.poi = obj.center;
                end
            else
                % empty object
            end
        end
                
        % set a point of interest (poi) relative to the center of the box
        % set the usepoi property so that rotations will take place with
        % respect to the poi rather than the center
        function obj = setpoi(obj,rp)
            obj.poi = obj.center + rp;
            obj.usepoi = true;
        end
        
        % getter for side, returned as a line2d segment
        function side12 = get.side12(obj)
            side12 = line2d(obj.c1,obj.c2,'segment');
        end
        
        % getter for side, returned as a line2d segment
        function side23 = get.side23(obj)
            side23 = line2d(obj.c2,obj.c3,'segment');
        end
        
        % getter for side, returned as a line2d segment
        function side34 = get.side34(obj)
            side34 = line2d(obj.c3,obj.c4,'segment');
        end
        
        % getter for side, returned as a line2d segment
        function side41 = get.side41(obj)
            side41 = line2d(obj.c4,obj.c1,'segment');
        end

        % getter for angle, returned in degrees
        function angle = get.angle(obj)
            angle = obj.side34.angleDeg;
        end
        
        % find the corner with the minimum X coordinate. return the corner
        % as well as the index of the corner (1-4)
        function [c,Ic] = minX(obj)
            [xmin,Ic] = min([obj.c1(1) obj.c2(1) obj.c3(1) obj.c4(1)]);
            switch Ic
                case 1
                    c = obj.c1;
                case 2
                    c = obj.c2;
                case 3
                    c = obj.c3;
                case 4
                    c = obj.c4;
            end
        end
        
        % find the corner with the maximum X coordinate. return the corner
        % as well as the index of the corner (1-4)
        function [c,Ic] = maxX(obj)
            [xmax,Ic] = max([obj.c1(1) obj.c2(1) obj.c3(1) obj.c4(1)]);
            switch Ic
                case 1
                    c = obj.c1;
                case 2
                    c = obj.c2;
                case 3
                    c = obj.c3;
                case 4
                    c = obj.c4;
            end
        end
        
        % find the corner with the minimum Y coordinate. return the corner
        % as well as the index of the corner (1-4)
        function [c,Ic] = minY(obj)
            [ymin,Ic] = min([obj.c1(2) obj.c2(2) obj.c3(2) obj.c4(2)]);
            switch Ic
                case 1
                    c = obj.c1;
                case 2
                    c = obj.c2;
                case 3
                    c = obj.c3;
                case 4
                    c = obj.c4;
            end
        end
        
        % find the corner with the maximum Y coordinate. return the corner
        % as well as the index of the corner (1-4)
        function [c,Ic] = maxY(obj)
            [ymax,Ic] = max([obj.c1(2) obj.c2(2) obj.c3(2) obj.c4(2)]);
            switch Ic
                case 1
                    c = obj.c1;
                case 2
                    c = obj.c2;
                case 3
                    c = obj.c3;
                case 4
                    c = obj.c4;
            end
        end
        
        % get a corner of the box given its index
        function c = getCorner(obj,Ic)
            switch Ic
                case 1
                    c = obj.c1;
                case 2
                    c = obj.c2;
                case 3
                    c = obj.c3;
                case 4
                    c = obj.c4;
                otherwise
                    error('only 4 corners in box2d')
            end
        end
        
        % get a side of the box given its index
        function side = getSide(obj,Is)
            switch Is
                case 1
                    side = obj.side12;
                case 2
                    side = obj.side23;
                case 3
                    side = obj.side34;
                case 4
                    side = obj.side41;
                otherwise
                    error('only 4 sides in box2d')
            end
        end
        
        % rotate the box around its center or poi by angle theta radians
        function obj = rotateRad(obj,theta)
            if obj.usepoi
                cc1 = line2d(obj.poi,obj.c1).rotateRad(theta);
                cc2 = line2d(obj.poi,obj.c2).rotateRad(theta);
                cc3 = line2d(obj.poi,obj.c3).rotateRad(theta);
                cc4 = line2d(obj.poi,obj.c4).rotateRad(theta);
            else
                cc1 = line2d(obj.center,obj.c1).rotateRad(theta);
                cc2 = line2d(obj.center,obj.c2).rotateRad(theta);
                cc3 = line2d(obj.center,obj.c3).rotateRad(theta);
                cc4 = line2d(obj.center,obj.c4).rotateRad(theta);
            end
            obj.c1 = cc1.point2;
            obj.c2 = cc2.point2;
            obj.c3 = cc3.point2;
            obj.c4 = cc4.point2;
        end
        
        % rotate the box around its center or poi by angle theta degrees
        function obj = rotateDeg(obj,theta)
            obj = rotateRad(obj,theta*pi/180);
        end
        
        % translate the box by a vector offset
        function obj = translate(obj,offset)
            if length(offset)~=2
                error('offset must be a 1x2 vector')
            end
            obj.center = obj.center + offset;
            obj.c1 = obj.c1 + offset;
            obj.c2 = obj.c2 + offset;
            obj.c3 = obj.c3 + offset;
            obj.c4 = obj.c4 + offset;
        end
        
        % plot a box2d
        function plot(obj,varargin)
            X = [obj.c1(1) obj.c2(1) obj.c3(1) obj.c4(1) obj.c1(1)]';
            Y = [obj.c1(2) obj.c2(2) obj.c3(2) obj.c4(2) obj.c1(2)]';
            plot(X,Y,varargin{:})
            hold on
            plot(obj.center(1),obj.center(2),'.')
            if obj.usepoi
                plot(obj.poi(1),obj.poi(2),'s')
            end
            plot(obj.c1(1),obj.c1(2),'.')
            axis equal
        end
    end
    
    % static methods
    methods (Static)
        % minimum distance between two boxes calculated using the rotating
        % calipers method.  The minimum distance may be
        % between two corners, a corner and a side, or two non-corner
        % points on a side.  the maximum distance is guaranteed to be
        % between two corners.  Because of that, this method is probably
        % not the most efficient method for calculating maximum distance.
        % Assume that one box is not fully enclosed by the other
        function [dist,collision,ends] = distance(box1,box2,show)
            if ~isa(box1,'box2d') || ~isa(box2,'box2d')
                error('must supply box2d objects as input')
            end
            if nargin<3
                show = false;
            end
            distv = NaN(1,20); % initialize distance vector
            sideofv = NaN(1,20); % initialize sideof vector
            obj1 = cell(1,20); % initialize object 1 cell array
            obj2 = cell(1,20); % initialize object 2 cell array
            count = 1; % initialize count
            % initialize caliper on first box at minimum Y point
            [c1,Ic1] = minY(box1);
            side1 = box1.getSide(Ic1);
            side1_init = side1;
            caliper1 = line2d(c1,c1-[1 0],'segment');
            % initialize caliper on second box at maximum Y point
            [c2,Ic2] = maxY(box2);
            side2 = box2.getSide(Ic2);
            side2_init = side2;
            caliper2 = line2d(c2,c2+[1 0],'segment');
            % initialize distance between calipers
            isParallel1 = line2d.parallel(caliper1,side1);
            isParallel2 = line2d.parallel(caliper2,side2);
            if isParallel1 && isParallel2
                % both calipers are parallel to sides
                distv(count) = line2d.distSegmentToSegmentParallel(caliper1,caliper2);
                obj1{count} = caliper1;
                obj2{count} = caliper2;
            elseif isParallel1
                % caliper 1 is parallel to a side
                distv(count) = distToPoint(caliper1,c2);
                obj1{count} = caliper1;
                obj2{count} = c2;
            elseif isParallel2
                % caliper 2 is parallel to a side
                distv(count) = distToPoint(caliper2,c1);
                obj1{count} = c1;
                obj2{count} = caliper2;
            else
                % neither caliper is parallel to a side
                distv(count) = length(line2d(c1,c2));
                obj1{count} = c1;
                obj2{count} = c2;
            end
            % keep track of relative location of caliper 2 to caliper 1
            sideofv(count) = side(caliper1,caliper2.point1);
            if show
                NewFigure('box2d: distance');
                plot(box1)
                plot(caliper1,'r')
                hold on
                plot(box2)
                plot(caliper2,'r')
%                 pause(2)
            end
            % begin loop to rotate calipers around boxes
            while true
                count = count + 1;
                % find the next side encountered by rotating caliper CW
                % assume the included angle is acute
                % and calculate the distance between calipers
                % as well as the relative location
                ang1 = line2d.angleIncludedDeg(side1,caliper1);
                ang2 = line2d.angleIncludedDeg(side2,caliper2);
                if abs(ang1-ang2)<box2d.TOL % calipers are on parallel sides
                    if show, disp('parallel'), end
                    if ang1<box2d.TOL % caliper is on a side
                        % update side 1
                        Ic1 = Ic1 + 1;
                        if Ic1>4
                            Ic1 = 1;
                        end
                        side1 = box1.getSide(Ic1);
                        % update side 2
                        Ic2 = Ic2 + 1;
                        if Ic2>4
                            Ic2 = 1;
                        end
                        side2 = box2.getSide(Ic2);
                    end
                    % move calipers to current sides
                    caliper1 = side1;
                    caliper2 = side2;
                    % find distance between parallel segments
                    dist = line2d.distSegmentToSegmentParallel(caliper1,caliper2);
                    obj1{count} = caliper1;
                    obj2{count} = caliper2;
                elseif ang1<box2d.TOL % caliper 1 is on a side
                    if show, disp('caliper 1 on a side, move caliper 2 to next side'), end
                    caliper1 = caliper1.flipDirection.rotateDeg(180-ang2);
                    caliper2 = side2;
                    dist = distToPoint(caliper2,caliper1.point1);
                    obj1{count} = caliper1.point1;
                    obj2{count} = caliper2;
                    % update side 1
                    Ic1 = Ic1 + 1;
                    if Ic1>4
                        Ic1 = 1;
                    end
                    side1 = box1.getSide(Ic1);
                elseif ang2<box2d.TOL % caliper 2 is on a side
                    if show, disp('caliper 2 on a side, move caliper 1 to next side'), end
                    caliper1 = side1;
                    caliper2 = caliper2.flipDirection.rotateDeg(180-ang1);
                    dist = distToPoint(caliper1,caliper2.point1);
                    obj1{count} = caliper1;
                    obj2{count} = caliper2.point1;
                    % update side 2
                    Ic2 = Ic2 + 1;
                    if Ic2>4
                        Ic2 = 1;
                    end
                    side2 = box2.getSide(Ic2);
                elseif ang1<ang2 % caliper 1 is closer to next side
                    if show, disp('caliper 1 next'), end
                    caliper1 = side1;
                    caliper2 = caliper2.rotateDeg(-ang1);
                    dist = distToPoint(caliper1,caliper2.point1);
                    obj1{count} = caliper1;
                    obj2{count} = caliper2.point1;
                elseif ang1>ang2 % caliper 2 is closer to next side
                    if show, disp('caliper 2 next'), end
                    caliper1 = caliper1.rotateDeg(-ang2);
                    caliper2 = side2;
                    dist = distToPoint(caliper2,caliper1.point1);
                    obj1{count} = caliper1.point1;
                    obj2{count} = caliper2;
                else % whoops, unexpected condition!
                    keyboard
                end
                % plot boxes if show is true
                if show
                    NewFigure('box2d: distance');
                    plot(box1)
                    plot(caliper1,'r')
                    hold on
                    plot(box2)
                    plot(caliper2,'r')
%                     pause(2)
                end
                % calipers should always be parallel to each other
                if ~line2d.parallel(caliper1,caliper2)                   
                    error('box2d: calipers not parallel!')
                end
                % add distance to its vector
                distv(count) = dist;
                % add relaive location of caliper 2 to its vector
                sideofv(count) = side(caliper1,caliper2.point1);
                % test for passing initial conditions
                if line2d.equals(side1,side1_init) && length(distv(~isnan(distv)))>4
                    if show, disp('side 1 match. done'), end
                    break
                end
                if line2d.equals(side2,side2_init) && length(distv(~isnan(distv)))>4
                    if show, disp('side 2 match. done'), end
                    break
                end
            end
            % test for collision between the boxes
            nz = sideofv(~isnan(sideofv) & sideofv~=0);
            if all(nz<0) || all(nz>0) % collision
                collision = true;
            else % no collision
                collision = false;
            end
            % return the minimun distance
            % assume that one box is not fully
            % enclosed by the other
            if collision
                [dist,Idist] = min(fliplr(distv));
                Idist = length(distv)-Idist+1;
                dist = 0;
            else
                [dist,Idist] = min(fliplr(distv));
                Idist = length(distv)-Idist+1;
            end
            % return the objects (point or line) that bound the distance
            ends = {obj1{Idist} obj2{Idist}};
            % plot the distance line if show is true
            if show
                isLine1 = isa(ends{1},'line2d');
                isLine2 = isa(ends{2},'line2d');
                if isLine1 && isLine2
                    [dist,end1,end2] = line2d.distSegmentToSegmentParallel(ends{1},ends{2});
                    if all(isnan(end1)) && all(isnan(end2))
                        dline = intersectionPerp(ends{2},ends{1}.point1);
                    elseif all(isnan(end1))
                        dline = intersectionPerp(ends{1},end2);
                        dline = dline.flipDirection;
                    elseif all(isnan(end2))
                        dline = intersectionPerp(ends{2},end1);
                    else
                        dline = line2d(end1,end2);
                    end
                elseif isLine1
                    [dist,end1] = distToPoint(ends{1},ends{2}); % ends{2} is a point
                    if all(isnan(end1))
                        dline = intersectionPerp(ends{1},ends{2});
                        dline = dline.flipDirection;
                    else
                        dline = line2d(end1,ends{2});
                    end
                elseif isLine2
                    [dist,end2] = distToPoint(ends{2},ends{1}); % ends{1} is a point
                    if all(isnan(end2))
                        dline = intersectionPerp(ends{2},ends{1});
                    else
                        dline = line2d(ends{1},end2);
                    end
                else
                    dline = line2d(ends{1},ends{2});
                end
                plot(dline,'k')
%                 pause(5)
            end
        end
        
        % find critical support lines between two box2ds
        function csl = criticalSupportLines(box1,box2,show)
            if ~isa(box1,'box2d') || ~isa(box2,'box2d')
                error('must supply box2d objects as input')
            end
            if nargin<3
                show = false;
            end
            box2d.TOL = 0.03; % testing for parallel sides needs a tolerance
            count = 1; % initialize count
            % initialize caliper on first box at minimum Y point
            [c1,Ic1] = minY(box1);
            Ic1prev = Ic1 - 1;
            if Ic1prev<1, Ic1prev=4; end
            Ic1next = Ic1 + 1;
            if Ic1next>4, Ic1next=1; end
            side1 = box1.getSide(Ic1);
            side1_init = side1;
            caliper1 = line2d(c1,c1-[1 0],'segment');
            % initialize caliper on second box at maximum Y point
            [c2,Ic2] = maxY(box2);
            Ic2prev = Ic2 - 1;
            if Ic2prev<1, Ic2prev=4; end
            Ic2next = Ic2 + 1;
            if Ic2next>4, Ic2next=1; end
            side2 = box2.getSide(Ic2);
            side2_init = side2;
            caliper2 = line2d(c2,c2+[1 0],'segment');
            % initialize line between calipers and test for csl
            csl = [];
            line = line2d(c1,c2,'line');
            s1 = [side(line,box1.getCorner(Ic1prev)) side(line,box1.getCorner(Ic1next))];
            s2 = [side(line,box2.getCorner(Ic2prev)) side(line,box2.getCorner(Ic2next))];
            if (s1(1)*s1(2)>0) && (s2(1)*s2(2)>0) && (s1(1)*s2(1)<0)
                csl = [csl line];
            end
            % plot the box if show is true
            if show
                NewFigure('box2d: csl');
                plot(box1)
                plot(caliper1,'r')
                hold on
                plot(box2)
                plot(caliper2,'r')
%                 pause(2)
            end
            % begin loop to rotate calipers around boxes
            while true
                count = count + 1;
                % find the next side encountered by rotating caliper CW
                % assume the included angle is acute
                % and calculate the distance between calipers
                % as well as the relative location
                ang1 = line2d.angleIncludedDeg(side1,caliper1);
                ang2 = line2d.angleIncludedDeg(side2,caliper2);
                if abs(ang1-ang2)<box2d.TOL % calipers are on parallel sides
                    if show, disp('parallel'), end
                    if ang1<box2d.TOL % caliper is on a side
                        % update side 1
                        Ic1 = Ic1 + 1;
                        if Ic1>4
                            Ic1 = 1;
                        end
                        side1 = box1.getSide(Ic1);
                        % update side 2
                        Ic2 = Ic2 + 1;
                        if Ic2>4
                            Ic2 = 1;
                        end
                        side2 = box2.getSide(Ic2);
                    end
                    % move calipers to current sides
                    caliper1 = side1;
                    caliper2 = side2;
                elseif ang1<box2d.TOL % caliper 1 is on a side
                    if show, disp('caliper 1 on a side, move caliper 2 to next side'), end
                    caliper1 = caliper1.flipDirection.rotateDeg(180-ang2);
                    caliper2 = side2;
                    % update side 1
                    Ic1 = Ic1 + 1;
                    if Ic1>4
                        Ic1 = 1;
                    end
                    side1 = box1.getSide(Ic1);
                elseif ang2<box2d.TOL % caliper 2 is on a side
                    if show, disp('caliper 2 on a side, move caliper 1 to next side'), end
                    caliper1 = side1;
                    caliper2 = caliper2.flipDirection.rotateDeg(180-ang1);
                    % update side 2
                    Ic2 = Ic2 + 1;
                    if Ic2>4
                        Ic2 = 1;
                    end
                    side2 = box2.getSide(Ic2);
                elseif ang1<ang2 % caliper 1 is closer to next side
                    if show, disp('caliper 1 next'), end
                    caliper1 = side1;
                    caliper2 = caliper2.rotateDeg(-ang1);
                elseif ang1>ang2 % caliper 2 is closer to next side
                    if show, disp('caliper 2 next'), end
                    caliper1 = caliper1.rotateDeg(-ang2);
                    caliper2 = side2;
                else % whoops, unexpected condition!
                    keyboard
                end
                % update prev & next corners, line, and test for csl
                Ic1prev = Ic1 - 1;
                if Ic1prev<1, Ic1prev=4; end
                Ic1next = Ic1 + 1;
                if Ic1next>4, Ic1next=1; end
                Ic2prev = Ic2 - 1;
                if Ic2prev<1, Ic2prev=4; end
                Ic2next = Ic2 + 1;
                if Ic2next>4, Ic2next=1; end
                line = line2d(box1.getCorner(Ic1),box2.getCorner(Ic2),'line');
                s1 = [side(line,box1.getCorner(Ic1prev)) side(line,box1.getCorner(Ic1next))];
                s2 = [side(line,box2.getCorner(Ic2prev)) side(line,box2.getCorner(Ic2next))];
                if (s1(1)*s1(2)>0) && (s2(1)*s2(2)>0) && (s1(1)*s2(1)<0)
                    if max(size(csl))==1
                        if ~line2d.parallel(csl(1),line)
                            csl = [csl line];
                        end
                    else
                        csl = [csl line];
                    end
                end
                % plot boxes if show is true
                if show
                    NewFigure('box2d: csl');
                    plot(box1)
                    plot(caliper1,'r')
                    hold on
                    plot(box2)
                    plot(caliper2,'r')
                    plot(line,'y')
                    try
                    for i = 1:max(size(csl))
                        plot(csl(i),'c')
                    end
                    catch
                        keyboard
                    end
%                     pause(2)
                end
                % calipers should always be parallel to each other
                if ~line2d.parallel(caliper1,caliper2)
                    error('box2d: calipers not parallel!')
                end
                % test for passing initial conditions
                if max(size(csl))==2
                    if show, disp('two critical support lines found. done.'), end
                    break
                end
            end
        end        
    end
end