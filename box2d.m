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
        length
        width
        refoffset
        reference
        angleDeg
    end
            
    properties (Dependent)
        center
        angleRad
    end
    
    properties (Hidden)
        % hypotenuse
        hypo1
        hypo2
        hypo3
        hypo4
        % corners
        c1
        c2
        c3
        c4
        % sides
        side12
        side23
        side34
        side41
    end
        
    properties (Constant, Hidden)
        TOL = 0.03;
    end
    
    % class methods
    methods
        % box2d constructor requires length and width parameters.  an
        % optional argument is the offset from the center of the box to the
        % refernce point
        function obj = box2d(length,width,refoffset)
            if nargin>0
                obj.length = length;
                obj.width = width;
                if nargin<3
                    obj.refoffset = [0 0];
                else
                    obj.refoffset = refoffset;
                end
            else
                % empty object
            end
        end

        % set the reference point of the box.  it may be at the center or
        % offset from the center, depending on the value of the refoffset.
        % the angle and reference point may both be arrays, but they need 
        % to have the same dimension
        function obj = setreference(obj,point)
            obj.reference = point;
            if isempty(obj.angleDeg)
                return
            end
            % the size of the reference point must be the same as the size
            % of the angle.  both can be arrays
            n1 = size(point,1);
            n2 = size(obj.angleDeg,1);
            if n1==n2
                return
            end
            if n1>1 && n2>1
                error('number of reference points and angles incompatible')
            end
            if n1==1
                obj.reference = repmat(point,n2,1);
            elseif n2==1
                obj.angleDeg = repmat(obj.angleDeg,n1,1);
            end
        end
        
        % set the orientation of the box in radians.  the angle and
        % reference point may both be arrays, but they need to have the
        % same dimension
        function obj = setangleDeg(obj,angleDeg)
            obj.angleDeg = angleDeg;
            if isempty(obj.reference)
                return
            end
            % the size of the angle must be the same as the size of the
            % reference point.  both can be arrays
            n1 = size(angleDeg,1);
            n2 = size(obj.reference,1);
            if n1==n2
                return
            end
            if n1>1 && n2>1
                error('number of angles and reference points incompatible')
            end
            if n1==1
                obj.angleDeg = repmat(angleDeg,n2,1);
            elseif n2==1
                obj.reference = repmat(obj.reference,n1,1);
            end
        end
        
        % get a subset of the object
        function one = getone(obj,idx)
            if idx>size(obj.reference,1)
                error('index greater than object count')
            end
            one = box2d(obj.length,obj.width,obj.refoffset);
            one = one.setreference(obj.reference(idx,:));
            one = one.setangleDeg(obj.angleDeg(idx,:));
            one = one.update;
        end
        
        % translate the box by some offset to a new location
        % if update has been called, it will need to be re-called after
        % this
        function obj = translate(obj,offset)
            obj.reference(:,1) = obj.reference(:,1) + offset(:,1);
            obj.reference(:,2) = obj.reference(:,2) + offset(:,2);
        end
        
        % dependent getter for the center property.  The center location
        % depends on the reference point and the refoffset property
        function center = get.center(obj)
            if isempty(obj.angleDeg)
                center(:,1) = obj.reference(:,1) - obj.refoffset(:,1);
                center(:,2) = obj.reference(:,2) - obj.refoffset(:,2);
            else
                center(:,1) = obj.reference(:,1) - ...
                    obj.refoffset(:,1).*cos(obj.angleRad) + ...
                    obj.refoffset(:,2).*sin(obj.angleRad);
                center(:,2) = obj.reference(:,2) - ...
                    obj.refoffset(:,2).*cos(obj.angleRad) - ...
                    obj.refoffset(:,1).*sin(obj.angleRad);
            end
        end
        
        % dependent getter for the angle in radians
        function angleRad = get.angleRad(obj)
            angleRad = obj.angleDeg*pi/180;
        end
        
        function obj = update(obj)
            obj = obj.setHypotenuses;
            obj = obj.setCorners;
            obj = obj.setSides;
        end
        
        function obj = setHypotenuses(obj)
            % hypotenuse 1
            x = obj.length/2 - obj.refoffset(1);
            y = -obj.width/2 - obj.refoffset(2);
            hypo1 = line2d(zeros(size(obj.reference)),[x y],'segment');
            hypo1 = hypo1.rotateDeg(obj.angleDeg);
            obj.hypo1 = hypo1.translate(obj.reference);
            % hypotenuse 2
            x = -obj.length/2 - obj.refoffset(1);
            y = -obj.width/2 - obj.refoffset(2);
            hypo2 = line2d(zeros(size(obj.reference)),[x y],'segment');
            hypo2 = hypo2.rotateDeg(obj.angleDeg);
            obj.hypo2 = hypo2.translate(obj.reference);
            % hypotenuse 3
            x = -obj.length/2 - obj.refoffset(1);
            y = obj.width/2 - obj.refoffset(2);
            hypo3 = line2d(zeros(size(obj.reference)),[x y],'segment');
            hypo3 = hypo3.rotateDeg(obj.angleDeg);
            obj.hypo3 = hypo3.translate(obj.reference);
            % hypotenuse 4
            x = obj.length/2 - obj.refoffset(1);
            y = obj.width/2 - obj.refoffset(2);
            hypo4 = line2d(zeros(size(obj.reference)),[x y],'segment');
            hypo4 = hypo4.rotateDeg(obj.angleDeg);
            obj.hypo4 = hypo4.translate(obj.reference);
        end
                
        function obj = setCorners(obj)
            obj.c1 = obj.hypo1.point2;
            obj.c2 = obj.hypo2.point2;
            obj.c3 = obj.hypo3.point2;
            obj.c4 = obj.hypo4.point2;
        end
        
        function obj = setSides(obj)
            obj.side12 = line2d(obj.c1,obj.c2,'segment');
            obj.side23 = line2d(obj.c2,obj.c3,'segment');
            obj.side34 = line2d(obj.c3,obj.c4,'segment');
            obj.side41 = line2d(obj.c4,obj.c1,'segment');
        end
        
        % find the corner with the minimum X coordinate. return the corner
        % as well as the index of the corner (1-4)
        function [c,Ic] = minX(obj)
            x = [obj.c1(:,1) obj.c2(:,1) obj.c3(:,1) obj.c4(:,1)];
            y = [obj.c1(:,2) obj.c2(:,2) obj.c3(:,2) obj.c4(:,2)];
            [xmin,Ic] = min(x,[],2);
            rows = (1:size(x,1))';
            linearInd = sub2ind(size(x),rows,Ic);
            c = [x(linearInd) y(linearInd)];
        end
        
        % find the corner with the maximum X coordinate. return the corner
        % as well as the index of the corner (1-4)
        function [c,Ic] = maxX(obj)
            x = [obj.c1(:,1) obj.c2(:,1) obj.c3(:,1) obj.c4(:,1)];
            y = [obj.c1(:,2) obj.c2(:,2) obj.c3(:,2) obj.c4(:,2)];
            [xmin,Ic] = max(x,[],2);
            rows = (1:size(x,1))';
            linearInd = sub2ind(size(x),rows,Ic);
            c = [x(linearInd) y(linearInd)];
        end
        
        % find the corner with the minimum Y coordinate. return the corner
        % as well as the index of the corner (1-4)
        function [c,Ic] = minY(obj)
            x = [obj.c1(:,1) obj.c2(:,1) obj.c3(:,1) obj.c4(:,1)];
            y = [obj.c1(:,2) obj.c2(:,2) obj.c3(:,2) obj.c4(:,2)];
            [xmin,Ic] = min(y,[],2);
            rows = (1:size(x,1))';
            linearInd = sub2ind(size(x),rows,Ic);
            c = [x(linearInd) y(linearInd)];
        end
        
        % find the corner with the maximum Y coordinate. return the corner
        % as well as the index of the corner (1-4)
        function [c,Ic] = maxY(obj)
            x = [obj.c1(:,1) obj.c2(:,1) obj.c3(:,1) obj.c4(:,1)];
            y = [obj.c1(:,2) obj.c2(:,2) obj.c3(:,2) obj.c4(:,2)];
            [xmin,Ic] = max(y,[],2);
            rows = (1:size(x,1))';
            linearInd = sub2ind(size(x),rows,Ic);
            c = [x(linearInd) y(linearInd)];
        end
        
        % get a corner of the box given its index
        function c = getCorner(obj,Ic)
            if any(Ic>4)
                error('only 4 corners in a box2d')
            end
            % if input index is scalar, upgrade it to a vector if needed
            if isscalar(Ic)
                Ic = repmat(Ic,size(obj.reference(:,1)));
            end
            x = [obj.c1(:,1) obj.c2(:,1) obj.c3(:,1) obj.c4(:,1)];
            y = [obj.c1(:,2) obj.c2(:,2) obj.c3(:,2) obj.c4(:,2)];
            if length(Ic)==1
                Ic = repmat(Ic,size(x,1),1);
            end
            rows = (1:size(x,1))';
            linearInd = sub2ind(size(x),rows,Ic);
            c = [x(linearInd) y(linearInd)];
        end
        
        % get a side of the box given its index
        function side = getSide(obj,Is)
            if any(Is>4)
                error('only 4 sides in a box2d')
            end
            % if input index is scalar, upgrade it to a vector if needed
            if isscalar(Is)
                Is = repmat(Is,size(obj.reference(:,1)));
            end
            % first point of line
            x = [obj.side12.point1(:,1) obj.side23.point1(:,1) obj.side34.point1(:,1) obj.side41.point1(:,1)];
            y = [obj.side12.point1(:,2) obj.side23.point1(:,2) obj.side34.point1(:,2) obj.side41.point1(:,2)];
            rows = (1:size(x,1))';
            linearInd = sub2ind(size(x),rows,Is);
            point1 = [x(linearInd) y(linearInd)];
            % second point of line
            x = [obj.side12.point2(:,1) obj.side23.point2(:,1) obj.side34.point2(:,1) obj.side41.point2(:,1)];
            y = [obj.side12.point2(:,2) obj.side23.point2(:,2) obj.side34.point2(:,2) obj.side41.point2(:,2)];
            rows = (1:size(x,1))';
            linearInd = sub2ind(size(x),rows,Is);
            point2 = [x(linearInd) y(linearInd)];
            % reconstruct the selected sides
            side = line2d(point1,point2,'segment');
        end
                
        % find the smallest distance from a box2d to an arbitrary point.
        % the function does not check to see whether the point is inside
        % the box
        function dist = distToPoint(obj,point)
            dist12 = obj.side12.distToPoint(point);
            dist23 = obj.side23.distToPoint(point);
            dist34 = obj.side34.distToPoint(point);
            dist41 = obj.side41.distToPoint(point);
            [dist,Imin] = min([dist12 dist23 dist34 dist41],[],2);
        end
        
        % plot a box2d
        function plot(obj,varargin)
            n1 = size(obj.reference,1);
            if n1==1
                X = [obj.c1(1) obj.c2(1) obj.c3(1) obj.c4(1) obj.c1(1)]';
                Y = [obj.c1(2) obj.c2(2) obj.c3(2) obj.c4(2) obj.c1(2)]';
                plot(X,Y,varargin{:})
                hold on
                plot(obj.center(1),obj.center(2),'o')
                plot(obj.reference(1),obj.reference(2),'s')
                plot(obj.c1(1),obj.c1(2),'*')
                axis equal
            else
                hold on
                for idx = 1:n1
                    X = [obj.c1(idx,1) obj.c2(idx,1) obj.c3(idx,1) obj.c4(idx,1) obj.c1(idx,1)]';
                    Y = [obj.c1(idx,2) obj.c2(idx,2) obj.c3(idx,2) obj.c4(idx,2) obj.c1(idx,2)]';
                    plot(X,Y,varargin{:})
                    hold on
                    plot(obj.center(idx,1),obj.center(idx,2),'o')
                    plot(obj.reference(idx,1),obj.reference(idx,2),'s')
                    plot(obj.c1(idx,1),obj.c1(idx,2),'*')
                    axis equal
                end
            end
        end
        
        function plotn(obj,n,varargin)
            n1 = size(obj.reference,1);
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
                X = [obj.c1(idx(i),1) obj.c2(idx(i),1) obj.c3(idx(i),1) obj.c4(idx(i),1) obj.c1(idx(i),1)]';
                Y = [obj.c1(idx(i),2) obj.c2(idx(i),2) obj.c3(idx(i),2) obj.c4(idx(i),2) obj.c1(idx(i),2)]';
                plot(X,Y,varargin{:})
                hold on
                plot(obj.center(idx(i),1),obj.center(idx(i),2),'o')
                plot(obj.reference(idx(i),1),obj.reference(idx(i),2),'s')
                plot(obj.c1(idx(i),1),obj.c1(idx(i),2),'*')
                axis equal
            end
        end
        
        function ploti(obj,i,varargin)
            n1 = size(obj.reference,1);
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
                X = [obj.c1(idx(i),1) obj.c2(idx(i),1) obj.c3(idx(i),1) obj.c4(idx(i),1) obj.c1(idx(i),1)]';
                Y = [obj.c1(idx(i),2) obj.c2(idx(i),2) obj.c3(idx(i),2) obj.c4(idx(i),2) obj.c1(idx(i),2)]';
                plot(X,Y,varargin{:})
                hold on
                plot(obj.center(idx(i),1),obj.center(idx(i),2),'o')
                plot(obj.reference(idx(i),1),obj.reference(idx(i),2),'s')
                plot(obj.c1(idx(i),1),obj.c1(idx(i),2),'*')
                axis equal
            end
        end        
    end
end
