classdef caliper
    properties
        box
        cal
        index
        done
    end
    
    properties (Hidden)
        index0
        parallel
        top
    end
    
    properties (Hidden, Dependent)
        corner
        prevcorner
        nextcorner
        side
        prevside
        nextAngle
    end
    
    properties (Constant)
        TOL = 0.03;
    end
    
    methods
        function obj = caliper(box,top)
            if nargin>0
                obj.box = box;
                obj.top = top;
                if nargin<2
                    top = false;
                end
                obj = initialize(obj,top);
            else
                % empty object
            end
        end
        
        function obj = initialize(obj,top)
            % update the box if it hasn't been done yet
            if isempty(obj.box.c1)
                obj.box = obj.box.update;
            end
            if top
                [c,Ic] = maxY(obj.box);
                obj.cal = line2d(c,[c(:,1)+1 c(:,2)],'segment');
            else % bottom
                [c,Ic] = minY(obj.box);
                obj.cal = line2d(c,[c(:,1)-1 c(:,2)],'segment');
            end
            obj.index = Ic;
            isParallel = line2d.parallel(obj.cal,obj.side);
            obj.parallel = isParallel;
            
            obj.index(isParallel) = obj.index(isParallel) + 1;
            obj.index(obj.index>4) = 1;
            c = obj.box.getCorner(obj.index);
            if top
                obj.cal = line2d(c,[c(:,1)+1 c(:,2)],'segment');
            else % bottom
                obj.cal = line2d(c,[c(:,1)-1 c(:,2)],'segment');
            end
            
%             sides = obj.side.flipDirection(isParallel).rotateDeg(180,isParallel);
%             obj.index(isParallel) = obj.index(isParallel) + 1;
%             obj = obj.assignCal(sides,isParallel);
            obj.index0 = obj.index;
            obj.done = false(size(obj.index));
        end
            
        % get a subset of the object
        function one = getone(obj,idx)
            if idx>obj.cal.count;
                error('index greater than object count')
            end
            one = caliper(obj.box.getone(idx),obj.top);
        end
        
        function obj = rotateNext(obj,selection)
            if nargin<2
                selection = true(obj.cal.count,1);
            end
            sides = obj.side.flipDirection.rotateDeg(180);
            obj.parallel(selection) = true;
            obj.index(selection) = obj.index(selection) + 1;
            obj.index(obj.index>4) = 1;
            obj = obj.assignCal(sides,selection);
            obj.done(selection) = ...
                obj.done(selection) | ...
                obj.index(selection)== ...
                obj.index0(selection);
        end
        
        function obj = rotateRad(obj,angleRad,selection)
            if nargin<3
                selection = true(obj.cal.count,1);
            end
            obj.cal = obj.rotateDeg(obj,angleRad*180/pi,selection);
        end
        
        function obj = rotateDeg(obj,angleDeg,selection)
            if nargin<3
                selection = true(obj.cal.count,1);
            end
            if any(angleDeg(selection)<(-obj.nextAngle(selection)-caliper.TOL))
                warning('you are rotating the caliper past the next side')
            end
            obj.cal = obj.cal.rotateDeg(angleDeg,selection);
            obj.parallel(selection) = false;
        end
        
        function obj = assignCal(obj,lines,selection)
            if nargin<3
                selection = true(obj.cal.count,1);
            end
            obj.cal.point1(selection,:) = lines.point1(selection,:);
            obj.cal.point2(selection,:) = lines.point2(selection,:);
        end
        
        function corner = get.corner(obj)
            corner = obj.box.getCorner(obj.index);
        end
        
        function prevcorner = get.prevcorner(obj)
            index = obj.index - 1;
            index(index<1) = 4;
            prevcorner = obj.box.getCorner(index);
        end
        
        function nextcorner = get.nextcorner(obj)
            index = obj.index + 1;
            index(index>4) = 1;
            nextcorner = obj.box.getCorner(index);
        end
        
        function side = get.side(obj)
            side = obj.box.getSide(obj.index);
        end
                
        function prevside = get.prevside(obj)
            index = obj.index - 1;
            index(index<1) = 4;
            prevside = obj.box.getSide(index);
        end
        
        function nextAngle = get.nextAngle(obj)
            nextAngle = line2d.angleIncludedDeg(obj.side,obj.cal);
        end
        
        function plot(obj,varargin)
            plot(obj.box,varargin{:})
            hold on
            plot(obj.cal,varargin{:})
        end
        
        function plotn(obj,n,varargin)
            plotn(obj.box,n,varargin{:})
            hold on
            plotn(obj.cal,n,varargin{:})
        end
        
        function ploti(obj,i,varargin)
            ploti(obj.box,i,varargin{:})
            hold on
            ploti(obj.cal,i,varargin{:})
        end
    end
end