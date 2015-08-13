classdef mover2d < box2d
    properties
        velocity
    end
    
    properties (Dependent)
        velline
    end
    
    methods
        function obj = mover2d(length,width,refoffset)
            if nargin<3
                refoffset = [0 0];
            end
            obj = obj@box2d(length,width,refoffset);
        end
             
        function velline = get.velline(obj)
            v = line2d([0 0],[obj.velocity zeros(size(obj.velocity))],'segment');
            velline = v.rotateDeg(obj.angleDeg).translate(obj.reference);
        end
                
        % set the velocity of the mover.  it may be an array, but it needs 
        % to have the same dimension as the reference and angle of the
        % box2d superclass
        function obj = setvelocity(obj,vel)
            if isempty(obj.reference) && isempty(obj.angleDeg)
                error('set reference or angle before setting velocity')
            end
            if size(vel,2)>1
                error('only velocity magnitude is required here')
            end
            obj.velocity = vel;
            % the size of the velocity must be the same as the size
            % of the reference and/or the angle.  all can be arrays
            n1 = size(vel,1);
            if ~isempty(obj.reference)
                n2 = size(obj.reference,1);
            else
                n2 = size(obj.angleDeg,1);
            end
            if n1==n2
                return
            end
            if n1>1 && n2>1
                error('dimension of velocity and reference/angleDeg incompatible')
            end
            if n1==1
                obj.velocity = repmat(vel,n2,1);
            else
                if size(obj.angleDeg,1)==1
                    obj.angleDeg = repmat(obj.angleDeg,n1,1);
                end
                if size(obj.reference,1)==1
                    obj.reference = repmat(obj.reference,n1,1);
                end
            end
        end
        
        % get a subset of the object
        function one = getone(obj,idx)
            if idx>size(obj.reference,1)
                error('index greater than object count')
            end
            one = mover2d(obj.length,obj.width,obj.refoffset);
            one = one.setreference(obj.reference(idx,:));
            one = one.setangleDeg(obj.angleDeg(idx,:));
            one = one.setvelocity(obj.velocity(idx,:));
            one = one.update;
        end
        
        % predict the mover2d's location after some time, where time is
        % interpreted as the parameter in the parametric line equation.
        % this interpretation is true as long as the length of the line is
        % the magnitude of the velocity
        function obj = predict(obj,time)
            velline = obj.velline;
            point = velline.getPoint(time);
            offset = point - velline.point1;
            obj = obj.translate(offset);
        end
        
        % plot a mover2d
        function plot(obj,varargin)
            plot@box2d(obj,varargin{:});
            hold on
            plot(obj.velline,varargin{:})
        end
        
        function plotn(obj,n,varargin)
            plotn@box2d(obj,n,varargin{:});
            hold on
            plotn(obj.velline,n,varargin{:})
        end
    end
end