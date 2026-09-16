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
        
        function [obji,distTrav,distProjected] = projectVelocity(obj,Fs,time)
            % if there are any non-finite values (NaN, Inf) in the object,
            % it will mess up the interpolation done for distProj below, so
            % we must fill them in with interpolated values
            if ~all(isfinite(obj.velocity))
                isValid = isfinite(obj.velocity);
                f = (1:length(obj.velocity))';
                obj.velocity = interp1(f(isValid),obj.velocity(isValid),f,'linear','extrap');
                obj.velocity(obj.velocity<0) = 0;
                obj.reference(:,1) = interp1(f(isValid),obj.reference(isValid,1),f,'linear','extrap');
                obj.reference(:,2) = interp1(f(isValid),obj.reference(isValid,2),f,'linear','extrap');
                obj.angleDeg = interp1(f(isValid),obj.angleDeg(isValid),f,'linear','extrap');
                obj = obj.update;
            end
            % calculate projection to future distance assuming constant
            % velocity from its value at each time. results in a projected
            % distance vector as some future time
            velocity = obj.velocity;
            distTrav = cumsum(velocity)/Fs;
            % if time is a scalar, then use it for all calculations in the
            % segment, but if it is a vector then it should be the same
            % length as the segment and possibly different times are
            % projected for each element of the vector
            if isscalar(time)
                distProj = distTrav + time*velocity;
            elseif length(time)==length(obj.velocity)
                distProj = distTrav + time.*velocity;
            else
                error('dimensions of time argument are not compatible')
            end
            % the projected distance is not allowed to extend beyond the
            % distance travelled because we don't have data out that far
            isOver = distProj>distTrav(end);
            isUnder = distProj<distTrav(1);
            isOut = isOver | isUnder;
            distProj(isOut) = [];
            vel = velocity(~isOut);
            try
                % use interpolation to map the current distance travelled to
                % the projected distance travelled
                refxi = interp1(distTrav,obj.reference(:,1),distProj,'linear','extrap');
                refyi = interp1(distTrav,obj.reference(:,2),distProj,'linear','extrap');
                angledegi = interp1(distTrav,obj.angleDeg,distProj,'linear','extrap');
            catch err
                % we can fix the error that stems from not having a
                % monotonic 'xi' variable
                if ~isempty(strfind(err.message,'monotonic'))
                    % unique and sorted version of distTrav
                    [distTravUnique,IAtrav,ICtrav] = unique(distTrav,'stable');
                    velocity = velocity(IAtrav);
                    [distTravSort,Is] = unique(distTrav);
                    % unique and sorted version of distProj
                    [distProjUnique,IAproj,ICproj] = unique(distProj,'stable');
                    [distProjSort,Isort] = sort(distProjUnique);
                    I = (1:length(distProjUnique))';
                    IunsortProj(Isort) = I; % find the reverse sort indices
                    % use interpolation to map the current distance travelled to
                    % the projected distance travelled
                    refxi = interp1(distTrav(Is),obj.reference(Is,1),distProjSort,'linear','extrap');
                    refxi = refxi(IunsortProj); % undo sort
                    refxi = refxi(ICproj); % undo unique
                    refyi = interp1(distTrav(Is),obj.reference(Is,2),distProjSort,'linear','extrap');
                    refyi = refyi(IunsortProj); % undo sort
                    refyi = refyi(ICproj); % undo unique
                    angledegi = interp1(distTrav(Is),obj.angleDeg(Is),distProjSort,'linear','extrap');
                    angledegi = angledegi(IunsortProj); % undo sort
                    angledegi = angledegi(ICproj); % undo unique
                else
                    keyboard
                end
            end
            % filter the object variables
            [B,A] = butter(2,0.5);
            if length(refxi)>6
                refxif = filtfilt(B,A,double(refxi));
                refyif = filtfilt(B,A,double(refyi));
                angledegif = filtfilt(B,A,double(angledegi));
            else
                refxif = refxi;
                refyif = refyi;
                angledegif = angledegi;
            end
            % build the full variables
            distProjected = NaN(size(distTrav));
            distProjected(~isOut) = distProj;
            velocity = NaN(size(distTrav));
            velocity(~isOut) = vel;
            reference = NaN(size(distTrav,1),2);
            reference(~isOut,:) = [refxif refyif];
            angleDeg = NaN(size(distTrav));
            angleDeg(~isOut) = angledegif;
            % create new object at projected distances
            obji = mover2d(obj.length,obj.width,obj.refoffset);
            obji = obji.setreference(reference);
            obji = obji.setangleDeg(angleDeg);
            obji = obji.setvelocity(velocity);
            obji = obji.update;
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
        
        % plot n points of a mover2d
        function plotn(obj,n,varargin)
            plotn@box2d(obj,n,varargin{:});
            hold on
            plotn(obj.velline,n,varargin{:})
        end
        
        % plot a custom set of points of a mover2d
        function ploti(obj,i,varargin)
            ploti@box2d(obj,i,varargin{:});
            hold on
            ploti(obj.velline,i,varargin{:})
        end
    end
end