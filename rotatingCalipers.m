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

% a class that represents the rotating calipers and some of the operations one can
% do with them.
classdef rotatingCalipers
    properties
        cal1
        cal2
    end
    
    properties (Dependent)
        done
    end
    
    properties (Hidden)
        distmat
        sideofmat
        x1mat
        x2mat
        y1mat
        y2mat
        csl1
        csl2
        bridge1
        bridge2
    end
    
    properties (Constant)
        TOL = 0.01;
    end
    
    methods
        function obj = rotatingCalipers(box1,box2)
            if nargin>0
                obj.cal1 = caliper(box1,false); % start on bottom
                obj.cal2 = caliper(box2,true); % start on top
            else
                % empty object
            end
        end
        
        function done = get.done(obj)
            nans = isnan(obj.cal1.cal.point1(:,1)) | isnan(obj.cal2.cal.point1(:,1));
            done = all(obj.cal1.done(~nans)) && all(obj.cal2.done(~nans));
        end
        
        % get a subset of the object
        function one = getone(obj,idx)
            if idx>obj.cal1.cal.count;
                error('index greater than object count')
            end
            one = rotatingCalipers(obj.cal1.box.getone(idx),obj.cal2.box.getone(idx));
        end
        
        function obj = rotateNext(obj)
            ang1 = obj.cal1.nextAngle;
            ang2 = obj.cal2.nextAngle;
            isSame = abs(ang1-ang2)<rotatingCalipers.TOL;
            isLess = ang1<ang2 & ~isSame;
            isGreater = ang1>ang2 & ~isSame;
            obj.cal1 = obj.cal1.rotateNext(isSame | isLess);
            obj.cal2 = obj.cal2.rotateNext(isSame | isGreater);
            obj.cal1 = obj.cal1.rotateDeg(-ang2,isGreater);
            obj.cal2 = obj.cal2.rotateDeg(-ang1,isLess);
        end
        
        function obj = rotate(obj)
            obj = obj.runCalculationsAtStep;
            while ~obj.done
                obj = obj.rotateNext();
                obj = obj.runCalculationsAtStep;
            end
        end
        
        function obj = resetCalculations(obj)
            obj.distmat = [];
            obj.x1mat = [];
            obj.x2mat = [];
            obj.y1mat = [];
            obj.y2mat = [];
            obj.sideofmat = [];
            obj.bridge1 = [];
            obj.bridge2 = [];
            obj.csl1 = [];
            obj.csl2 = [];
            obj.cal1.initialize(false);
            obj.cal2.initialize(true);
        end
        
        function obj = runCalculationsAtStep(obj)
            obj = obj.minDistanceAtStep;
            obj = obj.relativeSideAtStep;
            obj = obj.crossingLineAtStep;
        end
        
        function obj = minDistanceAtStep(obj)
            % caliper 1
            parallel1 = obj.cal1.parallel;
            side1 = obj.cal1.prevside;
            corner1 = obj.cal1.corner;
            % caliper 2
            parallel2 = obj.cal2.parallel;
            side2 = obj.cal2.prevside;
            corner2 = obj.cal2.corner;
            % distance arrays and dlines
            [dist,dline] = line2d.distSegmentToSegmentParallel(side1, side2, parallel1 & parallel2);
            [distSegToPoint,dlineSegToPoint] = side1.distToPoint(corner2);
            [distPointToSeg,dlinePointToSeg] = side2.distToPoint(corner1);
            dlinePointToSeg = dlinePointToSeg.flipDirection;
            dlinePointToPoint = line2d(corner1,corner2);
            distPointToPoint = length(dlinePointToPoint);
            dist(parallel1 & ~parallel2) = distSegToPoint(parallel1 & ~parallel2);
            dline.point1(parallel1 & ~parallel2,:) = dlineSegToPoint.point1(parallel1 & ~parallel2,:);
            dline.point2(parallel1 & ~parallel2,:) = dlineSegToPoint.point2(parallel1 & ~parallel2,:);
            dist(~parallel1 & parallel2) = distPointToSeg(~parallel1 & parallel2);
            dline.point1(~parallel1 & parallel2,:) = dlinePointToSeg.point1(~parallel1 & parallel2,:);
            dline.point2(~parallel1 & parallel2,:) = dlinePointToSeg.point2(~parallel1 & parallel2,:);
            dist(~parallel1 & ~parallel2) = distPointToPoint(~parallel1 & ~parallel2);
            dline.point1(~parallel1 & ~parallel2,:) = dlinePointToPoint.point1(~parallel1 & ~parallel2,:);
            dline.point2(~parallel1 & ~parallel2,:) = dlinePointToPoint.point2(~parallel1 & ~parallel2,:);
%             figure
%             plot(obj)
%             plot(dline,'r')
            % update state of object for final distance calculation
            obj.distmat = [obj.distmat dist];
            obj.x1mat = [obj.x1mat dline.point1(:,1)];
            obj.x2mat = [obj.x2mat dline.point2(:,1)];
            obj.y1mat = [obj.y1mat dline.point1(:,2)];
            obj.y2mat = [obj.y2mat dline.point2(:,2)];
        end
        
        function obj = relativeSideAtStep(obj)
            sideof = obj.cal1.cal.side(obj.cal2.corner);
            % update state of object for final intersection calculation
            obj.sideofmat = [obj.sideofmat sideof];
        end
        
        function obj = crossingLineAtStep(obj)
            w = line2d(obj.cal1.corner,obj.cal2.corner);
            if isempty(obj.csl1)
                obj.csl1 = w;
                w.point1 = NaN(size(w.point1));
                w.point2 = w.point1;
                obj.csl2 = obj.csl1;
                obj.bridge1 = obj.csl1;
                obj.bridge2 = obj.csl1;
            end
            sideof1prev = w.side(obj.cal1.prevcorner);
            sideof1next = w.side(obj.cal1.nextcorner);
            sideof2prev = w.side(obj.cal2.prevcorner);
            sideof2next = w.side(obj.cal2.nextcorner);
            isCandidate = sideof1prev==sideof1next & sideof2prev==sideof2next;
%             isBridge1 = sideof1prev>0 & sideof2prev>0;
%             isBridge2 = sideof1prev<0 & sideof2prev<0;
            isCsl1 = sideof1prev>0 & sideof2prev<0;
            isCsl2 = sideof1prev<0 & sideof2prev>0;
            obj.csl1.point1(isCandidate & isCsl1,:) = w.point1(isCandidate & isCsl1,:);
            obj.csl1.point2(isCandidate & isCsl1,:) = w.point2(isCandidate & isCsl1,:);
            obj.csl2.point1(isCandidate & isCsl2,:) = w.point1(isCandidate & isCsl2,:);
            obj.csl2.point2(isCandidate & isCsl2,:) = w.point2(isCandidate & isCsl2,:);
            % bridges don't work for some reason
%             obj.bridge1.point1(isCandidate & isBridge1,:) = w.point1(isCandidate & isBridge1,:);
%             obj.bridge1.point2(isCandidate & isBridge1,:) = w.point2(isCandidate & isBridge1,:);
%             obj.bridge2.point1(isCandidate & isBridge2,:) = w.point1(isCandidate & isBridge2,:);
%             obj.bridge2.point2(isCandidate & isBridge2,:) = w.point2(isCandidate & isBridge2,:);
        end
        
        function [dist,dline] = minDistance(obj)
            if ~obj.done
                obj = obj.resetCalculations;
                obj = obj.rotate;
            end
            [dist,Imin] = min(obj.distmat,[],2);
            rows = (1:size(dist,1))';
            linearInd = sub2ind(size(obj.distmat),rows,Imin);
            point1 = [obj.x1mat(linearInd) obj.y1mat(linearInd)];
            point2 = [obj.x2mat(linearInd) obj.y2mat(linearInd)];
            dline = line2d(point1,point2,'segment');
        end
        
        function intersection = intersection(obj)
            if ~obj.done
                obj = obj.resetCalculations;
                obj = obj.rotate;
            end
            intersection = all(obj.sideofmat<0,2) | all(obj.sideofmat>0,2);
        end
        
        function [csl1,csl2] = criticalSupportLines(obj)
            if ~obj.done
                obj = obj.resetCalculations;
                obj = obj.rotate;
            end
            csl1 = obj.csl1;
            csl2 = obj.csl2;
        end
        
        function between = isAngleBetweenCSLs(obj,line)
            if isempty(obj.csl1) || isempty(obj.csl2)
                error('critical support lines not calculated yet')
            end
            ang1 = obj.csl1.angleDeg;
            ang2 = obj.csl2.angleDeg;
            minang = min([ang1 ang2],[],2);
            maxang = max([ang1 ang2],[],2);
            ang = line.angleDeg;
            angflip = line.flipDirection.angleDeg;
            between1 = ang>=minang & ang<=maxang;
            between2 = angflip>=minang & angflip<=maxang;
            between = between1 | between2;
        end
        
        function [ttc,tca,dist,dist_tca,intersection,dline] = timeToCollision(obj,show)
            if nargin<2
                show = false;
            end
            if ~isa(obj.cal1.box,'mover2d') || ~isa(obj.cal2.box,'mover2d')
                error('TTC requires use of mover2d class for velocity')
            end
            if ~obj.done
                obj = obj.resetCalculations;
                obj = obj.rotate;
            end
            [dist,dline] = obj.minDistance;
            intersection = obj.intersection;
            dist(intersection) = 0;
            dvec = dline.toVector;
            vvec = obj.cal2.box.velline.toVector - obj.cal1.box.velline.toVector;
            vline = line2d([0 0],vvec,'segment');
            between = obj.isAngleBetweenCSLs(vline);
            % v = v2-v1;
            % tca = -dot(d,v)/dot(v,v);
            num = dvec(:,1).*vvec(:,1) + dvec(:,2).*vvec(:,2);
            den = vvec(:,1).*vvec(:,1) + vvec(:,2).*vvec(:,2);
            tca = -num./den;
            % interpolate to fill in missing frames and smooth out
            % discontinuities
            % the assumption is that all entries in this vector are related
            % by being contiguous points in time.  i'm not sure how valid
            % this assumption will be as a general use case
            if length(tca)>3
                tca = DropOutlierByPercentile(tca);
                if any(isnan(tca))
                    disp('interpolating tca vector')
                    f = (1:length(tca))';
                    isValid = ~isnan(tca);
                    tca = interp1(f(isValid),tca(isValid),f,'linear','extrap');
                end
            end
            % find the predicted miss distance by moving the boxes to their
            % locations at the TCA time
            mov1 = obj.cal1.box.predict(tca);
            mov1 = mov1.update;
            mov2 = obj.cal2.box.predict(tca);
            mov2 = mov2.update;
            rc_tca = rotatingCalipers(mov1,mov2);
            dist_tca = rc_tca.minDistance;
            dist_tca(between) = 0;
            % time to collision
            ttc = tca;
            ttc(~between) = NaN;
            ttc(ttc<0) = NaN;
            if show
                NewFigure('rc:ttc');
                subplot(3,1,1)
                plot(tca)
                hold on
                plot(ttc,'c.')
                title('time to collision and time to closest approach')
                legend('tca','ttc','Location','Best')
                ylabel('(sec)')
                subplot(3,1,2)
                plot(dist,'g')
                title('minimum distance')
                ylabel('(ft)')
                subplot(3,1,3)
                plot(intersection,'ko')
                title('collision')
                
                NewFigure('rc:rc');
                plotn(obj,10)
            end
        end
        
        function plot(obj,varargin)
            plot(obj.cal1,'b')
            hold on
            plot(obj.cal2,'r')
%             if obj.done
%                 [dist,dline] = obj.minDistance;
%                 plot(dline,'r')
%             end
%             if ~isempty(obj.csl1)
%                 plot(obj.csl1,'b--')
%             end
%             if ~isempty(obj.csl2)
%                 plot(obj.csl2,'c--')
%             end
%             if ~isempty(obj.bridge1)
%                 plot(obj.bridge1,'r--')
%             end
%             if ~isempty(obj.bridge2)
%                 plot(obj.bridge2,'m--')
%             end
        end
        
        function plotn(obj,n,varargin)
            plotn(obj.cal1,n,'b')
            hold on
            plotn(obj.cal2,n,'r')
%             if obj.done
%                 [dist,dline] = obj.minDistance;
%                 plotn(dline,n,'r')
%             end
%             if ~isempty(obj.csl1)
%                 plotn(obj.csl1,n,'b--')
%             end
%             if ~isempty(obj.csl2)
%                 plotn(obj.csl2,n,'c--')
%             end
%             if ~isempty(obj.bridge1)
%                 plotn(obj.bridge1,n,'r--')
%             end
%             if ~isempty(obj.bridge2)
%                 plotn(obj.bridge2,n,'m--')
%             end
        end
        
        function ploti(obj,i,varargin)
            ploti(obj.cal1,i,'b')
            hold on
            ploti(obj.cal2,i,'r')
        end
    end
end