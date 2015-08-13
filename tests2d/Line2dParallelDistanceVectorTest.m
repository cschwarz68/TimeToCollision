classdef Line2dParallelDistanceVectorTest < matlab.unittest.TestCase
    properties
        segments
        rays
    end
    
    methods (TestMethodSetup)
        function initsegments(testCase)
            % line0 = line2d([1 1],[3 1]);
            % rays
            p1 = [-1 2; -3 0];
            p2 = [-3 2; -1 0];
            testCase.rays = line2d(p1,p2,'ray');
            % segments
            p1 = [-1 2; -1 -1; 5 2; 5 -1; 2 2; 2 -1; 2 2; 2 -1];
            p2 = [-3 2; -3 -1; 7 2; 7 -1; -3 2; -3 -1; 7 2; 7 -1];
            testCase.segments = line2d(p1,p2,'segment');
        end
    end
    
    methods (TestMethodTeardown)
    end
    
    methods (Test)        
        function testDistRayToRay(testCase)
            line0 = line2d([1 1],[3 1],'ray');
            [d,dline] = line2d.distParallel(line0,testCase.rays);
%             plot(line0)
%             hold on
%             plot(testCase.rays)
%             plot(dline,'r')
            expected_d = [norm([1 1]-[-1 2]); 1];
            testCase.verifyEqual(d,expected_d);
        end

        function testDistRayToSegment(testCase)
            line0 = line2d([1 1],[3 1],'ray');
            d = line2d.distParallel(line0,testCase.segments);
            expected_d = [norm([1 1]-[-1 2]);
                norm([1 1]-[-1 -1]);
                1; 2; 1; 2; 1; 2];
            testCase.verifyEqual(d,expected_d);
        end
                
        function testDistSegmentToRay(testCase)
            line0 = line2d([1 1],[3 1],'ray');
            d = line2d.distParallel(testCase.segments,line0);
            expected_d = [norm([1 1]-[-1 2]);
                norm([1 1]-[-1 -1]);
                1; 2; 1; 2; 1; 2];
            testCase.verifyEqual(d,expected_d);
        end
                
        function testDistSegmentToRayFlip(testCase)
            line0 = line2d([1 1],[3 1],'ray');
            line0 = line0.flipDirection;
            d = line2d.distParallel(testCase.segments,line0);
            expected_d = [1; 2; norm([3 1]-[5 2]); norm([3 1]-[5 -1]); 
                1; 2; 1; 2];
            testCase.verifyEqual(d,expected_d);
        end
        
        function testDistSegmentToSegment(testCase)
            line0 = line2d([1 1],[3 1],'segment');
            [d,dline] = line2d.distParallel(line0,testCase.segments);
%             plot(line0)
%             hold on
%             plot(testCase.segments)
%             plot(dline,'r')
            expected_d = [norm([1 1]-[-1 2]); 
                norm([1 1]-[-1 -1]);
                norm([3 1]-[5 2]);
                norm([3 1]-[5 -1]);
                1; 2; 1; 2];
            testCase.verifyEqual(d,expected_d);
        end        
    end
end