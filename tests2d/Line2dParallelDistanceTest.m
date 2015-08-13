classdef Line2dParallelDistanceTest < matlab.unittest.TestCase
    properties
        segments
        rays
    end
    
    methods (TestMethodSetup)
        function initsegments(testCase)
            % line0 = line2d([1 1],[3 1]);
            % non-overlapping ray
            testCase.rays{1} = line2d([-1 2],[-3 2],'ray');
            % overlapping ray
            testCase.rays{2} = line2d([-3 2],[-1 2],'ray');
            % non-overlapping segments
            testCase.segments{1} = line2d([-1 2],[-3 2],'segment');
            testCase.segments{2} = line2d([-1 -1],[-3 -1],'segment');
            testCase.segments{3} = line2d([5 2],[7 2],'segment');
            testCase.segments{4} = line2d([5 -1],[7 -1],'segment');
            % overlapping segments
            testCase.segments{5} = line2d([2 2],[-3 2],'segment');
            testCase.segments{6} = line2d([2 -1],[-3 -1],'segment');
            testCase.segments{7} = line2d([2 2],[7 2],'segment');
            testCase.segments{8} = line2d([2 -1],[7 -1],'segment');
        end
    end
    
    methods (TestMethodTeardown)
    end
    
    methods (Test)
        function testDistLineNonOverlapping(testCase)
            line0 = line2d([1 1],[3 1]);
            d = [line2d.distParallel(line0,testCase.segments{1}) ...
                line2d.distParallel(line0,testCase.segments{2}) ...
                line2d.distParallel(line0,testCase.segments{3}) ...
                line2d.distParallel(line0,testCase.segments{4})];
            expected_d = [1 2 1 2];
            testCase.verifyEqual(d,expected_d);
        end
        
        function testDistLineOverlapping(testCase)
            line0 = line2d([1 1],[3 1]);
            d = [line2d.distParallel(line0,testCase.segments{5}) ...
                line2d.distParallel(line0,testCase.segments{6}) ...
                line2d.distParallel(line0,testCase.segments{7}) ...
                line2d.distParallel(line0,testCase.segments{8})];
            expected_d = [1 2 1 2];
            testCase.verifyEqual(d,expected_d);
        end
        
        function testDistRayToRay(testCase)
            line0 = line2d([1 1],[3 1],'ray');
            d = [line2d.distParallel(line0,testCase.rays{1}) ...
                line2d.distParallel(line0,testCase.rays{2})];
            expected_d = [norm([1 1]-[-1 2]) 1];
            testCase.verifyEqual(d,expected_d);
        end

        function testDistRayToSegmentNonOverlapping(testCase)
            line0 = line2d([1 1],[3 1],'ray');
            d = [line2d.distParallel(line0,testCase.segments{1}) ...
                line2d.distParallel(line0,testCase.segments{2}) ...
                line2d.distParallel(line0,testCase.segments{3}) ...
                line2d.distParallel(line0,testCase.segments{4})];
            expected_d = [norm([1 1]-[-1 2]) norm([1 1]-[-1 -1]) 1 2];
            testCase.verifyEqual(d,expected_d);
        end
        
        function testDistRayToSegmentOverlapping(testCase)
            line0 = line2d([1 1],[3 1],'ray');
            d = [line2d.distParallel(line0,testCase.segments{5}) ...
                line2d.distParallel(line0,testCase.segments{6}) ...
                line2d.distParallel(line0,testCase.segments{7}) ...
                line2d.distParallel(line0,testCase.segments{8})];
            expected_d = [1 2 1 2];
            testCase.verifyEqual(d,expected_d);
        end
        
        function testDistRayToSegmentOverlappingFlip(testCase)
            line0 = line2d([1 1],[3 1],'ray');
            line0 = line0.flipDirection;
            d = [line2d.distParallel(line0,testCase.segments{1}) ...
                line2d.distParallel(line0,testCase.segments{2}) ...
                line2d.distParallel(line0,testCase.segments{3}) ...
                line2d.distParallel(line0,testCase.segments{4})];
            expected_d = [1 2 norm([3 1]-[5 2]) norm([3 1]-[5 -1])];
            testCase.verifyEqual(d,expected_d);
        end
        
        function testDistSegmentToRayToNonOverlapping(testCase)
            line0 = line2d([1 1],[3 1],'ray');
            d = [line2d.distParallel(testCase.segments{1},line0) ...
                line2d.distParallel(testCase.segments{2},line0) ...
                line2d.distParallel(testCase.segments{3},line0) ...
                line2d.distParallel(testCase.segments{4},line0)];
            expected_d = [norm([1 1]-[-1 2]) norm([1 1]-[-1 -1]) 1 2];
            testCase.verifyEqual(d,expected_d);
        end
        
        function testDistSegmentToRayToOverlapping(testCase)
            line0 = line2d([1 1],[3 1],'ray');
            d = [line2d.distParallel(testCase.segments{5},line0) ...
                line2d.distParallel(testCase.segments{6},line0) ...
                line2d.distParallel(testCase.segments{7},line0) ...
                line2d.distParallel(testCase.segments{8},line0)];
            expected_d = [1 2 1 2];
            testCase.verifyEqual(d,expected_d);
        end
        
        function testDistSegmentToRayToOverlappingFlip(testCase)
            line0 = line2d([1 1],[3 1],'ray');
            line0 = line0.flipDirection;
            d = [line2d.distParallel(testCase.segments{1},line0) ...
                line2d.distParallel(testCase.segments{2},line0) ...
                line2d.distParallel(testCase.segments{3},line0) ...
                line2d.distParallel(testCase.segments{4},line0)];
            expected_d = [1 2 norm([3 1]-[5 2]) norm([3 1]-[5 -1])];
            testCase.verifyEqual(d,expected_d);
        end
        
        function testDistSegmentToSegmentNonOverlapping(testCase)
            line0 = line2d([1 1],[3 1],'segment');
            d = [line2d.distParallel(line0,testCase.segments{1}) ...
                line2d.distParallel(line0,testCase.segments{2}) ...
                line2d.distParallel(line0,testCase.segments{3}) ...
                line2d.distParallel(line0,testCase.segments{4})];
            expected_d = [norm([1 1]-[-1 2]) norm([1 1]-[-1 -1]) norm([3 1]-[5 2]) norm([3 1]-[5 -1])];
            testCase.verifyEqual(d,expected_d);
        end
        
        function testDistSegmentToSegmentNonOverlappingFlip(testCase)
            line0 = line2d([1 1],[3 1],'segment');
            d = [line2d.distParallel(line0,testCase.segments{1}.flipDirection) ...
                line2d.distParallel(line0,testCase.segments{2}.flipDirection) ...
                line2d.distParallel(line0,testCase.segments{3}.flipDirection) ...
                line2d.distParallel(line0,testCase.segments{4}.flipDirection)];
            expected_d = [norm([1 1]-[-1 2]) norm([1 1]-[-1 -1]) norm([3 1]-[5 2]) norm([3 1]-[5 -1])];
            testCase.verifyEqual(d,expected_d);
        end
        
        function testDistSegmentToSegmentOverlapping(testCase)
            line0 = line2d([1 1],[3 1],'segment');
            d = [line2d.distParallel(line0,testCase.segments{5}) ...
                line2d.distParallel(line0,testCase.segments{6}) ...
                line2d.distParallel(line0,testCase.segments{7}) ...
                line2d.distParallel(line0,testCase.segments{8})];
            expected_d = [1 2 1 2];
            testCase.verifyEqual(d,expected_d);
        end
        
            
        function testDistSegmentToSegmentOverlappingFlip(testCase)
            line0 = line2d([1 1],[3 1],'segment');
            d = [line2d.distParallel(line0,testCase.segments{5}.flipDirection) ...
                line2d.distParallel(line0,testCase.segments{6}.flipDirection) ...
                line2d.distParallel(line0,testCase.segments{7}.flipDirection) ...
                line2d.distParallel(line0,testCase.segments{8}.flipDirection)];
            expected_d = [1 2 1 2];
            testCase.verifyEqual(d,expected_d);
        end
        
    end
end