classdef CaliperVectorTest < matlab.unittest.TestCase
    properties
        cal1
        cal2
    end
    
    methods (TestMethodSetup)
        function initboxes(testCase)
            % box1
            box1 = box2d(5,2,[1 0]);
            ref1 = [0 0; -1 5; -2 10; -3 15; -4 20];
            box1 = box1.setreference(ref1);
            ang1 = [0; 70; 140; 210; 280];
            box1 = box1.setangleDeg(ang1);
            testCase.cal1 = caliper(box1);
            % box2
            box2 = box2d(4,3);
            ref2 = [5 0; 6 4; 7 8; 8 12; 9 16];
            box2 = box2.setreference(ref2);
            ang2 = [0; 50; 100; 150; 200];
            box2 = box2.setangleDeg(ang2);
            testCase.cal2 = caliper(box2,true);
        end
    end
    
    methods (TestMethodTeardown)
    end
    
    methods (Test)
        function testPlotCaliper(testCase)
            NewFigure('calipers');
            plot(testCase.cal1)
            hold on
            plot(testCase.cal2)
        end
        
        function testIndex0(testCase)
            ind0 = testCase.cal1.index0;
            expected_ind0 = [2; 2; 3; 4; 1];
            testCase.verifyEqual(ind0,expected_ind0);
            ind0 = testCase.cal2.index0;
            expected_ind0 = [4; 4; 1; 1; 2];
            testCase.verifyEqual(ind0,expected_ind0);
        end
        
        function testParallel(testCase)
            parallel1 = testCase.cal1.parallel;
            parallel2 = testCase.cal2.parallel;
            expected_ispar1 = [true; false; false; false; false];
            expected_ispar2 = [true; false; false; false; false];
            testCase.verifyEqual(parallel1,expected_ispar1);
            testCase.verifyEqual(parallel2,expected_ispar2);
            
            % rotate by angle
            testCase.cal1 = testCase.cal1.rotateDeg(-10);
            testCase.cal2 = testCase.cal2.rotateDeg(-10);
            parallel1 = testCase.cal1.parallel;
            parallel2 = testCase.cal2.parallel;
            expected_ispar1 = [false; false; false; false; false];
            expected_ispar2 = [false; false; false; false; false];
            testCase.verifyEqual(parallel1,expected_ispar1);
            testCase.verifyEqual(parallel2,expected_ispar2);
            
            % rotate to next side
            testCase.cal1 = testCase.cal1.rotateNext;
            testCase.cal2 = testCase.cal2.rotateNext;
            parallel1 = testCase.cal1.parallel;
            parallel2 = testCase.cal2.parallel;
            expected_ispar1 = [true; true; true; true; true];
            expected_ispar2 = [true; true; true; true; true];
            testCase.verifyEqual(parallel1,expected_ispar1);
            testCase.verifyEqual(parallel2,expected_ispar2);
        end
        
        function testRotateNext(testCase)            
            testCase.cal1 = testCase.cal1.rotateNext;
            ind = testCase.cal1.index;
            expected_ind = [3; 3; 4; 1; 2];
            testCase.verifyEqual(ind,expected_ind);
            
            testCase.cal1 = testCase.cal1.rotateNext;
            testCase.cal1 = testCase.cal1.rotateNext;
            done = testCase.cal1.done;
            expected_done = [false; false; false; false; false];
            testCase.verifyEqual(done,expected_done);

            testCase.cal1 = testCase.cal1.rotateNext;
            ind = testCase.cal1.index;
            expected_ind = [2; 2; 3; 4; 1];
            testCase.verifyEqual(ind,expected_ind);
            done = testCase.cal1.done;
            expected_done = [true; true; true; true; true];
            testCase.verifyEqual(done,expected_done);
        end
        
        function testRotate(testCase)
            testCase.cal1 = testCase.cal1.rotateDeg(-10);
            plot(testCase.cal1)
            ang = testCase.cal1.nextAngle;
            expected_ang = [80; 10; 30; 50; 70];
            testCase.verifyEqual(ang,expected_ang,'AbsTol',1e-3);
            
            testCase.cal1 = testCase.cal1.rotateDeg(-15);
            plot(testCase.cal1)
            ang = testCase.cal1.nextAngle;
            expected_ang = [65; 5; 15; 35; 55];
            testCase.verifyEqual(ang,expected_ang,'AbsTol',1e-3);
        end
    end
end