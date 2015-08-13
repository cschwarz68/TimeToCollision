classdef RotatingCalipersVectorTest < matlab.unittest.TestCase
    properties
        rc
    end
    
    methods (TestMethodSetup)
        function initboxes(testCase)
            % box1
            box1 = box2d(5,2,[1 0]);
            ref1 = [0 0; -1 5; -2 10; -3 15; -4 20];
            box1 = box1.setreference(ref1);
            ang1 = [0; 70; 140; 210; 280];
            box1 = box1.setangleDeg(ang1);
            % box2
            box2 = box2d(4,3);
            ref2 = [5 0; 6 4; 7 8; 8 12; 9 16];
            box2 = box2.setreference(ref2);
            ang2 = [0; 50; 100; 150; 200];
            box2 = box2.setangleDeg(ang2);
            % rotating caliper
            testCase.rc = rotatingCalipers(box1,box2);
        end
    end
    
    methods (TestMethodTeardown)
    end
    
    methods (Test)
        function testPlotRotatingCalipers(testCase)
            testCase.rc = testCase.rc.rotate;
            NewFigure('rotatingCalipers');
            plot(testCase.rc)
        end
        
        function testRotate(testCase)
            % a complete rotating calipers revolution
            % done status does not reset
            testCase.rc = testCase.rc.rotate;
            % tests for cal1
            ind1 = testCase.rc.cal1.index;
            expected_ind1 = [2; 2; 3; 4; 1];
            testCase.verifyEqual(ind1,expected_ind1);
            done1 = testCase.rc.cal1.done;
            expected_done1 = [true; true; true; true; true];
            testCase.verifyEqual(done1,expected_done1);
            ang1 = testCase.rc.cal1.nextAngle;
            expected_ang1 = [90; 70; 50; 90; 90];
            testCase.verifyEqual(ang1,expected_ang1,'AbsTol',1e-3);
            % tests for cal2
            ind2 = testCase.rc.cal2.index;
            expected_ind2 = [4; 4; 1; 1; 2];
            testCase.verifyEqual(ind2,expected_ind2);
            done2 = testCase.rc.cal2.done;
            expected_done2 = [true; true; true; true; true];
            testCase.verifyEqual(done2,expected_done2);
            ang2 = testCase.rc.cal2.nextAngle;
            expected_ang2 = [90; 90; 90; 60; 80];
            testCase.verifyEqual(ang2,expected_ang2,'AbsTol',1e-3);
            % test for bridge lines and critical support lines
            plot(testCase.rc)
            [csl1,csl2] = testCase.rc.criticalSupportLines;
            ang1 = csl1.angleDeg;
            expected_ang1 = [59.0362; 34.739; 27.9433; 4.8354; -0.9702];
            testCase.verifyEqual(ang1,expected_ang1,'AbsTol',1e-3);
            ang2 = csl2.angleDeg;
            expected_ang2 = [-59.0362; -40.2541; -35.4343; -44.6544; -40.793];
            testCase.verifyEqual(ang2,expected_ang2,'AbsTol',1e-3);
        end
        
        function testCalculations(testCase)
            mindist = testCase.rc.minDistance;
            expected_dist = [1.5; 3.8263; 4.0002; 6.4103; 9.5675];
            testCase.verifyEqual(mindist,expected_dist,'AbsTol',1e-3);
            intersections = testCase.rc.intersection;
            expected_ixsn = [false; false; false; false; false];
            testCase.verifyEqual(intersections,expected_ixsn);
        end
        
        function testIntersection(testCase)
            % box1
            box1 = box2d(5,2,[1 0]);
            ref1 = [0 0; 0 5; 0 10; 0 15; 0 20];
            box1 = box1.setreference(ref1);
            ang1 = [0; 70; 140; 210; 280];
            box1 = box1.setangleDeg(ang1);
            % box2
            box2 = box2d(4,3);
            ref2 = [3 0; 3 4; 4 8; 5 12; 6 16];
            box2 = box2.setreference(ref2);
            ang2 = [0; 50; 100; 150; 200];
            box2 = box2.setangleDeg(ang2);
            % rotating caliper
            rc = rotatingCalipers(box1,box2);
%             figure
%             plot(rc)
            ixsn = rc.intersection;
            expected_ixsn = [true; false; true; false; false];
            testCase.verifyEqual(ixsn,expected_ixsn);
        end
                
        function testRotateNext(testCase)
            % first rotation
            testCase.rc = testCase.rc.rotateNext;
%             figure
%             plot(testCase.rc)
            % tests for cal1
            ind1 = testCase.rc.cal1.index;
            expected_ind1 = [3; 3; 4; 4; 1];
            testCase.verifyEqual(ind1,expected_ind1);
            done1 = testCase.rc.cal1.done;
            expected_done1 = [false; false; false; false; false];
            testCase.verifyEqual(done1,expected_done1);
            ang1 = testCase.rc.cal1.nextAngle;
            expected_ang1 = [90; 90; 90; 30; 10];
            testCase.verifyEqual(ang1,expected_ang1,'AbsTol',1e-3);
            % tests for cal2
            ind2 = testCase.rc.cal2.index;
            expected_ind2 = [1; 4; 1; 2; 3];
            testCase.verifyEqual(ind2,expected_ind2);
            done2 = testCase.rc.cal2.done;
            expected_done2 = [false; false; false; false; false];
            testCase.verifyEqual(done2,expected_done2);
            ang2 = testCase.rc.cal2.nextAngle;
            expected_ang2 = [90; 20; 40; 90; 90];
            testCase.verifyEqual(ang2,expected_ang2,'AbsTol',1e-3);
            
            % second rotation
            testCase.rc = testCase.rc.rotateNext;
%             figure
%             plot(testCase.rc)
            % tests for cal1
            ind1 = testCase.rc.cal1.index;
            expected_ind1 = [4; 3; 4; 1; 2];
            testCase.verifyEqual(ind1,expected_ind1);
            done1 = testCase.rc.cal1.done;
            expected_done1 = [false; false; false; false; false];
            testCase.verifyEqual(done1,expected_done1);
            ang1 = testCase.rc.cal1.nextAngle;
            expected_ang1 = [90; 70; 50; 90; 90];
            testCase.verifyEqual(ang1,expected_ang1,'AbsTol',1e-3);
            % tests for cal2
            ind2 = testCase.rc.cal2.index;
            expected_ind2 = [2; 1; 2; 2; 3];
            testCase.verifyEqual(ind2,expected_ind2);
            done2 = testCase.rc.cal2.done;
            expected_done2 = [false; false; false; false; false];
            testCase.verifyEqual(done2,expected_done2);
            ang2 = testCase.rc.cal2.nextAngle;
            expected_ang2 = [90; 90; 90; 60; 80];
            testCase.verifyEqual(ang2,expected_ang2,'AbsTol',1e-3);
            
            % third rotation
            testCase.rc = testCase.rc.rotateNext;
%             figure
%             plot(testCase.rc)
            % tests for cal1
            ind1 = testCase.rc.cal1.index;
            expected_ind1 = [1; 4; 1; 1; 2];
            testCase.verifyEqual(ind1,expected_ind1);
            done1 = testCase.rc.cal1.done;
            expected_done1 = [false; false; false; false; false];
            testCase.verifyEqual(done1,expected_done1);
            ang1 = testCase.rc.cal1.nextAngle;
            expected_ang1 = [90; 90; 90; 30; 10];
            testCase.verifyEqual(ang1,expected_ang1,'AbsTol',1e-3);
            % tests for cal2
            ind2 = testCase.rc.cal2.index;
            expected_ind2 = [3; 1; 2; 3; 4];
            testCase.verifyEqual(ind2,expected_ind2);
            done2 = testCase.rc.cal2.done;
            expected_done2 = [false; false; false; false; false];
            testCase.verifyEqual(done2,expected_done2);
            ang2 = testCase.rc.cal2.nextAngle;
            expected_ang2 = [90; 20; 40; 90; 90];
            testCase.verifyEqual(ang2,expected_ang2,'AbsTol',1e-3);
            
            % fourth rotation
            testCase.rc = testCase.rc.rotateNext;
%             figure
%             plot(testCase.rc)
            % tests for cal1
            ind1 = testCase.rc.cal1.index;
            expected_ind1 = [2; 4; 1; 2; 3];
            testCase.verifyEqual(ind1,expected_ind1);
            done1 = testCase.rc.cal1.done;
            expected_done1 = [true; false; false; false; false];
            testCase.verifyEqual(done1,expected_done1);
            ang1 = testCase.rc.cal1.nextAngle;
            expected_ang1 = [90; 70; 50; 90; 90];
            testCase.verifyEqual(ang1,expected_ang1,'AbsTol',1e-3);
            % tests for cal2
            ind2 = testCase.rc.cal2.index;
            expected_ind2 = [4; 2; 3; 3; 4];
            testCase.verifyEqual(ind2,expected_ind2);
            done2 = testCase.rc.cal2.done;
            expected_done2 = [true; false; false; false; false];
            testCase.verifyEqual(done2,expected_done2);
            ang2 = testCase.rc.cal2.nextAngle;
            expected_ang2 = [90; 90; 90; 60; 80];
            testCase.verifyEqual(ang2,expected_ang2,'AbsTol',1e-3);
            
            % fifth rotation
            testCase.rc = testCase.rc.rotateNext;
            
            % sixth rotation
            testCase.rc = testCase.rc.rotateNext;
            
            % seventh rotation
            testCase.rc = testCase.rc.rotateNext;
%             figure
%             plot(testCase.rc)
            % tests for cal1
            ind1 = testCase.rc.cal1.index;
            expected_ind1 = [1; 2; 3; 3; 4];
            testCase.verifyEqual(ind1,expected_ind1);
            done1 = testCase.rc.cal1.done;
            expected_done1 = [true; true; true; false; false];
            testCase.verifyEqual(done1,expected_done1);
            ang1 = testCase.rc.cal1.nextAngle;
            expected_ang1 = [90; 90; 90; 30; 10];
            testCase.verifyEqual(ang1,expected_ang1,'AbsTol',1e-3);
            % tests for cal2
            ind2 = testCase.rc.cal2.index;
            expected_ind2 = [3; 3; 4; 1; 2];
            testCase.verifyEqual(ind2,expected_ind2);
            done2 = testCase.rc.cal2.done;
            expected_done2 = [true; false; false; true; true];
            testCase.verifyEqual(done2,expected_done2);
            ang2 = testCase.rc.cal2.nextAngle;
            expected_ang2 = [90; 20; 40; 90; 90];
            testCase.verifyEqual(ang2,expected_ang2,'AbsTol',1e-3);
            
            % eigth rotation
            testCase.rc = testCase.rc.rotateNext;
%             figure
%             plot(testCase.rc)
            % tests for cal1
            ind1 = testCase.rc.cal1.index;
            expected_ind1 = [2; 2; 3; 4; 1];
            testCase.verifyEqual(ind1,expected_ind1);
            done1 = testCase.rc.cal1.done;
            expected_done1 = [true; true; true; true; true];
            testCase.verifyEqual(done1,expected_done1);
            ang1 = testCase.rc.cal1.nextAngle;
            expected_ang1 = [90; 70; 50; 90; 90];
            testCase.verifyEqual(ang1,expected_ang1,'AbsTol',1e-3);
            % tests for cal2
            ind2 = testCase.rc.cal2.index;
            expected_ind2 = [4; 4; 1; 1; 2];
            testCase.verifyEqual(ind2,expected_ind2);
            done2 = testCase.rc.cal2.done;
            expected_done2 = [true; true; true; true; true];
            testCase.verifyEqual(done2,expected_done2);
            ang2 = testCase.rc.cal2.nextAngle;
            expected_ang2 = [90; 90; 90; 60; 80];
            testCase.verifyEqual(ang2,expected_ang2,'AbsTol',1e-3);
        end
    end
end