classdef Box2dVectorTest < matlab.unittest.TestCase
    properties
        box1
        box2
    end
    
    methods (TestMethodSetup)
        function initboxes(testCase)
            % box1
            testCase.box1 = box2d(5,2,[1 0]);
            ref1 = [0 0; -1 5; -2 10; -3 15; -4 20];
            testCase.box1 = testCase.box1.setreference(ref1);
            ang1 = [0; 70; 140; 210; 280];
            testCase.box1 = testCase.box1.setangleDeg(ang1);
            testCase.box1 = testCase.box1.update;
            % box2
            testCase.box2 = box2d(4,3);
            ref2 = [5 0; 6 4; 7 8; 8 12; 9 16];
            testCase.box2 = testCase.box2.setreference(ref2);
            ang2 = [0; 50; 100; 150; 200];
            testCase.box2 = testCase.box2.setangleDeg(ang2);
            testCase.box2 = testCase.box2.update;
        end
    end
    
    methods (TestMethodTeardown)
    end
    
    methods (Test)
        function testPlotBox(testCase)
            NewFigure('box2d test boxes')
            plot(testCase.box1)
            hold on
            plot(testCase.box2)
        end
        
        function testCenter(testCase)
            c1 = testCase.box1.center;
            c2 = testCase.box2.center;
            c = [c1; c2];
            x = c(:,1);
            expected_x = [-1; -1.342; -1.234; -2.134; -4.1736; 5; 6; 7; 8; 9];
            testCase.verifyEqual(x,expected_x,'AbsTol',1e-3);
            y = c(:,2);
            expected_y = [0; 4.0603; 9.3572; 15.5; 20.9848; 0; 4; 8; 12; 16];
            testCase.verifyEqual(y,expected_y,'AbsTol',1e-3);
        end
        
        function testGetCorner(testCase)
            Ic = [1; 2; 3; 4; 1];
            c1 = testCase.box1.getCorner(Ic);
            c2 = testCase.box2.getCorner(Ic);
            c = [c1; c2];
            x = c(:,1);
            expected_x = [1.5; -1.2574; 0.0384; -3.799; -4.7243; 7; 5.8635; 5.8701; 5.5179; 6.6076];
            testCase.verifyEqual(x,expected_x,'AbsTol',1e-3);
            y = c(:,2);
            expected_y = [-1; 1.3691; 6.9842; 13.384; 18.3491; -1.5; 1.5037; 5.7699; 11.701; 16.7255];
            testCase.verifyEqual(x,expected_x,'AbsTol',1e-3);
        end
        
        function testGetSide(testCase)
            Is = [1; 2; 3; 4; 1];
            s1 = testCase.box1.getSide(Is);
            s2 = testCase.box2.getSide(Is);
            hold on
            plot(s1,'r')
            plot(s2,'m')
            point1 = [s1.point1; s2.point1];
            point2 = [s1.point2; s2.point2];
            % test point 1
            x = point1(:,1);
            expected_x = [1.5; -1.2574; 0.0384; -3.799; -4.7243; 7; 5.8635; 5.8701; 5.5179; 6.6076];
            testCase.verifyEqual(x,expected_x,'AbsTol',1e-3);
            y = point1(:,2);
            expected_y = [-1; 1.3691; 6.9842; 13.384; 18.3491; -1.5; 1.5037; 5.7699; 11.701; 16.7255];
            testCase.verifyEqual(x,expected_x,'AbsTol',1e-3);
            % test point 2
            x = point2(:,1);
            expected_x = [-3.5; -3.1368; -3.7919; -4.799; -5.5926; 3; 3.5654; 5.1755; 7.0179; 10.3664];
            testCase.verifyEqual(x,expected_x,'AbsTol',1e-3);
            y = point2(:,2);
            expected_y = [-1; 2.0531; 10.1981; 15.116; 23.2732; -1.5; 3.4321; 9.7091; 14.299; 18.0936];
            testCase.verifyEqual(x,expected_x,'AbsTol',1e-3);
        end
        
        function testMinX(testCase)
            [c1,Ic1] = testCase.box1.minX;
            [c2,Ic2] = testCase.box2.minX;
            c = [c1; c2];
%             hold on
%             for i = 1:size(c,1)
%                 plot(c(i,1),c(i,2),'kd')
%             end
            d = [Ic1; Ic2];
            expected_d = [2; 3; 4; 1; 2; 2; 3; 4; 4; 1];
            testCase.verifyEqual(d,expected_d);
            x = c(:,1);
            expected_x = [-3.5; -3.1368; -3.7919; -4.799; -5.592; 3; 3.5654; 5.1755; 5.5179; 6.6076];
            testCase.verifyEqual(x,expected_x,'AbsTol',1e-3);
            y = c(:,2);
            expected_y = [-1; 2.0531; 10.1981; 15.1160; 23.2732; -1.5; 3.4321; 9.7091; 11.7010; 16.7255];
            testCase.verifyEqual(y,expected_y,'AbsTol',1e-3);
        end
        
        function testMaxX(testCase)
            [c1,Ic1] = testCase.box1.maxX;
            [c2,Ic2] = testCase.box2.maxX;
            c = [c1; c2];
%             hold on
%             for i = 1:size(c,1)
%                 plot(c(i,1),c(i,2),'kd')
%             end
            d = [Ic1; Ic2];
            expected_d = [1; 1; 2; 3; 4; 1; 1; 2; 2; 3];
            testCase.verifyEqual(d,expected_d);
            x = c(:,1);
            expected_x = [1.5; 0.4527; 1.3239; 0.5311; -2.7547; 7; 8.4346; 8.8245; 10.4821; 11.3924];
            testCase.verifyEqual(x,expected_x,'AbsTol',1e-3);
            y = c(:,2);
            expected_y = [-1; 6.0675; 8.5163; 15.884; 18.6964; -1.5; 4.5679; 6.2909; 12.299; 15.2745];
            testCase.verifyEqual(y,expected_y,'AbsTol',1e-3);
        end
        
        function testMinY(testCase)
            [c1,Ic1] = testCase.box1.minY;
            [c2,Ic2] = testCase.box2.minY;
            c = [c1; c2];
%             hold on
%             for i = 1:size(c,1)
%                 plot(c(i,1),c(i,2),'kd')
%             end
            d = [Ic1; Ic2];
            expected_d = [1; 2; 3; 4; 1; 1; 2; 3; 3; 4];
            testCase.verifyEqual(d,expected_d);
            x = c(:,1);
            expected_x = [1.5; -1.2574; 0.0384; -3.799; -4.7243; 7; 5.8635; 5.8701; 8.9821; 7.6336];
            testCase.verifyEqual(x,expected_x,'AbsTol',1e-3);
            y = c(:,2);
            expected_y = [-1; 1.3691; 6.9842; 13.384; 18.3491; -1.5; 1.5037; 5.7699; 9.701; 13.9064];
            testCase.verifyEqual(y,expected_y,'AbsTol',1e-3);
        end
        
        function testMaxY(testCase)
            [c1,Ic1] = testCase.box1.maxY;
            [c2,Ic2] = testCase.box2.maxY;
            c = [c1; c2];
%             hold on
%             for i = 1:size(c,1)
%                 plot(c(i,1),c(i,2),'kd')
%             end
            d = [Ic1; Ic2];
            expected_d = [3; 4; 1; 2; 3; 3; 4; 1; 1; 2];
            testCase.verifyEqual(d,expected_d);
            x = c(:,1);
            expected_x = [-3.5; -1.4267; -2.5063; -.4689; -3.623; 3; 6.1365; 8.1299; 7.0179; 10.3664];
            testCase.verifyEqual(x,expected_x,'AbsTol',1e-3);
            y = c(:,2);
            expected_y = [1; 6.7516; 11.7302; 17.616; 23.6205; 1.5; 6.4963; 10.2301; 14.299; 18.0936];
            testCase.verifyEqual(y,expected_y,'AbsTol',1e-3);
        end
    end
end