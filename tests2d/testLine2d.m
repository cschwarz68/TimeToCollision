% this is the beginning of a test suite for the line2d class
function test_suite = testLine2d
initTestSuite;

function testSimpleLineFunctions
line1 = line2d([1,1],[2,2]);
line2 = line2d([0,0],[0,2]);
% test length
len = [line1.length line2.length];
expected_len = [sqrt(2) 2];
assertElementsAlmostEqual(len,expected_len);
% test angleDeg and flipDirection
ang = [line1.angleDeg line2.angleDeg line1.flipDirection.angleDeg line2.flipDirection.angleDeg];
expected_ang = [45 90 -135 -90];
assertElementsAlmostEqual(ang,expected_ang);
% test rotateDeg
newang = [line1.rotateDeg(10).angleDeg ...
    line1.rotateDeg(-10).angleDeg ...
    line2.rotateDeg(10).angleDeg ...
    line2.rotateDeg(-10).angleDeg ...
    line2.flipDirection.rotateDeg(10).angleDeg];
expected_newang = [55 35 100 80 -80];
assertElementsAlmostEqual(newang,expected_newang);
% test rotate2Deg
newang2 = [line1.rotate2Deg(10).angleDeg line2.rotate2Deg(-10).angleDeg];
expected_newang2 = [55 80];
assertElementsAlmostEqual(newang2,expected_newang2);
% test side
assertTrue(line1.side([1 0])<0)
assertTrue(line1.side([0 1])>0)
assertTrue(line1.flipDirection.side([1 0])>0)
assertTrue(line1.flipDirection.side([0 1])<0)
assertTrue(line1.side([-1 -1])==0)

function testLineDistFunctions
line1 = line2d([1,1],[2,2]);
line2 = line2d([0,0],[0,2]);
line3 = line2d([1,1.1],[2,2.1]);
% check distToPoint
d = [line1.distToPoint([1,-1]) line2.distToPoint([2,4])];
expected_d = [sqrt(2) 2];
assertElementsAlmostEqual(d,expected_d);
% check checkParallel
assertTrue(line2d.parallel(line1,line3))
assertTrue(line2d.parallel(line1.flipDirection,line3))
% check equals
assertTrue(line2d.equals(line3,line3))

function testRayDistFunctions
line1 = line2d([1,1],[2,2],'ray');
line2 = line2d([0,0],[0,2],'ray');
line3 = line2d([1,1.1],[2,2.1]);
% check distToPoint
d = [line1.distToPoint([1,-1]) line2.distToPoint([2,4])];
expected_d = [2 2];
assertElementsAlmostEqual(d,expected_d);
% check checkParallel
assertTrue(line2d.parallel(line1,line3))
assertTrue(line2d.parallel(line1.flipDirection,line3))

function testSegmentDistFunctions
line1 = line2d([1,1],[2,2],'segment');
line2 = line2d([0,0],[0,2],'segment');
line3 = line2d([1,1.1],[2,2.1]);
% check distToPoint
d = [line1.distToPoint([1,-1]) line2.distToPoint([2,4])];
expected_d = [2 2*sqrt(2)];
assertElementsAlmostEqual(d,expected_d);
% check checkParallel
assertTrue(line2d.parallel(line1,line3))
assertTrue(line2d.parallel(line1.flipDirection,line3))
