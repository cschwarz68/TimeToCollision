% a test suite for box2d that is woefully incomplete
% I preferred to use the graphical test in boxdisttest.m
function test_suite = testBox2d
initTestSuite;

function boxes = setup
boxes{1} = box2d([-2 -1],[2 1]).rotateDeg(45).translate([-3 0]);
boxes{2} = box2d([-2 -1],[2 1]).translate([3 0]).rotateDeg(-45);
plot(boxes{1})
hold on
plot(boxes{2})

function testDistance(boxes)
[dist,sideof] = box2d.distance(boxes{1},boxes{2})
