% this is a graphical test of the distance between two boxes
% the test starts with two boxes and rotates them in a loop and calculates
% the minimum distance each time, plotting it for visual verification
box1 = box2d([-2 -1],[2 1]);
box1 = box1.translate([-3 0]);
box2 = box2d([-2 -1],[2 1]);
% box2 = box2.translate([3 0]); % non-intersecting boxes
box2 = box2.translate([0 0]); % intersecting boxes
for i = 1:10
    box1 = box1.rotateDeg(36);
    box2 = box2.rotateDeg(72);
%     if i<5, continue, end
    [dist,collision] = box2d.distance(box1,box2,true);
    if collision
        disp('collision!')
    end
    disp(['dist = ' num2str(dist)])
end