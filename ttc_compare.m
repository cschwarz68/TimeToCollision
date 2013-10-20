% this script was used to generate figures for the paper
option = 1;
switch option
    case 1 % miss
        v = line2d([-1 -.3],[.133 .467],'ray');
        u = line2d([1 0],[.5 .167],'ray');
        tend = 3;
    case 2 % collision
        v = line2d([0 0],[1 1/3],'ray');
        u = line2d([3 0],[2 1/3],'ray');
        tend = 2;
    case 3 % perpendicular
        v = line2d([0 0],[0 1],'ray');
        u = line2d([2.5 2],[1 2],'ray');
        tend = 3;
    otherwise
        error('unknown option')
end
% velocity vectors
vv = v.toVector;
uv = u.toVector;
uv-vv
NewFigure('velocity vectors');
plot(v)
hold on
plot(u,'r')
% init loop
ttc1 = [];
ttc2 = [];
ttc3 = [];
SKIP = 0.01;
p0 = -.01;
d = line2d(v.getPoint(p0),u.getPoint(p0),'segment');
dv = d.toVector;
dist = v.distToLineParametric(u,p0);
% the angle between the distance vector and the difference of the velocity
% vectors determines how much error there will be between the traditional
% computation method and the two dimensional analytic equation.  if
% cos(angle)=1, then the two methods are the same, otherwise they will be
% different
angle = acos(dot(dv,uv-vv)/(sqrt(dot(dv,dv)*dot(uv-vv,uv-vv))))*180/pi
% advance in time by increasing the parameter that defines the line
p = 0:SKIP:tend;
for i = 1:length(p)
    dist = [dist v.distToLineParametric(u,p(i))];
    d = line2d(v.getPoint(p(i)),u.getPoint(p(i)),'segment');
    dv = d.toVector;
    % one dimensional analytic
    ttc1 = [ttc1 -dot(dv,dv)/(dot(uv-vv,dv))];
    % one dimensional numeric
    ttc2 = [ttc2 -SKIP*dist(i+1)/(dist(i+1)-dist(i))];
    % two dimensional (no discontinuity!!!!!)
    ttc3 = [ttc3 -dot(dv,uv-vv)/dot(uv-vv,uv-vv)];
end
% plot the ttc over time
NewFigure('comparison')
hp1 = plot(p,ttc2)
grid on
hold on
hp2 = plot(p,ttc3,'g--')
hp3 = plot(p,dist(2:end),'-.')
xlabel('Time (seconds)')
set(gcf,'PaperPosition',[0.2500    2.5000    5.0000    4.0000])
switch option
    case 1
        ylabel('TCA (sec) & distance')
        legend('TTC','TCA','dist','Location','SouthWest')
        axis([0 1.5 -1.5 2.5])
        print('-deps','-r1000','miss.eps')
        print('-dtiff','miss.tif')
    case 2
        ylabel('TTC (sec) & distance')
        legend('TTC','TCA','dist','Location','NorthEast')
        print('-deps','-r1000','collision.eps')
        print('-dtiff','collision.tif')
    case 3
        ylabel('TTC (sec) & distance')
        legend('TTC','TCA','dist','Location','NorthEast')
        print('-deps','-r1000','perpendicular.eps')
        print('-dtiff','perpendicular.tif')
end

