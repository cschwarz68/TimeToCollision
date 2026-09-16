% example data with an ego vehicle following a lead vehicle
load example.mat

% create a MoverPair object using position, heading, velocity, length,
% width, and bounding box center
pair = MoverPair(ego, lead);

% call timeToCollision using the rotating calipers class
% plot the results
show = true;
[ttc,tca,dist,dist_tca,intersection,dline] = timeToCollision(pair, show);