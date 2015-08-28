% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.

% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.

% You should have received a copy of the GNU General Public License
% along with this program.  If not, see <http://www.gnu.org/licenses/>.

% Core and topic information
core = 'http://ramflight-arena:11311';
topic = 'James/setpoint';

% Initialize node, publisher and message
node = rosmatlab.node('matlab_square',core);
pose_pub = rosmatlab.publisher(topic, 'geometry_msgs/Pose', node);
pose = rosmatlab.message('geometry_msgs/Pose',node);
tic;

% Paramers
l = 5; % Time per setpoint
d = 1; % Setpoint coord.
z = 1.5; % Setpoint z

% Prepare plot
figure;

while true
    if toc() > 0*l
        pose.getPosition().setX(-d);
        pose.getPosition().setY(-d);
    end
    if toc() > 1*l
        pose.getPosition().setX(-d);
        pose.getPosition().setY(d);
    end
    if toc() > 2*l
        pose.getPosition().setX(d);
        pose.getPosition().setY(d);
    end
    if toc() > 3*l
        pose.getPosition().setX(d);
        pose.getPosition().setY(-d);
    end
    if toc() > 4*l
        % Reset timer  to keep looping
        tic; 
    end
    pose.getPosition().setZ(z);
    pose.getOrientation().setW(1);
    
    plot(pose.getPosition().getX(),pose.getPosition().getY(),'rd');
    axis([-1.5 1.5 -1.5 1.5]);
    
    pause(0.5);
    pose_pub.publish(pose);
end