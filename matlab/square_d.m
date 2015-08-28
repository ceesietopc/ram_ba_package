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
topic1 = 'John/setpoint';
topic2 = 'James/setpoint';

% Initialize node, publisher and message
node = rosmatlab.node('matlab_square',core);
pose_pub1 = rosmatlab.publisher(topic1, 'geometry_msgs/Pose', node);
pose1 = rosmatlab.message('geometry_msgs/Pose',node);
pose_pub2 = rosmatlab.publisher(topic2, 'geometry_msgs/Pose', node);
pose2 = rosmatlab.message('geometry_msgs/Pose',node);
tic;

% Paramers
l = 2; % Time per setpoint
d = 1; % Setpoint coord.
z = 1.5; % Setpoint z

% Prepare plot
figure;

while true
    if toc() > 0*l
        pose1.getPosition().setX(-d);
        pose1.getPosition().setY(-d);
        pose2.getPosition().setX(-d);
        pose2.getPosition().setY(d);
    end
    if toc() > 1*l
        pose1.getPosition().setX(-d);
        pose1.getPosition().setY(d);
        pose2.getPosition().setX(d);
        pose2.getPosition().setY(d);
    end
    if toc() > 2*l
        pose1.getPosition().setX(d);
        pose1.getPosition().setY(d);
        pose2.getPosition().setX(d);
        pose2.getPosition().setY(-d);
    end
    if toc() > 3*l
        pose1.getPosition().setX(d);
        pose1.getPosition().setY(-d);
        pose2.getPosition().setX(-d);
        pose2.getPosition().setY(-d);
    end
    if toc() > 4*l
        % Reset timer  to keep looping
        tic;
    end
    pose1.getPosition().setZ(z);
    pose2.getPosition().setZ(z);
    
    clf;
    hold on;
    plot(pose1.getPosition().getX(),pose1.getPosition().getY(),'rd');
    plot(pose2.getPosition().getX(),pose2.getPosition().getY(),'gd');
    axis([-1.5 1.5 -1.5 1.5]);
    
    pause(0.5);
    pose_pub1.publish(pose1);
    pose_pub2.publish(pose2);
    
end