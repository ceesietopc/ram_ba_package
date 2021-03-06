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
node = rosmatlab.node('matlab_circle',core);
pose_pub1 = rosmatlab.publisher(topic1, 'geometry_msgs/Pose', node);
pose1 = rosmatlab.message('geometry_msgs/Pose',node);
pose_pub2 = rosmatlab.publisher(topic2, 'geometry_msgs/Pose', node);
pose2 = rosmatlab.message('geometry_msgs/Pose',node);
tic;

% Paramers
r = 1; % Setpoint coord.
w = 3; % wait in the beginning
z = 1.5; % Setpoint z

% Prepare plot
figure;

while true
    pose1.getPosition().setX(-r);
    pose1.getPosition().setY(0);
    pose2.getPosition().setX(r);
    pose2.getPosition().setY(0);

    if toc() > w
        pose1.getPosition().setX(cos(toc()*(0.3))*r);
        pose1.getPosition().setY(sin(toc()*(0.3))*r);
        pose2.getPosition().setX(-cos(toc()*(0.3))*r);
        pose2.getPosition().setY(-sin(toc()*(0.3))*r);
    end
    pose1.getPosition().setZ(z);
    pose2.getPosition().setZ(z);
    pose1.getOrientation().setW(1);
    pose2.getOrientation().setW(1);
    
    clf;
    hold on;
    
    plot(pose1.getPosition().getX(),pose1.getPosition().getY(),'rd');
    plot(pose2.getPosition().getX(),pose2.getPosition().getY(),'gd');
    axis([-1.5 1.5 -1.5 1.5]);
    
    pause(0.1);
    pose_pub1.publish(pose1);
    pose_pub2.publish(pose2);
end