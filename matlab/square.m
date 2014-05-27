core = 'http://ramflight-arena:11311';
topic = 'setpoint';

node = rosmatlab.node('matlab_square',core);
pose_pub = rosmatlab.publisher(topic, 'geometry_msgs/Point', node);
pose = rosmatlab.message('geometry_msgs/Point',node);
tic;
l = 10;

while true
    pose.setX(0);
    pose.setY(0);
    if toc() > 1*l
        pose.setX(-0.4);
        pose.setY(-0.4);
    end
    if toc() > 2*l
        pose.setX(-0.4);
        pose.setY(0.4);
    end
    if toc() > 3*l
        pose.setX(0.4);
        pose.setY(0.4);
    end
    if toc() > 4*l
        pose.setX(0.4);
        pose.setY(-0.4);
    end
    pose.setZ(1);
    pause(0.5);
    disp('Publishing!')
    pose_pub.publish(pose);
end