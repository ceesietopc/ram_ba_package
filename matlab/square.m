core = 'http://ramflight-arena:11311';
topic = 'setpoint';

node = rosmatlab.node('matlab_square',core);
pose_pub = rosmatlab.publisher(topic, 'geometry_msgs/Pose', node);
pose = rosmatlab.message('geometry_msgs/Pose',node);
tic;
l = 10;

while true
    pose.getPosition().setX(0);
    pose.getPosition().setY(0);
    if toc() > 1*l
        pose.getPosition().setX(-0.4);
        pose.getPosition().setY(-0.4);
    end
    if toc() > 2*l
        pose.getPosition().setX(-0.4);
        pose.getPosition().setY(0.4);
    end
    if toc() > 3*l
        pose.getPosition().setX(0.4);
        pose.getPosition().setY(0.4);
    end
    if toc() > 4*l
        pose.getPosition().setX(0.4);
        pose.getPosition().setY(-0.4);
    end
    pose.getPosition().setZ(1);
    pose.getOrientation().
    pause(0.5);
    disp('Publishing!')
    pose_pub.publish(pose);
end