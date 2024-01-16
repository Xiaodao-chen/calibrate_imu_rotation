clc;clear;close

% Specify the path to your ROS bag file
bagFilePath = 'name.bag';
bag = rosbag(bagFilePath);

% change to your topic name 
odom_msgs = select(bag,'Topic','/ekf/ekf_odom');
sensor_msgs = select(bag,'Topic','/mavros/imu/data_raw');

%parse the struct
odom_structs = readMessages(odom_msgs,'DataFormat','struct');
imu_structs  = readMessages(sensor_msgs,'DataFormat','struct'); 


X_acc =  cellfun(@(m) double(m.LinearAcceleration.X), imu_structs);
Y_acc =  cellfun(@(m) double(m.LinearAcceleration.Y), imu_structs);
Z_acc =  cellfun(@(m) double(m.LinearAcceleration.Z), imu_structs);

num = length(X_acc);
gravity=9.78;

% change it to your city 
z_w = [0,0,1];

acc  = [X_acc,Y_acc,Z_acc];
zb = sum(acc,1)/gravity/num;

% axis to rotation 
axis_of_rotation = normalize(cross(zb,z_w));


disp(zb);
cosTheta = dot(z_w,zb)/norm(zb)/norm(z_w);
angleInRadians = acos(cosTheta);


I = eye(3);

A = axis_of_rotation;
A_skew = [0, -A(3), A(2); 
          A(3), 0, -A(1); 
          -A(2), A(1), 0];
calibrate_imu_rotation = cos(angleInRadians)*I + (1-cos(angleInRadians))*axis_of_rotation*axis_of_rotation' + sin(angleInRadians)*A_skew;


disp(calibrate_imu_rotation);

inverse_A = inv(calibrate_imu_rotation);
disp(inverse_A);

determinant_A = det(calibrate_imu_rotation)
determinant_ivnerseA = det(inverse_A)