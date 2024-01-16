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

X = cellfun(@(m) double(m.Pose.Pose.Orientation.X), odom_structs);
Y = cellfun(@(m) double(m.Pose.Pose.Orientation.Y), odom_structs);
Z = cellfun(@(m) double(m.Pose.Pose.Orientation.Z), odom_structs);
W = cellfun(@(m) double(m.Pose.Pose.Orientation.W), odom_structs);

X_acc =  cellfun(@(m) double(m.LinearAcceleration.X), imu_structs);
Y_acc =  cellfun(@(m) double(m.LinearAcceleration.Y), imu_structs);
Z_acc =  cellfun(@(m) double(m.LinearAcceleration.Z), imu_structs);


% Create a quaternion array from separate scalar and vector parts
num = length(X);
quat =  quaternion(W,X,Y,Z);
acc  = [X_acc,Y_acc,Z_acc];

% change to your city gravity 
gravity = [0,0, 9.87];

% Initial guess for calibration parameters
initialGuess = eye(3);
% calibratedParameters =zeros(3,3);




% Perform the optimization
% Options for lsqnonlin
options = optimoptions('lsqnonlin', 'TolFun', 1e-6);
options = optimoptions('lsqnonlin', 'Algorithm', 'levenberg-marquardt');
options = optimoptions('lsqnonlin', 'StepTolerance', 1e-10);


optimize_param = lsqnonlin(@(calibrationParameters)costfunction(calibrationParameters,quat,acc,gravity,2000),initialGuess,[],[],options);
disp("optimize_param: ");
disp( optimize_param);

% quat2rotm(quat(1))*acc(1,:)';
beforecost =  totalcostfunction(initialGuess,quat,acc,gravity,num);
disp(beforecost);
costtotal  =  totalcostfunction(optimize_param,quat,acc,gravity,num);
disp(costtotal);


function cost = costfunction(calibrationParameters,quat,acc,gravity,num)
    % Implement your calibration model here
    cost= zeros(num,1);
    for i = 1:num
        vector_error  = (gravity' - calibrationParameters*quat2rotm(quat(i))*acc(i,:)');

        cost(i,1) = norm(vector_error);
        % error status 
        if cost(i,1) > 3
            cost(i,1) = 0;
        end
    end
    
end

function cost = totalcostfunction(calibrationParameters,quat,acc,gravity,num)
    % Implement your calibration model here
    cost= 0 ;
    for i = 1:num
        vector_error  = (gravity' - calibrationParameters * quat2rotm(quat(i)) * acc(i,:)');
        % you can change to your steps
        if norm(vector_error) < 3
            cost = cost+ norm(vector_error);
        end
    end
end



