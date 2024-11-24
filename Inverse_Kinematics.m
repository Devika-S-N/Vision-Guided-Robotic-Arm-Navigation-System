% Path to the URDF file
urdfFile = 'E:\RASLAB\new_trial\structure\my_pro600.urdf';

% Load the robot model
robot = importrobot(urdfFile);
robot.DataFormat = 'row';
robot.Gravity = [0, 0, -9.81];

% Define the end effector
endEffector = 'link6';

% Prompt user input for start and end positions
disp('Enter start position and orientation:');
startPosition = input('Start position [x, y, z]: '); % E.g., [-0.315, -0.291, 0.01]
startOrientationDeg = input('Start orientation [roll, pitch, yaw] (degrees): '); % E.g., [-178, 0, 0]

disp('Enter end position and orientation:');
endPosition = input('End position [x, y, z]: '); % E.g., [-0.242, -0.250, 0.01]
endOrientationDeg = input('End orientation [roll, pitch, yaw] (degrees): '); % E.g., [-178, 0, 0]

% Convert orientations to radians
startOrientation = deg2rad(startOrientationDeg);
endOrientation = deg2rad(endOrientationDeg);

% Create an inverse kinematics solver
ik = inverseKinematics('RigidBodyTree', robot);

% Apply solver settings based on `iksolver` from the Inverse Kinematics Designer
ik.SolverAlgorithm = 'BFGSGradientProjection';
ik.SolverParameters.MaxIterations = 1500;
ik.SolverParameters.GradientTolerance = 1e-6;
ik.SolverParameters.SolutionTolerance = 1e-6;

% Weights for position and orientation
weights = [1, 1, 1, 0.1, 0.1, 0.1];

% Initial guess for joint configuration
initialGuess = robot.homeConfiguration;

% Calculate inverse kinematics for the start position
startTform = trvec2tform(startPosition) * eul2tform(startOrientation, 'XYZ');
[configStart, startInfo] = ik(endEffector, startTform, weights, initialGuess);

% Ensure configStart is handled as a row array
if isa(configStart, 'struct') && isfield(configStart, 'JointPosition')
    startJointAngles = rad2deg(cell2mat(arrayfun(@(c) c.JointPosition, configStart, 'UniformOutput', false)));
else
    startJointAngles = rad2deg(configStart); % For matrix output
end

% Display the start configuration
disp('Start Joint Angles (degrees):');
disp(startJointAngles);

% Calculate inverse kinematics for the end position
endTform = trvec2tform(endPosition) * eul2tform(endOrientation, 'XYZ');
[configEnd, endInfo] = ik(endEffector, endTform, weights, initialGuess);

% Ensure configEnd is handled as a row array
if isa(configEnd, 'struct') && isfield(configEnd, 'JointPosition')
    endJointAngles = rad2deg(cell2mat(arrayfun(@(c) c.JointPosition, configEnd, 'UniformOutput', false)));
else
    endJointAngles = rad2deg(configEnd); % For matrix output
end

% Display the end configuration
disp('End Joint Angles (degrees):');
disp(endJointAngles);

% Optional: Display solver info for debugging
disp('Solver Info for Start Configuration:');
disp(startInfo);
disp('Solver Info for End Configuration:');
disp(endInfo);

% Visualize both configurations in the same figure
figure;
show(robot, configStart, 'PreservePlot', false); % Start configuration
title('Robot Configuration at Start and End Positions');
hold on;

show(robot, configEnd, 'PreservePlot', true); % End configuration
legend('Start Position', 'End Position');
hold off;