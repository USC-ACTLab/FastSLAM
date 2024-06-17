%% FastSLAM numerical simulation
clear, clc, close all;

%% Part 1: Measurement Prediction Function
rob_starting_pose = [0 0 0]'; % x, y, heading
landmark_loc = [0 1]';

test_meas_1 = measFunc([1 0 0]', [0.02 1.11]');
test_meas_2 = measFunc([1 0 0]', [1.76,2.15]');
test_meas_3 = measFunc([1 0 0]', [0.78,-1.37]');
test_meas_4 = measFunc([1 0 0]',[1.2, -0.7]');
test_meas_5 = measFunc([1 0 0]',[-0.16,0]');
test_meas_6 = measFunc([1 0 1.94]',[-0.16, 0]');

%% Part 2: Measurement Model Prediction

syms x_land y_land x_rob y_rob theta_rob

range = sqrt((x_land - x_rob)^2 + (y_land - y_rob)^2);
theta = atan((y_land - y_rob)/(x_land - x_rob)) - theta_rob;

G = simplify(jacobian([range, theta],[x_land, y_land]));
G(2,1) = -(y_land - y_rob) / ((y_land-y_rob)^2 + (x_land-x_rob)^2);
G(2,2) = (x_land - x_rob) / ((y_land-y_rob)^2 + (x_land-x_rob)^2);

G1 = eval(subs(G, [x_land, y_land, x_rob, y_rob, theta_rob], [1, 0, 1, 1, 0]))
G2 = eval(subs(G, [x_land, y_land, x_rob, y_rob, theta_rob], [0, 1, 1, 0, 0]))
G3 = eval(subs(G, [x_land, y_land, x_rob, y_rob, theta_rob], [0, 1, 1, 1, 0]))

%% Function declarations

function meas = measFunc(rob_pose, lm_pose)
    meas(1) = norm(rob_pose(1:2) - lm_pose, 2);
    reco_angle = atan((lm_pose(2) - rob_pose(2))/(lm_pose(1) - rob_pose(1)));
    if (lm_pose(1) - rob_pose(1) < 0)
        meas(2) = reco_angle + pi - rob_pose(3);
        meas(2) = meas(2) - 2*pi*floor(meas(2)/pi);
    else
        meas(2) = reco_angle - rob_pose(3);
    end
end