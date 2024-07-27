%% Verify EKF one step update
% by Emma Slaght & Will Wu
clear; clc;

%% set up test
init_pose = [0; 0; 0]; %initial pose of the robot [x ; y; theta]
init_sigma = eye(2);

%current belief of where landmark is, POLAR [range, bearing] -- in ROBOT frame
p_init_obs = [1; pi/2]; 
c_init_obs = [p_init_obs(1) * cos(p_init_obs(2)); 
              p_init_obs(1) * sin(p_init_obs(2))];

% current belief of landmark position in world frame
prev_lm_belief = [c_init_obs(1) + init_pose(1) ; c_init_obs(2) + init_pose(2)];

vel_cmd = [1; 0; 0];
new_pose = [(vel_cmd(1) * cos(init_pose(3))) + init_pose(1) ; 
            (vel_cmd(1) * sin(init_pose(3))) + init_pose(2) ; 
             vel_cmd(2) + init_pose(3)];

% hypothetical new obs after movement, in polar, robot frame
p_new_obs = [1.5; pi/4]; 

% what the robot thinks it will see based on current position and prev
% landmark belief, in polar
predict_meas = predictMeas(new_pose, prev_lm_belief);

R = [1, 0; 0, 1]; % measurement noise

%% Filter Update
G = measJacobian(prev_lm_belief, new_pose); %jacobian
residue = p_new_obs - predict_meas;


Q = (G' * init_sigma * G) + R; %q corrected, which is m_meas_cov in code

K = init_sigma * G * (inv(Q)); %kalman gain corrected
update_belief = prev_lm_belief + (K * (residue)); %potential transpose?
update_sigma = (eye(2) - (K * G')) * init_sigma; %updating cov

cpd = (1 / sqrt(det(2*pi*Q)))* exp(-0.5 * (residue') * inv(Q) * residue); %corrected

disp('expected state estimate:');
disp(update_belief);
disp('expected cov matrix: ');
disp(update_sigma);
disp('cpd');
disp(cpd);

%% function declarations
function G = measJacobian(lm1, new_pose1)
    dx = lm1(1) - new_pose1(1);
   dy = lm1(2) - new_pose1(2);
    G = zeros(2,2);
    if dx == 0 && dy == 0
        G(:) = NaN;
    else 
        G(1,1) = dx / sqrt(dx^2 +dy^2);
        G(1,2) = dy / sqrt(dx^2 +dy^2);
        G(2,1) = -dy / (dx^2 +dy^2);
        G(2,2) = dx / (dx^2 +dy^2);
    end
end

function predict_meas = predictMeas(rob_pose, lm_pose)
    predict_meas(1,1) = norm(rob_pose(1:2) - lm_pose, 2);
    reco_angle = atan((lm_pose(2) - rob_pose(2))/(lm_pose(1) - rob_pose(1)));
    if (lm_pose(1) - rob_pose(1) < 0)
        predict_meas(2,1) = reco_angle + pi - rob_pose(3);
        predict_meas(2,1) = predict_meas(2) - 2*pi*floor(predict_meas(2)/pi);
    else
        predict_meas(2,1) = reco_angle - rob_pose(3);
    end
end
