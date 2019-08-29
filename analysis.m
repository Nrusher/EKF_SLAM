close all
clear all;

% load('sim_result_iekf_map3.mat')
load('sim_result.mat')
IEKF_pre_trajectory = sim_result.EKF_pre_trajectory;

load('sim_result_color_noise_map3.mat')
EKF_pre_trajectory_color = sim_result.EKF_pre_trajectory;

load('sim_result_asy_map3.mat')
EKF_pre_trajectory_asy = sim_result.EKF_pre_trajectory;

load('sim_result_less_landmark_map3.mat')
EKF_pre_trajectory_ls = sim_result.EKF_pre_trajectory;

% load('sim_result_ekf_map3.mat')
% load('sim_result_less_landmark_map3.mat')
load('sim_result_color_noise_map3.mat')
ture_trajectory = sim_result.ture_trajectory;
model_pre_trajectory = sim_result.model_pre_trajectory;
EKF_pre_trajectory = sim_result.EKF_pre_trajectory;
landmarks = sim_result.landmarks;
wp = sim_result.wp;

length = size(ture_trajectory,2);

figure
axis equal
hold on;
% 画出landmarks
scatter( landmarks(1, :), landmarks(2, :), 'b*' )

% 画出历史轨迹
truep = plot( ture_trajectory(1, :), ture_trajectory(2, :), 'k--','linewidth',3);
    
% 画出历史EKF预测轨迹
ekfp = plot( EKF_pre_trajectory(1, :), EKF_pre_trajectory(2, :), 'r','linewidth',3 );
    
% 画出历史model预测轨迹
modelp = plot( model_pre_trajectory(1, :), model_pre_trajectory(2, :), 'b-.','linewidth',3 );

% 画出车的位姿
xtrue = ture_trajectory(:,length-1);
draw_car(xtrue,5,'k');
    
% EKF预测位姿
x = EKF_pre_trajectory(:,length-1);
draw_car(x,5,'r');
    
% 模型预测位姿
x_model_pre = model_pre_trajectory(:,length-1);
draw_car(x_model_pre,5,'g');

%图例
legend([truep ekfp,modelp],'true','ekf','model');

hold off;

e_ekf = EKF_pre_trajectory - ture_trajectory;
e_ekf(3,:) = pi_to_pi(e_ekf(3,:));
e_model = model_pre_trajectory - ture_trajectory;
e_model(3,:) = pi_to_pi(e_model(3,:));

e_ekf = abs(e_ekf);
e_model = abs(e_model);

fig2 = figure;
hold on
ticks = 1:1:length;

subplot(3,1,1)
hold on
plot(ticks,e_ekf(1,:),'linewidth',2);
plot(ticks,e_model(1,:),'linewidth',2);
xlabel("ticks")
ylabel("|x-error|")

subplot(3,1,2)
hold on
plot(ticks,e_ekf(2,:),'linewidth',2);
plot(ticks,e_model(2,:),'linewidth',2);
xlabel("ticks")
ylabel("|y-error|")

subplot(3,1,3)
hold on
plot(ticks,pi_to_pi(e_ekf(3,:)),'linewidth',2);
plot(ticks,pi_to_pi(e_model(3,:)),'linewidth',2);
xlabel("ticks")
ylabel("|\phi-error|")
legend('ekf-error','model-error');

ekf_eN2 = zeros(1,length);
model_eN2 = zeros(1,length);
for i=1:1:length
    ekf_eN2(i) = norm(e_ekf(:,i),2);
    model_eN2(i) = norm(e_model(:,i),2);
end
hold off

fig3 = figure;
hold on
plot(ticks,ekf_eN2,'linewidth',2);
plot(ticks,model_eN2,'linewidth',2);
xlabel("ticks")
ylabel("error")
legend('ekf-error','model-error');

e_ekf_ls = EKF_pre_trajectory_ls - ture_trajectory;
e_ekf_ls(3,:) = pi_to_pi(e_ekf_ls(3,:));
e_ekf_ls = abs(e_ekf_ls);

fig4 = figure;
hold on
ticks = 1:1:length;

subplot(3,1,1)
hold on
plot(ticks,e_ekf(1,:),'linewidth',2);
plot(ticks,e_ekf_ls(1,:),'linewidth',2);
xlabel("ticks")
ylabel("|x-error|")

subplot(3,1,2)
hold on
plot(ticks,e_ekf(2,:),'linewidth',2);
plot(ticks,e_ekf_ls(2,:),'linewidth',2);
xlabel("ticks")
ylabel("|y-error|")

subplot(3,1,3)
hold on
plot(ticks,pi_to_pi(e_ekf(3,:)),'linewidth',2);
plot(ticks,pi_to_pi(e_ekf_ls(3,:)),'linewidth',2);
xlabel("ticks")
ylabel("|\phi-error|")
legend('ekf','ekf-less-observation');

ekf_ls_eN2 = zeros(1,length);
for i=1:1:length
    ekf_ls_eN2(i) = norm(e_ekf_ls(:,i),2);
end

fig5 = figure;
hold on
plot(ticks,ekf_eN2,'linewidth',2);
plot(ticks,ekf_ls_eN2,'linewidth',2);
xlabel("ticks")
ylabel("error")
legend('ekf-error','ekf-less-observation');

fig6 = figure;
axis equal
hold on;
% 画出landmarks
scatter( landmarks(1, :), landmarks(2, :), 'b*' );

% 画出历史轨迹
truep = plot( ture_trajectory(1, :), ture_trajectory(2, :), 'k--','linewidth',3);
    
% 画出历史EKF预测轨迹
ekfp = plot( EKF_pre_trajectory(1, :), EKF_pre_trajectory(2, :), 'r','linewidth',3 );
ekfpls = plot( EKF_pre_trajectory_ls(1, :), EKF_pre_trajectory_ls(2, :), 'b-.','linewidth',3);

%图例
legend([truep,ekfp ekfpls],'true','ekf','ekf-less-observation');

fig7 = figure;
axis equal
hold on;
% 画出landmarks
scatter( landmarks(1, :), landmarks(2, :), 'b*' );

% 画出历史轨迹
truep = plot( ture_trajectory(1, :), ture_trajectory(2, :), 'k--','linewidth',3);
    
% 画出历史EKF预测轨迹
ekfp = plot( EKF_pre_trajectory(1, :), EKF_pre_trajectory(2, :), 'r','linewidth',3 );
ekfasyp = plot( EKF_pre_trajectory_asy(1, :), EKF_pre_trajectory_asy(2, :), 'b-.','linewidth',3);

%图例
legend([truep ekfp,ekfasyp],'true','ekf','ekf-asynchronous');

fig8 = figure;
axis equal
hold on;
% 画出landmarks
scatter( landmarks(1, :), landmarks(2, :), 'b*' );

% 画出历史轨迹
truep = plot( ture_trajectory(1, :), ture_trajectory(2, :), 'k--','linewidth',3);
    
% 画出历史EKF预测轨迹
ekfp = plot( EKF_pre_trajectory(1, :), EKF_pre_trajectory(2, :), 'r','linewidth',3 );
ekfcolorp = plot( EKF_pre_trajectory_color(1, :), EKF_pre_trajectory_color(2, :), 'b-.','linewidth',3);

%图例
legend([truep ekfp,ekfcolorp],'true','ekf','ekf-color');

fig9 = figure;
axis equal
hold on;
% 画出landmarks
scatter( landmarks(1, :), landmarks(2, :), 'b*' );

% 画出历史轨迹
truep = plot( ture_trajectory(1, :), ture_trajectory(2, :), 'k--','linewidth',3);
    
% 画出历史EKF预测轨迹
ekfp = plot( EKF_pre_trajectory(1, :), EKF_pre_trajectory(2, :), 'r','linewidth',3 );
iekfcolorp = plot( IEKF_pre_trajectory(1, :), IEKF_pre_trajectory(2, :), 'b-.','linewidth',3);

%图例
legend([truep ekfp,iekfcolorp],'true','ekf','iekf');

e_iekf = IEKF_pre_trajectory - ture_trajectory;
e_iekf(3,:) = pi_to_pi(e_iekf(3,:));
e_iekf = abs(e_iekf);

fig10 = figure;
hold on
ticks = 1:1:length;

subplot(3,1,1)
hold on
plot(ticks,e_ekf(1,:),'linewidth',2);
plot(ticks,e_iekf(1,:),'linewidth',2);
xlabel("ticks")
ylabel("|x-error|")

subplot(3,1,2)
hold on
plot(ticks,e_ekf(2,:),'linewidth',2);
plot(ticks,e_iekf(2,:),'linewidth',2);
xlabel("ticks")
ylabel("|y-error|")

subplot(3,1,3)
hold on
plot(ticks,pi_to_pi(e_ekf(3,:)),'linewidth',2);
plot(ticks,pi_to_pi(e_iekf(3,:)),'linewidth',2);
xlabel("ticks")
ylabel("|\phi-error|")
legend('ekf-error','iekf-error');

iekf_eN2 = zeros(1,length);
for i=1:1:length
    iekf_eN2(i) = norm(e_iekf(:,i),2);
end
hold off

fig11 = figure;
hold on
plot(ticks,ekf_eN2,'linewidth',2);
plot(ticks,iekf_eN2,'linewidth',2);
xlabel("ticks")
ylabel("error")
legend('ekf-error','iekf-error');


% mean(model_eN2(:))
% mean(ekf_eN2(:))
% mean(ekf_ls_eN2(:))

% mean(e_ekf_ls(3,:))
% mean(e_ekf_ls(2,:))
% mean(e_ekf_ls(1,:))
% 
% mean(e_ekf(3,:))
% mean(e_ekf(2,:))
% mean(e_ekf(1,:))




