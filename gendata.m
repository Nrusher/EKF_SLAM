function data_original = gendata( do_vis )
%
% 用于产生EKF_SLAM仿真所需要的数据。
%
% Inputs:
%   do_vis - 是否进行可视化
%
% Outputs: 
%   data_original - 产生的数据，各项含义如下
%       key_points - 地图关键点
%       lamdmarks - landmark的坐标
%       states.G - 真实舵角
%       states.V - 真实速度
%       states.Gn - 带有噪声的舵角
%       states.Vn - 带有噪声的速度
%       states.xtrue - 真实的位姿
%       states.ftag_visible - 可观测到的landmark
%       states.ztrue - 真实的landmark观测值
%       states.zn - 带有噪声的landmark观测值
%       states.observation_update - 是否进行了观测数据更新
%       states.next_keypoint - 下一目标点
%
% 纪友州 2019

% 清除数据
clear all

% 加载模型参数
simulation_config

% 无参数，默认可视化
if nargin < 1
    do_vis = 1;
end

% 产生运动轨迹的关键点
t = 0:0.8:6.28;
key_points = gen_trajectory2d( t );
data_original.key_points = key_points;

% 产生landmarks
n_landmarks = N_LANDMARKS;
border = [BORDER_LENGTH, BORDER_LENGTH];
minpos = min((key_points'));
maxpos = max((key_points'));
minlm = minpos - border;
maxlm = maxpos + border;
landmarks(1, :) = minlm(1)+rand(n_landmarks, 1)*(maxlm(1)-minlm(1));
landmarks(2, :) = minlm(2)+rand(n_landmarks, 1)*(maxlm(2)-minlm(2));

data_original.landmarks = landmarks;

% 控制频率
dt = DT_CONTROLS;
% 初始目标关键点
iwp= 1;
% 初始舵角
G= 0;
% 初始位姿
xtrue = [key_points(:,1);135*pi/180];

true_states = zeros(5,1000000);
wp = key_points;
dtsum= DT_OBSERVE+0.0001;
ftag= 1:size(landmarks,2);

if SAVE_GIF == 1

    if exist('gen_data.avi','file') == 2
        delete(gen_data.avi);
    end
    
    if exist('gen_data.gif','file') == 2
        delete(gen_data.avi);
    end
    
    %创建avi文件对象
    aviobj = VideoWriter('gen_data.avi','Uncompressed AVI');
    open(aviobj)
end



fig=figure;
i = 1;
k = 1;
while iwp ~= 0
    
    % 更新真实舵角
    [G,iwp]= compute_steering(xtrue, wp, iwp, AT_WAYPOINT, G, RATEG, MAXG, dt);
    
    % 判断是否跑完所有圈数
    if iwp==0 && NUMBER_LOOPS > 1 
        iwp=1; 
        NUMBER_LOOPS= NUMBER_LOOPS-1; 
    end 
    
    % 更新车的真实位置
    xtrue= vehicle_model(xtrue, V,G, WHEELBASE,dt);
    
    % 更新采集到的控制量
    [Vn,Gn]= add_control_noise(V,G,Q, SWITCH_CONTROL_NOISE);
    
    ob = 0;
    %真实landmark观测值
    dtsum= dtsum + dt;
    if dtsum >= DT_OBSERVE
        dtsum= 0;
        % 检测周围landmark
        [z,ftag_visible]= get_observations(xtrue, landmarks, ftag, MAX_RANGE);
        ztrue = z;
        % 添加观测噪声
        z = add_observation_noise(z,R, SWITCH_SENSOR_NOISE);
        
        % 真实数据
        true_state = [xtrue;V;G];
        true_states(:,i) = true_state;

        ob = 1;
        
        
        i = i + 1;   
    end
    
    % 采集数据
    state.G = G;
    state.V = V;
    state.xtrue = xtrue;
    state.Vn = Vn;
    state.Gn = Gn;
    state.ztrue = ztrue;
    state.ftag_visible = ftag_visible;
    state.zn = z;
    state.observation_update = ob;
    state.next_keypoint = iwp;
    data_original.states(k) = state;
    
    % 可视化操作
    if do_vis == 1
        % 清除图像
        cla;
        hold on;
        scatter( landmarks(1, :), landmarks(2, :), 'b*' ); 
        axis equal;
        plot( wp(1,:), wp(2, :), 'r.','markersize',26 );
    
        % 画出车的位姿
        draw_car(xtrue,5,'k');
    
        % 画出下一目标点的位置
        if iwp~=0
            plot(wp(1,iwp),wp(2,iwp),'bo','markersize',13,'linewidth',1);
        end
    
        % 画出激光雷达观测范围
        draw_circle(xtrue(1), xtrue(2),MAX_RANGE);
    
        % 画出激光雷达观测线
        plines = make_laser_lines(z,xtrue);
        if  ~isempty(plines)
            plot(plines(1,:),plines(2,:));
        end
    
        % 画出历史轨迹
        plot( true_states(1, 1:i-1), true_states(2, 1:i-1), 'k--','linewidth',3 );
 
        pause(0.00001); 
        
        if SAVE_GIF == 1
            %获取当前画面
            F = getframe(fig);
            %加入avi对象中
            writeVideo(aviobj,F);
            
            %转成gif图片,只能用256色
            im = frame2im(F);
            [I,map] = rgb2ind(im,256);
            %写入 GIF89a 格式文件   
            if k == 1
                imwrite(I,map,'gen_data.gif','GIF', 'Loopcount',inf,'DelayTime',0.1);
            else
                imwrite(I,map,'gen_data.gif','GIF','WriteMode','append','DelayTime',0.1);
            end
        end
        
        k = k+1;
    end
end
save data data_original 