function  myplotResult(DATAdir,cfg)
    % 自己位置推定（CR_debug）の記録を描く
    %   NDT の情報（/localization_debug，sensor(7)）：位置合わせの時間・angle_diff（受け取った周期だけ）
    %   Plant（Matching，sensor(5)）：/localization/base_pose の後輪間中心の姿勢（周期ごと．現在時刻まで予測した値）．
    %     2026-10-26 より前の記録は /ekf_pose（LiDAR の姿勢）
    %   LIDAR_pose（sensor(8)）：/localization/vlp16_pose の点群の時刻の LiDAR の姿勢（新しい組を受け取った周期だけ．
    %     横軸は受け取った周期の時刻）
    %   LiDAR_age：受け取った直後の ROS の時刻（data.Now）と使った点群の stamp の差（点群の古さ）
    % 線は後に描くほど細くし，点線・破線は使わない．枠は 4 辺とも黒
    %% Settings
    % warning('off','all');
    % 読み込み
    if nargin < 2
        clear variables; close all; clc;
        tmp     = matlab.desktop.editor.getActive;
        DATAdir = cd(fileparts(tmp.Filename));

        % File  = strcat(DATAdir, '/info.mat');
        % load(File, 'cfg');
    end

    File  = strcat(DATAdir, '/userLocal_tmp.mat');
    load(File, 'tmp');

    File  = strcat(DATAdir, '/userLocal.mat');
    load(File, 'userLocal');

    %% make folda
    Outputdir = strcat(DATAdir,'/result');
    mkdir(Outputdir,'jpeg');
    mkdir(Outputdir,'eps');
    mkdir(Outputdir,'fig');
    mkdir(Outputdir,'png');
    mkdir(Outputdir,'pdf');

    %% データ処理
    tmp_cat=cat(1,tmp{:,1});
    tmp_Estimator=cat(1,tmp_cat.Estimator);
    % 周期ごとの RawData はフィールドがそろわないことがある（古い記録の NDT_info）ので cell のまま扱う
    raw = {tmp_Estimator.RawData}';

    T = seconds(userLocal.Time)-seconds(userLocal.Time(1));

    % NDT の情報（受け取った周期だけ）
    hasNdt = cellfun(@(r) isfield(r,'NDT_info') && ~isempty(r.NDT_info), raw);
    tmp_NDT_info = cellfun(@(r) r.NDT_info, raw(hasNdt), 'UniformOutput', false);
    tmp_NDT_info = [tmp_NDT_info{:}];
    Tn = T(hasNdt);
    if ~isempty(tmp_NDT_info)
        ndt.downsampled_points = [tmp_NDT_info.downsampled_points]';
        ndt.matching_time = [tmp_NDT_info.matching_time]';
        ndt.convergence = [tmp_NDT_info.convergence]';
        ndt.score = [tmp_NDT_info.score]';
        ndt.angle_diff = [tmp_NDT_info.angle_diff]';
    end

    % Plant．受け取る前の周期は 0 なので除き，yaw は ±pi で折り返さないようにつなぐ．平面の位置は最初の値からの差にする
    state = [userLocal.X,userLocal.Y,userLocal.Z,...
            userLocal.Roll,userLocal.Pitch,userLocal.Yaw];
    first = find(any(state(:,1:3) ~= 0, 2), 1);
    if isempty(first), first = 1; end
    Tp = T(first:end);
    state = state(first:end,:);
    state(:,6) = unwrap(state(:,6));
    origin = [state(1,1:2),0,0,0,0];
    state = state - origin;

    % LiDAR の姿勢（新しい組を受け取った周期だけ．平面の位置は Plant と同じ原点）
    hasLidarPose = any(cellfun(@(r) isfield(r,'LIDAR_pose'), raw));
    isNewPair = cellfun(@(r) isfield(r,'LIDAR_pose') && ~isempty(r.LIDAR_pose) && r.LIDAR_new, raw);
    lp = cellfun(@(r) r.LIDAR_pose, raw(isNewPair), 'UniformOutput', false);
    lp = [lp{:}];
    if ~isempty(lp)
        Tl = T(isNewPair);
        lidar = [[lp.X]',[lp.Y]',[lp.Z]',[lp.Roll]',[lp.Pitch]',unwrap([lp.Yaw]')] - origin;
        stampL = [lp.Stamp]';
    end
    if hasLidarPose
        plantName = 'Base pose (rear axle center)';
    else
        plantName = 'Plant';   % 2026-10-26 より前の記録（/ekf_pose，LiDAR の姿勢）
    end

    %% プロット
    if ~isempty(tmp_NDT_info)
        plotWithAverage(Tn, ndt.matching_time, 'Matching time', 'Matching Time [s]');
        saveFigure('ndt_matching_time', Outputdir)

        plotWithAverage(Tn, ndt.angle_diff, 'Angle diff', 'Angle diff [deg]');
        saveFigure('ndt_angle_diff', Outputdir)
    end

    plotPosition(Tp, state, plantName);
    saveFigure('Position', Outputdir)

    plotAngle(Tp, state, plantName);
    saveFigure('Angle', Outputdir)

    if ~isempty(lp)
        plotPosition(Tl, lidar, 'LiDAR pose at the cloud stamp');
        saveFigure('Position_LiDAR', Outputdir)

        plotAngle(Tl, lidar, 'LiDAR pose at the cloud stamp');
        saveFigure('Angle_LiDAR', Outputdir)

        % 平面の軌跡：後輪間中心と LiDAR（LiDAR は後輪間中心の周りを取付の距離で回る）
        C = palette();
        newFigure();
        plot(state(:,1), state(:,2), '-', 'Color', C(1,:), 'LineWidth', 2.4);
        plot(lidar(:,1), lidar(:,2), '-', 'Color', C(2,:), 'LineWidth', 1.4);
        axis equal
        xlabel('X [m]')
        ylabel('Y [m]')
        legend(plantName, 'LiDAR pose at the cloud stamp', 'Location', 'best')
        saveFigure('Trajectory', Outputdir)

        % 新しい組の間隔（点群の stamp の差）．LiDAR の周期より長いところは点群か姿勢を受け取れなかった
        newFigure();
        plot(Tl(2:end), diff(stampL)*1e3, '-', 'Color', C(2,:), 'LineWidth', 1.6);
        xlabel('Time [s]')
        ylabel('Interval between new LiDAR pairs [ms]')
        xlim(T([1,end],:)')
        saveFigure('LiDAR_pair_interval', Outputdir)
    end

    % 点群の古さ：受け取った直後の ROS の時刻（data.Now）− 使った点群の stamp（周期ごと．新しい組を受け取った周期は点）
    hasAge = cellfun(@(r) isfield(r,'Now') && ~isempty(r.Now) && ~isempty(r.LIDAR), raw);
    if any(hasAge)
        C = palette();
        Ta = T(hasAge);
        age = cellfun(@(r) r.Now - (double(r.LIDAR.header.stamp.sec) + double(r.LIDAR.header.stamp.nanosec)*1e-9), ...
            raw(hasAge)) * 1e3;
        isNew = cellfun(@(r) isfield(r,'LIDAR_new') && r.LIDAR_new, raw(hasAge));
        newFigure();
        plot(Ta, age, '-', 'Color', C(1,:), 'LineWidth', 1.6);
        plot(Ta(isNew), age(isNew), 'o', 'Color', C(2,:), 'MarkerFaceColor', C(2,:), 'MarkerSize', 3);
        xlabel('Time [s]')
        ylabel('Current time - point cloud stamp [ms]')
        xlim(T([1,end],:)')
        legend(sprintf('Every cycle (mean %.0f ms)', mean(age)), ...
            sprintf('New pair received (mean %.0f ms)', mean(age(isNew))), 'Location', 'best')
        saveFigure('LiDAR_age', Outputdir)
    end
end


function C = palette()
    % dataviz の参照パレット（localization_tools の図と同じ）
    C = [42 120 214; 235 104 52; 27 175 122; 237 161 0]/255;
end

function ax = newFigure()
    figure
    ax = gca;
    box on; hold on;
    ax.XColor = 'k';
    ax.YColor = 'k';
end

function plotWithAverage(T, data, name, yName)
    C = palette();
    newFigure();
    plot(T, data, '-', 'Color', C(1,:), 'LineWidth', 2.0);
    [~, av] = plot_average(T, data);
    xlabel('Time [s]')
    ylabel(yName)
    xlim(T([1,end],:)')
    legend(name, sprintf('Average %.3g', av), 'Location', 'best')
end

function plotPosition(T, s, name)
    C = palette();
    ax = newFigure();
    yyaxis left
    plot(T, s(:,1), '-', 'Color', C(1,:), 'LineWidth', 2.4);
    plot(T, s(:,2), '-', 'Color', C(2,:), 'LineWidth', 1.8);
    ylabel('Position (X, Y) [m]')
    yyaxis right
    plot(T, s(:,3), '-', 'Color', C(3,:), 'LineWidth', 1.2);
    ylabel('Position (Z) [m]')
    ax.YAxis(1).Color = 'k';
    ax.YAxis(2).Color = 'k';
    xlabel('Time [s]')
    xlim(T([1,end],:)')
    title(name)
    legend('X','Y','Z')
end

function plotAngle(T, s, name)
    C = palette();
    ax = newFigure();
    yyaxis left
    plot(T, s(:,4), '-', 'Color', C(1,:), 'LineWidth', 2.4);
    plot(T, s(:,5), '-', 'Color', C(2,:), 'LineWidth', 1.8);
    ylabel('Angle (Roll, Pitch) [rad]')
    yyaxis right
    plot(T, s(:,6), '-', 'Color', C(3,:), 'LineWidth', 1.2);
    ylabel('Angle (Yaw) [rad]')
    ax.YAxis(1).Color = 'k';
    ax.YAxis(2).Color = 'k';
    xlabel('Time [s]')
    xlim(T([1,end],:)')
    title(name)
    legend('Roll','Pitch','Yaw')
end

function [p,average] = plot_average(T,data)
    C = palette();
    average = sum(data)/length(data);
    p = plot(T([1,end],:),[average,average], '-', 'Color', C(2,:), 'LineWidth', 1.2);
end

function saveFigure(Name, Outputdir)
	hold off;
	saveas(gcf,strcat(Outputdir,strcat('/eps/',Name)),'epsc');
    saveas(gcf,strcat(Outputdir,strcat('/jpeg/',Name)),'jpeg');
	savefig(gcf,strcat(Outputdir,strcat('/fig/',Name)));
end
