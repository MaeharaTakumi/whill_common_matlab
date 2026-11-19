function  myplotResult(DATAdir,cfg)
    % 自己位置推定（CR_debug）の記録を描く
    %   NDT の情報（/localization_debug，sensor(5)．LIDAR_pose.NDT_info．2026-11-19 より前は sensor(7) の NDT_info）：位置合わせの時間・angle_diff（受け取った周期だけ）
    %   Plant（Matching，sensor(5)）：後輪間中心の姿勢（周期ごと．今の ROS の時刻の /tf map → base（ekf_localizer の予測値を補間），引けなければ最新の TF．2026-11-13 まではいつも最新の TF）．
    %     2026-10-26 の 62 より前の記録は /ekf_pose（LiDAR の姿勢），68 より前は /localization/base_pose
    %   LIDAR_pose（sensor(5)）：点群の stamp の時刻の LiDAR の姿勢（新しい点群を受け取った周期だけ．横軸はその周期の時刻）．
    %     今の時刻との差がしきい値以上のときと，その時刻の TF が引けないときは，今の時刻（引けなければ最新）の TF（LIDAR_pose.Stamp が点群の stamp と違う）．
    %     /tf を引いた予測値（68 より前の記録は /localization/vlp16_pose の NDT で更新した直後の値）
    %   LiDAR_age：今の ROS の時刻（Plant.Stamp．2026-11-19 より前は受け取った直後の data.Now）と使った点群の stamp の差（点群の古さ）
    %   新しい点群を受け取った周期は，点群の stamp が前の周期と違う周期
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
    hasNdt = cellfun(@(r) ~isempty(ndtOf(r)), raw);
    tmp_NDT_info = cellfun(@ndtOf, raw(hasNdt), 'UniformOutput', false);
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

    % 新しい点群を受け取った周期（点群の stamp が前の周期と違う）．点群の無い周期の stamp は NaN
    stampAll = cellfun(@cloudStamp, raw);
    isNewCloud = ~isnan(stampAll) & stampAll ~= fillmissing([NaN; stampAll(1:end-1)], 'previous');

    % LiDAR の姿勢（新しい点群を受け取った周期だけ．平面の位置は Plant と同じ原点）
    hasLidarPose = any(cellfun(@(r) isfield(r,'LIDAR_pose'), raw));
    isNewPair = isNewCloud & cellfun(@(r) isfield(r,'LIDAR_pose') && ~isempty(r.LIDAR_pose), raw);
    lp = cellfun(@(r) r.LIDAR_pose, raw(isNewPair), 'UniformOutput', false);
    lp = [lp{:}];
    if ~isempty(lp)
        Tl = T(isNewPair);
        lidar = [[lp.X]',[lp.Y]',[lp.Z]',[lp.Roll]',[lp.Pitch]',unwrap([lp.Yaw]')] - origin;
        % 点群の stamp（今の時刻や最新の TF を使った周期は lp.Stamp が点群の stamp と違う）
        stampL = stampAll(isNewPair);
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
        timeLim(T)
        saveFigure('LiDAR_pair_interval', Outputdir)
    end

    % 点群の古さ：今の ROS の時刻 − 使った点群の stamp（周期ごと．新しい点群を受け取った周期は点）．
    % 今の時刻は Plant.Stamp（今の時刻で引いた TF の stamp．最新の TF で代えた周期はその stamp，真値を使うときは無い）
    plants = {tmp_Estimator.Plant}';
    age = (cellfun(@nowOf, raw, plants) - stampAll) * 1e3;
    hasAge = ~isnan(age);
    if any(hasAge)
        C = palette();
        Ta = T(hasAge);
        isNew = isNewCloud(hasAge);
        age = age(hasAge);
        newFigure();
        plot(Ta, age, '-', 'Color', C(1,:), 'LineWidth', 1.6);
        plot(Ta(isNew), age(isNew), 'o', 'Color', C(2,:), 'MarkerFaceColor', C(2,:), 'MarkerSize', 3);
        xlabel('Time [s]')
        ylabel('Current time - point cloud stamp [ms]')
        timeLim(T)
        legend(sprintf('Every cycle (mean %.0f ms)', mean(age)), ...
            sprintf('New point cloud received (mean %.0f ms)', mean(age(isNew))), 'Location', 'best')
        saveFigure('LiDAR_age', Outputdir)
    end
end

function t = cloudStamp(r)
    % 点群の stamp [s]．点群が無ければ NaN
    t = NaN;
    if isfield(r,'LIDAR') && ~isempty(r.LIDAR)
        t = double(r.LIDAR.header.stamp.sec) + double(r.LIDAR.header.stamp.nanosec)*1e-9;
    end
end

function n = ndtOf(r)
    % NDT の情報．2026-11-19 から LIDAR_pose.NDT_info，それより前は NDT_info．無ければ []
    n = [];
    if isfield(r,'LIDAR_pose') && isstruct(r.LIDAR_pose) && isfield(r.LIDAR_pose,'NDT_info')
        n = r.LIDAR_pose.NDT_info;
    elseif isfield(r,'NDT_info')
        n = r.NDT_info;
    end
end

function t = nowOf(r, P)
    % 今の ROS の時刻 [s]．2026-11-19 から Plant.Stamp，それより前の記録は受け取った直後の data.Now．無ければ NaN
    t = NaN;
    if isfield(r,'Now') && ~isempty(r.Now)
        t = r.Now;
    elseif isfield(P,'Stamp') && ~isempty(P.Stamp)
        t = P.Stamp;
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

function timeLim(T)
    % 横軸を最初から最後の時刻にする．点が 1 つしかない（新しい組が 1 つだけなど）と範囲が作れないので，そのときは自動のまま
    if numel(T) >= 2 && T(end) > T(1)
        xlim([T(1), T(end)])
    end
end

function plotWithAverage(T, data, name, yName)
    C = palette();
    newFigure();
    plot(T, data, '-', 'Color', C(1,:), 'LineWidth', 2.0);
    [~, av] = plot_average(T, data);
    xlabel('Time [s]')
    ylabel(yName)
    timeLim(T)
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
    timeLim(T)
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
    timeLim(T)
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
