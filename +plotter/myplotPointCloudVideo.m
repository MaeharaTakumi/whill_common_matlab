function myplotPointCloudVideo(DATAdir, MAPfile)
    % 点群地図と取得点群（地図座標系）を表示する動画を作る（result/video/pointcloud_map）
    %   DATAdir : userLocal.mat, userLocal_tmp.mat があるフォルダ
    %   MAPfile : 点群地図（.pcd）．空か無いファイルなら地図を描かない
    % 点群は，その stamp の時刻の LiDAR の姿勢（data.LIDAR_pose，/tf を点群の stamp で引いた値）で地図座標系に直す．
    % 車両中心（後輪間中心，黒い点），向きの矢印と軌跡は Plant（最新の /tf map → base）．LIDAR_pose の無い古い記録は，
    % Plant（/ekf_pose，LiDAR の姿勢）で点群を直す（点群と姿勢の時刻はそろっていない）．
    % 実験データの myplotPointCloudVideo.m（ndt/20261016）を元にした
    %% Settings
    if nargin < 1 || isempty(DATAdir)
        DATAdir = pwd;
    end
    if nargin < 2
        MAPfile = "";
    end

    zRange     = [-1.0 1.2];  % 表示する高さの範囲（その時刻の LiDAR の高さから）[m]（床・天井を除外）
    mapGrid    = 0.05;        % 地図のダウンサンプリング [m]
    viewHalf   = [14 8];      % 表示範囲 (自己位置から X±, Y±) [m]
    arrowLen   = 1.5;         % 姿勢矢印の長さ [m]
    FrameRate  = 1/0.05;      % 動画のフレームレート
    C = [42 120 214; 235 104 52; 27 175 122]/255;   % 軌跡・点群・LiDAR の色（dataviz の参照パレット）

    % 読み込み
    File  = strcat(DATAdir, '/userLocal_tmp.mat');
    load(File, 'tmp');

    File  = strcat(DATAdir, '/userLocal.mat');
    load(File, 'userLocal');

    %% make folda
    Outputdir = strcat(DATAdir,'/result');
    if ~exist(strcat(Outputdir,'/video'),'dir')
        mkdir(Outputdir,'video');
    end

    %% データ処理
    N = length(tmp);
    T = seconds(userLocal.Time - userLocal.Time(1));

    % 自己位置（Plant）と，点群・その時刻の LiDAR の姿勢（地図座標系）
    base = NaN(N,6);
    LiDAR = NaN(N,6);
    clouds = cell(N,1);
    hasLidarPose = false;
    for i = 1:N
        E = tmp{i, 1}.Estimator;
        P = E.Plant;
        base(i,:) = [P.X, P.Y, P.Z, P.Roll, P.Pitch, P.Yaw];
        R = E.RawData;
        if isfield(R, 'LIDAR_pose')
            hasLidarPose = true;
            if ~isempty(R.LIDAR_pose) && ~isempty(R.LIDAR)
                L = R.LIDAR_pose;
                LiDAR(i,:) = [L.X, L.Y, L.Z, L.Roll, L.Pitch, L.Yaw];
                clouds{i} = R.LIDAR;
            end
        elseif ~isempty(R.LIDAR)
            LiDAR(i,:) = base(i,:);   % 古い記録：Plant が LiDAR の姿勢
            clouds{i} = R.LIDAR;
        end
    end
    % Plant を受け取る前の周期（0）は描かない
    frames = find(any(base(:,1:3) ~= 0, 2)).';
    if isempty(frames)
        warning('myplotPointCloudVideo: no pose in %s', DATAdir);
        return
    end
    if hasLidarPose
        trajName = 'Trajectory (vehicle center)';
        poseName = 'Heading';
    else
        trajName = 'Trajectory';
        poseName = 'Pose';
    end

    % 点群地図（最初の LiDAR の高さから zRange の範囲）
    mapPt = zeros(0,3);
    if strlength(string(MAPfile)) > 0 && isfile(MAPfile)
        map = pcread(MAPfile);
        map = pcdownsample(map, 'gridAverage', mapGrid);
        mapPt = map.Location;
        z0 = LiDAR(find(~isnan(LiDAR(:,3)), 1), 3);
        if ~isempty(z0)
            mapPt = mapPt(mapPt(:,3) > z0 + zRange(1) & mapPt(:,3) < z0 + zRange(2), :);
        end
    elseif strlength(string(MAPfile)) > 0
        warning('myplotPointCloudVideo: map not found: %s', MAPfile);
    end

    %% Video
    if ispc
        writerObj = VideoWriter(strcat(Outputdir,'/video/pointcloud_map'),'MPEG-4');
    else
        writerObj = VideoWriter(strcat(Outputdir,'/video/pointcloud_map'));
        writerObj.Quality = 95;
    end
    writerObj.FrameRate = FrameRate;
    open(writerObj);

    % 非表示にしてウィンドウサイズが途中で変わらないようにする
    fig = figure('Color','w','Units','pixels','Position',[100 100 1200 760], ...
                 'Visible','off','Resize','off');

    %%--plotの設定--%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ax = axes(fig, 'Position', [0.08 0.11 0.9 0.86]);
    hold(ax,'on');
    p_map    = scatter(ax, mapPt(:,1), mapPt(:,2), 2, [0.6 0.6 0.6], 'filled');
    p_traj   = plot(ax, NaN, NaN, '-', 'Color', C(1,:), 'LineWidth', 2.0);
    p_scan   = scatter(ax, NaN, NaN, 4, C(2,:), 'filled');
    p_lidar  = plot(ax, NaN, NaN, 'o', 'Color', C(3,:), 'MarkerFaceColor', C(3,:), 'MarkerSize', 7);
    p_robot  = quiver(ax, NaN, NaN, NaN, NaN, 0, 'Color', 'k', 'LineWidth', 3, 'MaxHeadSize', 2);
    p_center = plot(ax, NaN, NaN, 'o', 'Color', 'k', 'MarkerFaceColor', 'k', 'MarkerSize', 9);
    time     = text(ax, 0.02, 0.96, '', 'Units', 'normalized', 'FontSize', 20, 'Interpreter', 'Latex', ...
                    'BackgroundColor', [1 1 1], 'EdgeColor', [0 0 0], 'VerticalAlignment', 'top');
    handles = [p_map, p_scan, p_traj, p_center, p_robot, p_lidar];
    names = {'Map', 'Scan', trajName, 'Vehicle center (rear axle)', poseName, 'LiDAR (scan stamp)'};
    show = [~isempty(mapPt), true, true, hasLidarPose, true, hasLidarPose];
    legend(ax, handles(show), names(show), 'Location', 'northeast', 'FontSize', 14);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    set(ax,'FontSize',20,'FontName','Times');
    xlabel(ax,'$X$ (m)','Interpreter', 'Latex','FontSize',20);
    ylabel(ax,'$Y$ (m)','Interpreter', 'Latex','FontSize',20);
    axis(ax,'equal');
    grid(ax,'on');
    box(ax,'on');
    ax.XColor = 'k';
    ax.YColor = 'k';
    ax.XLimMode = 'manual';
    ax.YLimMode = 'manual';

    % --ループ-------------------------------------------
    t0 = T(frames(1));
    num_frame = 0;
    frameSize = [];
    lastStamp = [];
    for count = frames
        pose = base(count,:);

        % 取得点群をその時刻の LiDAR の姿勢でグローバル座標系へ変換（同じ点群なら前のフレームのまま）
        if ~isempty(clouds{count})
            stamp = [double(clouds{count}.header.stamp.sec), double(clouds{count}.header.stamp.nanosec)];
            if ~isequal(stamp, lastStamp)
                lp    = LiDAR(count,:);
                xyz   = rosReadXYZ(clouds{count});
                xyz   = xyz(all(isfinite(xyz),2), :);
                Rxyz  = eul2rotm([lp(6), lp(5), lp(4)], 'ZYX');
                point = double(xyz)*Rxyz.' + lp(1:3);
                point = point(point(:,3) > lp(3) + zRange(1) & point(:,3) < lp(3) + zRange(2), :);
                set(p_scan, 'XData', point(:,1), 'YData', point(:,2));
                if hasLidarPose
                    set(p_lidar, 'XData', lp(1), 'YData', lp(2));
                end
                lastStamp = stamp;
            end
        end

        drawn = frames(frames <= count);
        set(p_traj, 'XData', base(drawn,1), 'YData', base(drawn,2));
        set(p_robot, 'XData', pose(1), 'YData', pose(2), ...
                     'UData', arrowLen*cos(pose(6)), 'VData', arrowLen*sin(pose(6)));
        if hasLidarPose   % 車両中心（後輪間中心．古い記録の Plant は LiDAR なので描かない）
            set(p_center, 'XData', pose(1), 'YData', pose(2));
        end

        ax.XLim = pose(1) + [-1 1]*viewHalf(1);
        ax.YLim = pose(2) + [-1 1]*viewHalf(2);
        set(time, 'String', ['$t = ', num2str(T(count) - t0,'%.2f'), '$ s']);
        drawnow;

        % フレーム調整 (実時間に合わせる) -------------
        frame = getframe(fig);
        % フレームサイズを最初のフレームに揃える (writeVideoは同一サイズが必須)
        if isempty(frameSize)
            frameSize = size(frame.cdata, [1 2]);
        elseif ~isequal(size(frame.cdata, [1 2]), frameSize)
            frame.cdata = imresize(frame.cdata, frameSize);
        end
        while num_frame*(1/writerObj.FrameRate) <= T(count) - t0
            writeVideo(writerObj, frame);
            num_frame = num_frame + 1;
        end
    end
    close(writerObj);
    close(fig);
    fprintf('Video saved: %s (%d frames)\n', fullfile(writerObj.Path, writerObj.Filename), num_frame);
end
