classdef ROS2CommManager < bridge.ROS2CommStrategy
    properties(Constant)
        % -------Wheelchair info --------
        % x:Left wheel, y:Right wheel
        t_cr = 0.495 % Tread width of CR
        WhillColOpt = {'red','white','blue'}
        
    end
    properties
        vehicleColor
        useSimTime  % ROS の時刻を /clock にするか（Gazebo と録画の再生）
        tftree      % ros2tf（/tf・/tf_static。後輪間中心と点群の時刻の LiDAR の姿勢を引く）

    end

    methods
        function obj = ROS2CommManager(cfg,mode)
            % Call obj.mode, obj.sensorIdx, obj.node
            obj@bridge.ROS2CommStrategy(cfg.vehicleInfo, mode, cfg.rosNamespace, cfg.RID);
            obj.vehicleColor = cfg.vehicleInfo.color;
            % ROS の時刻を /clock にする
            obj.useSimTime = mode == 2 || (isfield(cfg,'useSimTime') && cfg.useSimTime);
            if obj.useSimTime
                setParameter(obj.node, "use_sim_time", true);
            end
            
            % センサデータの収集クラス
            obj.sensorFetcher = bridge.CR_debug.SensorFetcher(cfg);
            % sensor(5)では後輪間中心とLiDARの姿勢を取得
            % 後輪間中心 : /tf (map→base_link) ,Gazebo(mode 2)の場合 /tf (map→base_footprint)
            % LiDAR中心  : /tf (map→velodyne)
            if obj.sensorIdx(5)
                obj.tftree = ros2tf(obj.node);
                frames = struct("map","map","base","base_link","lidar","velodyne");
                if mode == 2
                    frames.base = "base_footprint";
                end
                obj.sensorFetcher.setTfTree(obj.tftree, frames, obj.node);
            end
        end
        
        %% 必須メソッドの定義
        function topics = getSensorTopics(obj)
            % Subscribeするセンサのトピックを定義
            %   1 LiDAR       /velodyne_points     点群（そのまま data.LIDAR）
            %   4 SLAM        /wheelchair/pose     Gazebo だけ．真値を Plant にする
            %   5 Matching    /localization_debug  NDT の情報（data.LIDAR_pose.NDT_info）．姿勢は /tf から引く：
            %                                      Plant：今の ROS の時刻の map → base（後輪間中心．引けなければ最新），Plant.Stamp：使った TF の stamp，
            %                                      data.LIDAR_pose：点群の stamp の時刻の map → velodyne（今の時刻との差がしきい値以上なら今の時刻），
            %                                      data.LIDAR_pose.tform：取付 base → velodyne（/tf_static．rigidtform3d）
            switch obj.mode
                case 2 % Gazebo (4番目のみ真値 /wheelchair/pose，他は実機と共通)
                    topics = {'/velodyne_points','/ublox_gps_node/fix','/yolo/detections','/wheelchair/pose','/localization_debug'};
                case 3 % Real Exp.
                    topics = {'/velodyne_points','/ublox_gps_node/fix','/yolo/detections',[],'/localization_debug'};
            end
        end

        function types = getSensorMsgTypes(obj)
            % Subscribeするセンサのメッセージ型を定義
            switch obj.mode
                case 2
                    types = {'sensor_msgs/PointCloud2','sensor_msgs/NavSatFix','yolo_msgs/YoloDetectionArray','geometry_msgs/Pose','std_msgs/Float32MultiArray'};
                case 3
                    types = {'sensor_msgs/PointCloud2','sensor_msgs/NavSatFix','yolo_msgs/YoloDetectionArray',[],'std_msgs/Float32MultiArray'};
            end
        end

        function topics = getVehicleSubsTopics(obj)
            % Subscribeするビークルのトピックを定義
            switch obj.mode
                case 2
                    topics = {'/wheelchair/odom'};
                case 3
                    topics = {'/empty/whill_node/motor_speed'};
                    topics = setWhillColor(obj,topics);
            end
        end

        function types = getVehicleSubsMsgTypes(obj)
            % Subscribeするビークルのメッセージ型を定義
            switch obj.mode
                case 2
                    types = {'nav_msgs/Odometry'};
                case 3
                    types = {'geometry_msgs/Vector3'};
            end
        end

        function topics = getVehiclePubsTopics(obj)
            % Publishするビークルのトピックを定義
            switch obj.mode
                case 2
                    topics = {'/wheelchair/diff_drive_controller/cmd_vel'};
                case 3                    
                    topics = {'/empty/whill_node/cmd_vel'};
                    topics = setWhillColor(obj,topics);
            end
        end

        function types = getVehiclePubsMsgTypes(obj)
            % Publishするビークルのメッセージ型を定義
            switch obj.mode
                case 2
                    types = {'geometry_msgs/TwistStamped'};
                case 3
                    types = {'geometry_msgs/Twist'};
            end
        end

        function qos = getQoSProfile(obj, topicType)
            % トピックのQoSを設定
            if nargin < 2
                topicType = [];
            end
            switch obj.mode
                % 特に指定が無ければデフォルト
                % 指定する場合'sensorSubs','vehicleSubs','vehiclePubs'毎に指定
                case 2
                    qos = obj.BESTEFFORT_QOS;                    
                    if strcmp(topicType,'vehiclePubs')
                        qos.Reliability = "reliable"; % Fixing for sfm gazebo
                        qos.Depth = 10;
                    end
                case 3
                    qos = obj.DEFAULT_QOS;
            end
        end

        function msg = createVehicleCommand(obj, cmd)
            V = cmd.V;
            msg = obj.vehicleMsg;
            % seq = cmd.sequence;
            if obj.mode == 2
                msg.twist.linear.x = double(V(1));
                msg.twist.angular.z = double(V(2));
            elseif obj.mode == 3
                % システムの都合で角速度はobj.t_crを乗算
                msg.linear.x = double(V(1));
                msg.angular.z = double(V(2))*obj.t_cr; % rad/s -> m/s
            end
        end

        function cmd = createSafetyStopCommand(obj)
            % 実機実験用: 緊急停止コマンドを作成
            cmd = struct();
            cmd.V = zeros(size(obj.prevCmd.V));
            cmd.sequence = -1;
        end
    end
    methods (Access=private)
        function topic = setWhillColor(obj,topic)
            % CR用
            % 指定カラーに基づいてトピック名を定義
            color = char(lower(obj.vehicleColor));
            if ~ismember(color,obj.WhillColOpt)
                error('InvalidColor:NotRecognized', ...
                    'The specified color "%s" is invalid. Valid options are: %s.', ...
                    color, strjoin(colors, ', '));
            end
            prefix = ['/Drp5_whill_' color '/'];
            topic{1} = regexprep(topic{1}, '^(.*/)(whill_node.*)$', [prefix '$2']);
        end
    end    
end

