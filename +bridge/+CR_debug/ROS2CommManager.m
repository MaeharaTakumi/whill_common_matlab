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

    end

    methods
        function obj = ROS2CommManager(cfg,mode)
            % Call obj.mode, obj.sensorIdx, obj.node
            obj@bridge.ROS2CommStrategy(cfg.vehicleInfo, mode, cfg.rosNamespace, cfg.RID);
            obj.vehicleColor = cfg.vehicleInfo.color;
            % sensor(8)（LiDAR pose）を書いていない main でも動くように，足りない欄は false にする
            obj.sensorIdx(end+1:8) = false;
            % Gazebo（mode 2）と録画の再生（cfg.useSimTime）は，ROS の時刻を /clock にする（data.Now に使う）
            obj.useSimTime = mode == 2 || (isfield(cfg,'useSimTime') && cfg.useSimTime);
            if obj.useSimTime
                setParameter(obj.node, "use_sim_time", true);
            end
            
            % センサデータの収集クラス
            obj.sensorFetcher = bridge.CR_debug.SensorFetcher(cfg);
        end
        
        function [sensorData, Plant] = getSensorData(obj)
            % 受け取ったデータに，受け取った直後の ROS の時刻 data.Now [s] を付ける
            % （Gazebo・録画の再生は /clock，実機はこの PC の時刻．点群の stamp との差が点群の古さ）
            [sensorData, Plant] = getSensorData@bridge.ROS2CommStrategy(obj);
            t = ros2time(obj.node, "now");
            sensorData.Now = double(t.sec) + double(t.nanosec)*1e-9;
        end

        %% 必須メソッドの定義
        function topics = getSensorTopics(obj)
            % Subscribeするセンサのトピックを定義
            % 自己位置推定（ekf_localizer）の出力：
            %   1 LiDAR       /localization/vlp16_points  8 の姿勢と同じ stamp で出し直した点群
            %   5 Matching    /localization/base_pose     後輪間中心の姿勢（現在時刻まで予測した値，50 Hz）
            %   8 LiDAR pose  /localization/vlp16_pose    点群の時刻の LiDAR の姿勢（NDT で更新した直後の値）
            switch obj.mode
                case 2 % Gazebo (4番目のみ真値 /wheelchair/pose，他は実機と共通)
                    topics = {'/localization/vlp16_points','/ublox_gps_node/fix','/yolo/detections','/wheelchair/pose','/localization/base_pose',[],'/localization_debug','/localization/vlp16_pose'};
                case 3 % Real Exp.
                    topics = {'/localization/vlp16_points','/ublox_gps_node/fix','/yolo/detections','/ekf_pose','/localization/base_pose',[],'/localization_debug','/localization/vlp16_pose'};
            end
        end

        function types = getSensorMsgTypes(obj)
            % Subscribeするセンサのメッセージ型を定義
            switch obj.mode
                case 2
                    types = {'sensor_msgs/PointCloud2','sensor_msgs/NavSatFix','yolo_msgs/YoloDetectionArray','geometry_msgs/Pose','geometry_msgs/PoseWithCovarianceStamped',[],'std_msgs/Float32MultiArray','geometry_msgs/PoseWithCovarianceStamped'};
                case 3
                    types = {'sensor_msgs/PointCloud2','sensor_msgs/NavSatFix','yolo_msgs/YoloDetectionArray','geometry_msgs/PoseStamped','geometry_msgs/PoseWithCovarianceStamped',[],'std_msgs/Float32MultiArray','geometry_msgs/PoseWithCovarianceStamped'};
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

