classdef SensorFetcher < handle
    % 各センサデータのメッセージをsubscriberから受け取り，使いやすい形に整理するクラス

    properties
        mode
        sensorIdx
        baseSensor
        nostd = 0;
        lidarStampThreshold = 0.1; % [s] 今の時刻と点群の stamp の差がこれ以上なら，点群の stamp の TF を引かず最新の TF を使う

    end
    properties (Access=private)
        prevAngle
        timeout = 0.05;
        lidarPair       % 点群と，その stamp の時刻の LiDAR の姿勢の最新の組（struct: cloud, pose）
        lastLidarStamp  % 前の周期に渡した点群の stamp [sec, nanosec]
        tftree          % ros2tf（/tf・/tf_static を溜める．ROS2CommManager が setTfTree で渡す）
        frames          % TF のフレーム名（struct: map, base, lidar）
        node            % 今の時刻（ros2time）を取る ROS のノード

    end
    properties (Constant)
        t_cr = 0.495; % Tread width of CR
    end

    methods
        function obj = SensorFetcher(cfg)
            obj.mode = cfg.modeNumber;
            obj.sensorIdx = cfg.vehicleInfo.sensor;
            if isfield(cfg,'lidarStampThreshold')
                obj.lidarStampThreshold = cfg.lidarStampThreshold;
            end
            obj.baseSensor = cfg.vehicleInfo.base_sensor;
            if obj.baseSensor == 0
                obj.nostd = 1;
            end
        end

        function setTfTree(obj, tftree, frames, node)
            % sensor(5) の後輪間中心と LiDAR の姿勢を引く TF の木，フレーム名，今の時刻を取るノード
            obj.tftree = tftree;
            obj.frames = frames;
            obj.node = node;
            % 指定の時刻の TF が引けず最新の TF を使ったときの警告（main.m の warning('off','all') の後でも出す）
            warning('on', 'CR_debug:LidarTfFallback');
            warning('on', 'CR_debug:PlantTfFallback');
        end

        function [data,Plant] = getSensorData(obj,sensorSubs,whillSubs)
            % subscriberからセンサ値を取得
            doProcessing = 0;
            % LIDAR_pose：LIDAR（点群）の stamp の時刻の LiDAR の姿勢（sensor(5)．/tf を引く），LIDAR_new：前の周期と違う点群か，
            % LIDAR_latest_tf：新しい点群の stamp の時刻の TF が引けないか，今の時刻と点群の stamp の差がしきい値以上で，最新の TF で組にしたか，
            % LIDAR_waiting：TF が 1 つも引けず，前の組のままにしたか，
            % tform_velodyne：取付 base → LiDAR（/tf_static．sensor(5)．rigidtform3d で，LiDAR の座標の点を base の座標に直す．
            % 毎周期引き，引けなければ []），
            % NDT_info：/localization_debug（sensor(7)），Now：受け取った直後の ROS の時刻（ROS2CommManager が入れる）。
            % どの周期も同じフィールドにする（記録を cat で並べられるように）
            data = struct("LIDAR",[],"GNSS",[],"CAMERA",[],"IMU",[],"LIDAR_pose",[],"LIDAR_new",false, ...
                "LIDAR_latest_tf",false,"LIDAR_waiting",false,"tform_velodyne",[],"NDT_info",[],"Now",[]);
            % Plant.Stamp：Plant に使った TF の stamp [s]（sensor(5)．引けなければ []）
            Plant = struct("X",[.0],"Y",[.0],"Z",[.0], ...
                "Roll",[.0],"Pitch",[.0],"Yaw",[.0],"odom",[0, 0, 0, 0],"info",[],"Stamp",[]);
            if obj.mode == 1
                return;
            end
            if ~obj.sensorIdx(1) && (obj.sensorIdx(4)||obj.sensorIdx(5)) && obj.mode >= 2
                error("Localizer need to LiDAR. Please confirm LiDAR's sensor-index is true.")
            end
            if obj.sensorIdx(4) && obj.mode == 3
                error("CR_debug does not subscribe to a SLAM pose on the real vehicle. Please use Matching (sensor 5).")
            end
            if obj.sensorIdx(5) && isempty(obj.tftree)
                error("Matching (sensor 5) reads /tf. Please call setTfTree.")
            end
            tStart = tic;
            ret = cell(size(sensorSubs));
            while toc(tStart) < obj.timeout %true 
                whillret = whillSubs{1}.LatestMessage;                
                if ~isempty(sensorSubs{1})
                    ret{1} = sensorSubs{1}.LatestMessage;
                end
                if ~isempty(sensorSubs{2})
                    ret{2} = sensorSubs{2}.LatestMessage;
                end
                if ~isempty(sensorSubs{3})
                    ret{3} = sensorSubs{3}.LatestMessage;
                end
                if ~isempty(sensorSubs{4})
                    ret{4} = sensorSubs{4}.LatestMessage;
                end
                if ~isempty(sensorSubs{5})
                    ret{5} = sensorSubs{5}.LatestMessage;
                end
                if ~isempty(sensorSubs{7})
                    ret{7} = sensorSubs{7}.LatestMessage;
                end
                if obj.nostd || ~isempty(ret{obj.baseSensor})
                    break;
                end
            end
            if ~isempty(whillret)
                switch obj.mode
                    case 2
                        Plant.odom(1) = whillret.twist.twist.linear.x; % Linear velocity V (m/s)
                        Plant.odom(2) = whillret.twist.twist.angular.z; % Angular velocity Omega (rad/s)
                        Plant.odom(3) = Plant.odom(1) - (obj.t_cr/2) * Plant.odom(2); % LeftMSpeed
                        Plant.odom(4) = Plant.odom(1) + (obj.t_cr/2) * Plant.odom(2); % RightMSpeed
                    case 3
                        Plant.odom(1) = (-whillret.x + whillret.y) * 1000/3600 / 2; % V (m/s)
                        Plant.odom(2) = -(whillret.x + whillret.y) * 1000/3600 / obj.t_cr; % Omega (rad/s)
                        Plant.odom(3) = whillret.x * 1000/3600; % LeftMSpeed
                        Plant.odom(4) = whillret.y * 1000/3600; % RightMSpeed
                    otherwise
                        error(strcat(strcat('Invalid sensor index in mode',obj.mode)))

                end
            end

            
            if obj.sensorIdx(1) && ~isempty(ret{1})
                if obj.sensorIdx(5)
                    % 点群の stamp の時刻の map → LiDAR を /tf（ekf_localizer の周期ごとの予測値を補間）と
                    % /tf_static（取付）から引いて組にする．点群は常に最新を使うので，その時刻の TF が引けないときと，
                    % 今の時刻と点群の stamp の差が lidarStampThreshold 以上のときは，前の組を使わず最新の TF で組にする（どちらも警告を出す）
                    if isempty(obj.lidarPair) || ~isequal(obj.stampOf(ret{1}), obj.stampOf(obj.lidarPair.cloud))
                        tf = [];
                        age = obj.nowSec() - obj.stampSec(ret{1});
                        if abs(age) < obj.lidarStampThreshold
                            tf = obj.lookup(obj.frames.map, obj.frames.lidar, ret{1}.header.stamp);
                            reason = "not in the TF buffer";
                        else
                            reason = sprintf("%.3f s from now, threshold %.3f s", age, obj.lidarStampThreshold);
                        end
                        if isempty(tf)
                            tf = obj.lookup(obj.frames.map, obj.frames.lidar);
                            data.LIDAR_latest_tf = ~isempty(tf);
                            if isempty(tf)
                                action = "no TF yet, keeping the previous pair";
                            else
                                action = sprintf("using the latest TF (stamp %.3f)", obj.stampSec(tf));
                            end
                            warning('CR_debug:LidarTfFallback', 'LiDAR: no %s -> %s at the point cloud stamp %.3f (%s); %s', ...
                                obj.frames.map, obj.frames.lidar, obj.stampSec(ret{1}), reason, action);
                        end
                        if isempty(tf)
                            data.LIDAR_waiting = true;
                        else
                            obj.lidarPair = struct("cloud",ret{1},"pose",obj.poseOf(tf));
                        end
                    end
                    if ~isempty(obj.lidarPair)
                        stamp = obj.stampOf(obj.lidarPair.cloud);
                        data.LIDAR = obj.lidarPair.cloud;
                        data.LIDAR_pose = obj.lidarPair.pose;
                        data.LIDAR_new = ~isequal(stamp, obj.lastLidarStamp);
                        obj.lastLidarStamp = stamp;
                    end
                else
                    data.LIDAR = ret{1};
                end
            end
            if obj.sensorIdx(2) && ~isempty(ret{2})
                data.GNSS = ret{2};
            end
            if obj.sensorIdx(3) && ~isempty(ret{3}) && numel(ret{3}.detections) ~= 0
                data.CAMERA = ret{3};
                doProcessing = 1;
            end
            if obj.sensorIdx(5)
                % 取付 base → LiDAR（/tf_static）．毎周期引く．まだ届いていなければ []
                data.tform_velodyne = obj.tformOf(obj.lookup(obj.frames.base, obj.frames.lidar));
            end
            if obj.sensorIdx(5) && ~obj.sensorIdx(4)
                % 後輪間中心：今の ROS の時刻（Gazebo・再生は /clock）の map → base（ekf_localizer が周期ごとに出す予測値を補間）．
                % 引けなければ最新の TF を使い，警告を出す
                nowStamp = ros2time(obj.node, "now");
                ret{5} = obj.lookup(obj.frames.map, obj.frames.base, nowStamp);
                if isempty(ret{5})
                    ret{5} = obj.lookup(obj.frames.map, obj.frames.base);
                    if isempty(ret{5})
                        action = "no TF yet";
                    else
                        action = sprintf("using the latest TF (stamp %.3f)", obj.stampSec(ret{5}));
                    end
                    warning('CR_debug:PlantTfFallback', 'Plant: no %s -> %s at now %.3f; %s', ...
                        obj.frames.map, obj.frames.base, double(nowStamp.sec) + double(nowStamp.nanosec)*1e-9, action);
                end
            end
            if (obj.sensorIdx(4)|| obj.sensorIdx(5)) && (~isempty(ret{4}) || ~isempty(ret{5}))
                if obj.mode==2 && obj.sensorIdx(4) % Gazebo真値 (/wheelchair/pose, geometry_msgs/Pose)
                    Plant.X = ret{4}.position.x;
                    Plant.Y = ret{4}.position.y;
                    Plant.Z = ret{4}.position.z;
                    ang(1) = ret{4}.orientation.w;
                    ang(2) = ret{4}.orientation.x;
                    ang(3) = ret{4}.orientation.y;
                    ang(4) = ret{4}.orientation.z;
                elseif obj.sensorIdx(5) % Matching（今の時刻の /tf map → base．引けなければ最新）
                    Plant.Stamp = obj.stampSec(ret{5});
                    Plant.X = ret{5}.transform.translation.x;
                    Plant.Y = ret{5}.transform.translation.y;
                    Plant.Z = ret{5}.transform.translation.z;
                    ang(1) = ret{5}.transform.rotation.w;
                    ang(2) = ret{5}.transform.rotation.x;
                    ang(3) = ret{5}.transform.rotation.y;
                    ang(4) = ret{5}.transform.rotation.z;
                    if obj.sensorIdx(7) && ~isempty(ret{7})
                        % Plant.info.downsampled_points = ret{7}.data(1);
                        % Plant.info.matching_time = ret{7}.data(2);
                        % Plant.info.convergence = ret{7}.data(3);
                        % Plant.info.score = ret{7}.data(4);
                        % Plant.info.angle_diff = ret{7}.data(5);
                        data.NDT_info.downsampled_points = ret{7}.data(1);
                        data.NDT_info.matching_time = ret{7}.data(2);
                        data.NDT_info.convergence = ret{7}.data(3);
                        data.NDT_info.score = ret{7}.data(4);
                        data.NDT_info.angle_diff = ret{7}.data(5);
                    end
                end
                eul = quat2eul(ang,"ZYX");
                Plant.Yaw = eul(1);
                Plant.Pitch = eul(2);
                Plant.Roll = eul(3);
                tmpAngle.Yaw = Plant.Yaw;
                tmpAngle.Pitch = Plant.Pitch;
                tmpAngle.Roll = Plant.Roll;

                if ~isempty(obj.prevAngle)
                    Plant = AngleAdjstment(Plant,obj.prevAngle);
                end
                obj.prevAngle.Yaw = tmpAngle.Yaw;
                obj.prevAngle.Pitch = tmpAngle.Pitch;
                obj.prevAngle.Roll = tmpAngle.Roll;
            end
            if doProcessing
                data.CAMERA.processed_masks = camera_postProcess(data.CAMERA);
            end

            function [masks] = camera_postProcess(msg)
                %% YoloDetectionArray
                % std_msgs/Header header
                % YoloDetection[] detections
                % # Overlay of all detections' mask
                % std_msgs/UInt8MultiArray masks
                % uint16 rows
                % uint16 cols
                %% YoloDetection
                % # Store detections infomation of each object
                % uint8 label # COCO Labels ID
                % uint16[4] bboxparam # [xmin,ymin,xmax,ymax]
                % uint32[180] hue # Hue histogram
                % float32 score
                % # BoolArray mask Obsolete (too heavy processing)
                detNum = numel(msg.detections);
                if detNum == 0
                    return
                end                
                masks = encode_png(msg);
            
            end
            
            function mask = encode_png(msg)
                % msg : yolo_msgs/YoloDetectionArray から受け取ったメッセージ
                % msg.masks.data に PNG 圧縮バイト列が uint8 のベクトルで格納されている前提

                % 1) MATLAB の uint8 を Java の byte[] (signed) に変換
                pngData = uint8(msg.masks.data)';                  % 列ベクトル化
                javaBytes = typecast(pngData, 'int8');             % Java byte[] は符号付き
            
                % 2) ByteArrayInputStream を生成
                import java.io.ByteArrayInputStream
                bais = ByteArrayInputStream(javaBytes);
            
                % 3) Java の ImageIO で PNG をデコード
                import javax.imageio.ImageIO
                bufImg = ImageIO.read(bais);
                if isempty(bufImg)
                    error('Failed to process ImageIO.read (Possibly byte array is NOT PNG format)');
                end
            
                % 4) グレースケール（mask）は DataBufferByte で取得
                raster = bufImg.getRaster;                         % WritableRaster
                db     = raster.getDataBuffer;                     % DataBuffer
                byteArr= db.getData();                             % int8 の配列
            
                % 5) MATLAB の uint8 配列に戻す
                byteArr = typecast(int8(byteArr), 'uint8');        % Java int8 -> MATLAB uint8
            
                % 6) 画像サイズを取得して reshape
                w = bufImg.getWidth;
                h = bufImg.getHeight;
                mask = reshape(byteArr, [w, h])';                  % 転置して (h×w) に
            
            end
            function ret = AngleAdjstment(ret, Data)
                while ret.Roll - Data.Roll > pi
                   ret.Roll = ret.Roll - 2 * pi;
                end
            
                while ret.Roll - Data.Roll <= -pi
                   ret.Roll = ret.Roll + 2 * pi;
                end
            
                while ret.Pitch - Data.Pitch > pi
                   ret.Pitch = ret.Pitch - 2 * pi;
                end
            
                while ret.Pitch - Data.Pitch <= -pi
                   ret.Pitch = ret.Pitch + 2 * pi;
                end
            
                while ret.Yaw - Data.Yaw > pi
                   ret.Yaw = ret.Yaw - 2 * pi;
                end
            
                while ret.Yaw - Data.Yaw <= -pi
                   ret.Yaw = ret.Yaw + 2 * pi;
                end
            end

        end
    end
    methods (Access=private)
        function t = nowSec(obj)
            % 今の ROS の時刻 [s]（Gazebo・録画の再生は /clock）
            s = ros2time(obj.node, "now");
            t = double(s.sec) + double(s.nanosec)*1e-9;
        end

        function tf = lookup(obj, parent, child, stamp)
            % parent → child の TF（TransformStamped）．stamp を省くと最新．まだ引けなければ []（待たない）
            tf = [];
            try
                if nargin < 4
                    if canTransform(obj.tftree, parent, child)
                        tf = getTransform(obj.tftree, parent, child);
                    end
                elseif canTransform(obj.tftree, parent, child, stamp)
                    tf = getTransform(obj.tftree, parent, child, stamp);
                end
            catch
                tf = [];
            end
        end
    end
    methods (Static, Access=private)
        function s = stampOf(msg)
            % header.stamp を [sec, nanosec] にする（点群と姿勢の組を比べる）
            s = [double(msg.header.stamp.sec), double(msg.header.stamp.nanosec)];
        end

        function t = stampSec(msg)
            % header.stamp を [s] にする
            t = double(msg.header.stamp.sec) + double(msg.header.stamp.nanosec)*1e-9;
        end

        function pose = poseOf(tf)
            % TransformStamped（map → フレーム）を X, Y, Z, Roll, Pitch, Yaw，stamp [s] にする
            t = tf.transform.translation;
            q = tf.transform.rotation;
            eul = quat2eul([q.w, q.x, q.y, q.z],"ZYX");
            pose = struct("X",t.x,"Y",t.y,"Z",t.z,"Roll",eul(3),"Pitch",eul(2),"Yaw",eul(1), ...
                "Stamp",double(tf.header.stamp.sec) + double(tf.header.stamp.nanosec)*1e-9);
        end

        function T = tformOf(tf)
            % TransformStamped（parent → child）を rigidtform3d にする（child の座標の点を parent の座標に直す）．tf が [] なら []
            T = [];
            if isempty(tf)
                return;
            end
            t = tf.transform.translation;
            q = tf.transform.rotation;
            T = rigidtform3d(quat2rotm([q.w, q.x, q.y, q.z]), [t.x, t.y, t.z]);
        end
    end
end
