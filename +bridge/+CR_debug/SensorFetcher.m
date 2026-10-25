classdef SensorFetcher < handle
    % 各センサデータのメッセージをsubscriberから受け取り，使いやすい形に整理するクラス

    properties
        mode
        sensorIdx
        baseSensor
        nostd = 0;

    end
    properties (Access=private)
        prevAngle
        timeout = 0.05;
        lidarPair       % stamp が同じ点群と LiDAR の姿勢の最新の組（struct: cloud, pose）
        lastLidarStamp  % 前の周期に渡した点群の stamp [sec, nanosec]

    end
    properties (Constant)
        t_cr = 0.495; % Tread width of CR
    end

    methods
        function obj = SensorFetcher(cfg)
            obj.mode = cfg.modeNumber;
            obj.sensorIdx = cfg.vehicleInfo.sensor;
            obj.sensorIdx(end+1:8) = false; % sensor(8)（LiDAR pose）を書いていない main でも動くように
            obj.baseSensor = cfg.vehicleInfo.base_sensor;
            if obj.baseSensor == 0
                obj.nostd = 1;
            end
        end

        function [data,Plant] = getSensorData(obj,sensorSubs,whillSubs)
            % subscriberからセンサ値を取得
            doProcessing = 0;
            % LIDAR_pose：LIDAR（点群）の時刻の LiDAR の姿勢（sensor(8)），LIDAR_new：前の周期と違う点群か，
            % NDT_info：/localization_debug（sensor(7)）。どの周期も同じフィールドにする（記録を cat で並べられるように）
            data = struct("LIDAR",[],"GNSS",[],"CAMERA",[],"IMU",[],"LIDAR_pose",[],"LIDAR_new",false,"NDT_info",[]);
            Plant = struct("X",[.0],"Y",[.0],"Z",[.0], ...
                "Roll",[.0],"Pitch",[.0],"Yaw",[.0],"odom",[0, 0, 0, 0],"info",[]);
            if obj.mode == 1
                return;
            end
            if ~obj.sensorIdx(1) && (obj.sensorIdx(4)||obj.sensorIdx(5)) && obj.mode >= 2
                error("Localizer need to LiDAR. Please confirm LiDAR's sensor-index is true.")
            end
            if obj.sensorIdx(8) && ~obj.sensorIdx(1)
                error("LiDAR pose (sensor 8) is paired with the LiDAR point cloud. Please confirm LiDAR's sensor-index is true.")
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
                if numel(sensorSubs) >= 8 && ~isempty(sensorSubs{8})
                    ret{8} = sensorSubs{8}.LatestMessage;
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
                if obj.sensorIdx(8)
                    % 点群とその時刻の LiDAR の姿勢を stamp で組にする．2 つは続けて配信されるが届く順は
                    % 入れ替わることがあるので，stamp が同じときだけ組を新しくし，違うときは前の組を使う
                    if ~isempty(ret{8}) && isequal(obj.stampOf(ret{1}), obj.stampOf(ret{8}))
                        obj.lidarPair = struct("cloud",ret{1},"pose",ret{8});
                    end
                    if ~isempty(obj.lidarPair)
                        stamp = obj.stampOf(obj.lidarPair.cloud);
                        data.LIDAR = obj.lidarPair.cloud;
                        data.LIDAR_pose = obj.poseOf(obj.lidarPair.pose);
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
            if (obj.sensorIdx(4)|| obj.sensorIdx(5)) && (~isempty(ret{4}) || ~isempty(ret{5}))
                if obj.mode==2 && obj.sensorIdx(4) % Gazebo真値 (/wheelchair/pose, geometry_msgs/Pose)
                    Plant.X = ret{4}.position.x;
                    Plant.Y = ret{4}.position.y;
                    Plant.Z = ret{4}.position.z;
                    ang(1) = ret{4}.orientation.w;
                    ang(2) = ret{4}.orientation.x;
                    ang(3) = ret{4}.orientation.y;
                    ang(4) = ret{4}.orientation.z;
                elseif obj.sensorIdx(4) % SLAM
                    Plant.X = ret{4}.pose.position.x;
                    Plant.Y = ret{4}.pose.position.y;
                    Plant.Z = ret{4}.pose.position.z;
                    ang(1) = ret{4}.pose.orientation.w;
                    ang(2) = ret{4}.pose.orientation.x;
                    ang(3) = ret{4}.pose.orientation.y;
                    ang(4) = ret{4}.pose.orientation.z;
                elseif obj.sensorIdx(5) % Matching（/localization/base_pose：後輪間中心を現在時刻まで予測した値）
                    Plant.X = ret{5}.pose.pose.position.x;
                    Plant.Y = ret{5}.pose.pose.position.y;
                    Plant.Z = ret{5}.pose.pose.position.z;
                    ang(1) = ret{5}.pose.pose.orientation.w;
                    ang(2) = ret{5}.pose.pose.orientation.x;
                    ang(3) = ret{5}.pose.pose.orientation.y;
                    ang(4) = ret{5}.pose.pose.orientation.z;
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
    methods (Static, Access=private)
        function s = stampOf(msg)
            % header.stamp を [sec, nanosec] にする（点群と姿勢の組を比べる）
            s = [double(msg.header.stamp.sec), double(msg.header.stamp.nanosec)];
        end

        function pose = poseOf(msg)
            % PoseWithCovarianceStamped を X, Y, Z, Roll, Pitch, Yaw，共分散（6x6），stamp [s] にする
            p = msg.pose.pose;
            eul = quat2eul([p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z],"ZYX");
            pose = struct("X",p.position.x,"Y",p.position.y,"Z",p.position.z, ...
                "Roll",eul(3),"Pitch",eul(2),"Yaw",eul(1), ...
                "Cov",reshape(double(msg.pose.covariance),6,6)', ...
                "Stamp",double(msg.header.stamp.sec) + double(msg.header.stamp.nanosec)*1e-9);
        end
    end
end
