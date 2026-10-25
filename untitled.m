tmp_cat=cat(1,tmp{:,1});
tmp_Estimator=cat(1,tmp_cat.Estimator);
tmp_RawData=cat(1,tmp_Estimator.RawData);
tmp_NDT_info=cat(1,tmp_RawData.NDT_info);

ndt.downsampled_points = cat(1,tmp_NDT_info.downsampled_points);
ndt.matching_time = cat(1,tmp_NDT_info.matching_time);
ndt.convergence = cat(1,tmp_NDT_info.convergence);
ndt.score = cat(1,tmp_NDT_info.score);
ndt.angle_diff = cat(1,tmp_NDT_info.angle_diff);

state = [userLocal.X,userLocal.Y,userLocal.Z,...
        userLocal.Roll,userLocal.Pitch,userLocal.Yaw];

T = seconds(userLocal.Time)-seconds(userLocal.Time(1));
dt = T-[0;T(1:end-1)];

figure
box on;hold on;
plot(T,ndt.matching_time)
plot_average(T,ndt.matching_time);
xlabel('Time[s]')
ylabel('Matching Time[s]')
xlim(T([1,end],:)')

figure
box on;hold on;
plot(T,ndt.angle_diff)
[~,av] =plot_average(T,ndt.angle_diff);
xlabel('Time [s]')
ylabel('Angle diff')
xlim(T([1,end],:)')

figure
box on; hold on;
yyaxis left
plot(T, state(:,1));
plot(T, state(:,2));
ylabel('Position (X, Y) [m]')
yyaxis right
plot(T, state(:,3));
ylabel('Position (Z) [m]')
xlabel('Time [s]')
xlim(T([1,end],:)')
legend('X','Y','Z')

figure
box on; hold on;
yyaxis left
plot(T, state(:,4));
plot(T, state(:,5));
ylabel('Angle (Roll, Pitch) [rad]')
yyaxis right
plot(T, state(:,6));
ylabel('Angle (Yaw) [rad]')
xlabel('Time [s]')
xlim(T([1,end],:)')
legend('Roll','Pitch','Yaw')



function [p,average] = plot_average(T,data)
    average = sum(data)/length(data);
    p = plot(T([1,end],:),[average,average]);
end



