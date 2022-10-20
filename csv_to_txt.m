% This script takes in 2 csv file: rospose.csv and rostrig.csv, to produce
% an output script in .txt format. The .txt file is similar to the file
% generated in unity

rospose_filename = 'rospose.csv';
rostrig_filename = 'rostrig.csv';
output_filename = 'RawData_T1_400/session_1_01.txt';

% First, read the rospose.csv file to extract the rostime, pos_x, pos_y,
% orientation_z, orientation_w data. Then, calculate the euler angle from
% the quaternion
rospose = readtable(rospose_filename);
% use table2array to remove header
rostime = table2array(rospose(:,1));
pos_x = table2array(rospose(:,5));
pos_y = table2array(rospose(:,6));
orientation_z = table2array(rospose(:,10));
orientation_w = table2array(rospose(:,11));
q = [pos_x pos_y orientation_z orientation_w];
[pitch, roll, angle] = quat2angle(q, 'YXZ');
angle = (angle / pi * 180) + 180; % convert rad (-pi to pi) to angles (0 to 360)

% next, we need to convert the ros x and y coorinates to unity coordinates
x_offset = 2.305; % this value is manually calculated
y_offset = 0.76; % this value is manually calculated
pos_x = pos_x + x_offset;
posy = pos_y + y_offset;

% next, read the rostrig.csv file. We then try to sync the rostrig data
% with rospose data
rostrig = readtable(rostrig_filename);
trig_length = height(rostrig); %

% we try to fill up the rostrig data one by one into the rostrig data
current_index = 1;
max_index = height(rostime);
rostrig_data = zeros(max_index, 1);
count = 0;

for i=1:trig_length
    % fprintf("%d \n", i); %debug
    curr_rostime = table2array(rostrig(i,1)); % read the current rostrig rostime
    curr_num = table2array(rostrig(i,2)); % read the rostrig data

    for current_index = current_index:max_index
        if rostime(current_index) >= curr_rostime
            rostrig_data(current_index) = curr_num;
            current_index = current_index + 1;
            count = count + 1;
            % fprintf("count = %d \n", count); % debug
            break
        end
    end
end

% fill in the last rostrig_data if it didn't get filled in
if count < trig_length
    rostrig_data(current_index) = curr_num;
end

% the current rostime is in epoch unix rostime, need to convert to seconds
base_time = rostime(1);
time = rostime - base_time;
time = time / 1e9; %divide by 1e9 to get difference in seconds

% check which index the first data is filled, so that we can delete the top
% unfilled rows later
for j =1:max_index
    if rostrig_data(j) ~= 0
        break
    end
end

dt = time;
for k = j:length(time)-1
    temp = time(k);
    time_diff = time(k+1) - temp;
    dt(k+1) = time_diff;
end

dt(j)=0;

% calibrate time to start from 0 at start of first trial
% new_base_time = time(j);
% time = time - new_base_time;

result = table(rostrig_data, dt, pos_x, pos_y, angle);

rows_to_delete = j-1; % this is the number of rows to be deleted

% extra stuff needed to modify rplparallel
new_base_time = time(j);
time = time - new_base_time;
temp_timestamps = table(time);
if rows_to_delete ~= 0
    temp_timestamps([1:rows_to_delete],:) = [];
end
writetable(temp_timestamps, 'temp_timestamps.csv');

% if the number of rows is not zero, deleted j number of rows
if rows_to_delete ~= 0
    result([1:rows_to_delete],:) = [];
end

% convert from table to array
data = table2array(result);
data = sprintf(' %d %.8f %.4f %.4f %.4f \n',data');

% x=csvread('box.csv',1,0); %offset first row
% data=sprintf('%f %f %f %f %f %f %f %f %f \n',x'); % put %f number of rostimes same as number of columns,
%  Insted of %f you can set the decimals you wants like eg. %5.6f
% need to transpose the array x as the row and column data are inversed
content = sprintf("Version: 20171221 Taxi Continuous v4\n"...
+ "Trigger: v4\n"...
+ "TaskType: Continuous\n"...
+ "PosterLocations: P1(-5,1.5,-7.55) P2(-7.55,1.5,5) P3(7.55,1.5,-5) P4(5,1.5,7.55) P5(5,1.5,2.45) P6(-5,1.5,-2.45)\n"...
+ "TrialType: Double Tee\n"...
+ "SpecifiedRewardNo: 400\n"...
+ "CompletionWindow: 25000\n"...
+ "TimeoutDuration: 4000\n"...
+ "IntersessionInterval: 500\n"...
+ "Rewardrostime: 1000\n"...
+ "RotationSpeed: 60.4234\n"...
+ "TranslationSpeed: 5.218029\n"...
+ "JoystickDeadzone: 0.3033634\n"...
+ "RewardViewCriteria: 1 \n");

content = append(content, data);
fId= output_filename;
fId = fopen(fId, 'w') ;
fwrite( fId, content);
fclose( fId ) ;