function new( bag, varargin)
% INTELVIEW is a basic viewer GUI for Intel RealSense stereo cameras
%
%   INTELVIEW( bag )
%   INTELVIEW( bag, msg_number )
%   INTELVIEW( bag, msg_number, volume_of_interest )
%
%   ARGUMENTS:
%   bag - a polymorphic argument that is either the bag file name (string)
%       or a ROS bag selection object. We handle either because we're cool.
%   msg_num [optional] - the message number where viewing will start. The
%       default value is message 1.
%   volume_of_interest - 6-array for the 3D display limits with syntax
%       [x_min x_max y_min y_max z_min z_max]
%
%   INTELVIEW is a basic gui for visualizing bag Intel RealSense bag files.
%   It provides a means to pause the bag for interacting with the data of a
%   specific message. It also provides allows the user to cycle through
%   which gradient direction to use in coloring the point cloud (i.e.,
%   x,y,or z). 
%
%   SAMPLE USAGE:
%   Example 1: Stream directly from a bag file
%   >> intelview('good.bag');
%   Example 2: Stream from ROS bag selection object at image 1000
%   >> bag = rosbag('good3.bag');
%   >> intelview(bag,1000);
%   Example 3: SAB, but set axis limits for the point cloud
%   >> intelview(bag,1000,[-5 5 -5 5 0 10]);
%
% Copyright (C) 2018, by JRS @ Lehigh University

%% Argument Checks
if nargin>=2                    % Optional message number check
    msg_num = varargin{1};
    
else
    msg_num = 1;
end
initial_num = msg_num;
if nargin==3                    % Optional axis display limits
    voi = varargin{2};
else
    voi = [];
end

%% Figure setup
% Set up the subplot for the point cloud...
close all;
f = figure('name',' Intel RealSense Viewer 0.11');
% h1 = subplot(1,2,1);
% set(gca,'color','k');
% set(gca,'gridcolor','w');
% colormap(gca,'jet');
% grid on;
% if nargin==3
%     axis('equal',voi);
% else
% %     axis('manual','equal');
%     axis('equal');
% end
% view(-60,40);
% xlabel('x (m)');
% ylabel('y (m)');
% zlabel('z (m)');
% hold on;
% And the RGB image...
h2 = subplot(2,2,[1 3]);
format long;
im = zeros(480,640,3,'uint8');
h_im3 = imagesc(im);
axis image;
hold on;
h_im = imagesc(im, 'AlphaData', 0.2);
axis image;
hold on;
% add side view
h3 = subplot(2,2,2);
axis([480-250 480 0 inf]);
% add plot of cov
h5 = subplot(2,2,4);
% add plot
%h4 = subplot(2,2,4);

%axis([480-horizon 480 0 inf]);
% To accommodate bag files and bag selection objects
if ~isa(bag,'robotics.ros.BagSelection' )
    bag = rosbag(bag);
end
% Get the topics
im_top = select(bag,'topic','/camera/color/image_rect_color');
pc_top = select(bag,'topic','/camera/depth_registered/points');
% Get the minimum number of messages for the two topics
num_msg = min(im_top.NumMessages,pc_top.NumMessages);

% Pause & stop buttons
addpushbutton(@pausecallback,f,'pause','on');
addpushbutton(@stopcallback,f,'stop');
addpushbutton(@gradcallback,f,'gradient');
addpushbutton(@colorcallback,f,'color');
% For callbacks
is_paused = false;          % For pause button
is_stopped = false;         % For stop button
% Coloring can either be gradient based upon a point direction
grad_ind = 3;               % For which gradient we are using
% Or we can use solid colorings
colors = ['r','g','b','y'];
color_ind = 1;
is_color = false;

v = VideoWriter('direction_depth_update','mpeg-4');
v.FrameRate = 15;
open(v);
is_MAX = true;

desire_distance = 4;
valid_distance = 10;
%horizon = 280;
danger_height = .8;
divide_num = 8;


% 2d array to contain frame number and the height of certain distane
f_to_h = [];

% 2d array to contain frame number and the depth of peak > danger_height
f_to_d = [];

% fft Data Collection
load('x.mat');
%X = [];

% three different tracker
index_count = 1;
max_height_tracker = [];%zeros(divide_num, num_msg, 5);
first_height_tracker = [];%zeros(divide_num, num_msg, 5);
back_height_tracker = [];

waveFront = [];
waveTop = [];
waveBack = [];

slope_direction = [];

initialHorizon = 0;
init = false;

wavePassed = 0;
prev_slope = 0;

%% Infinite display loop, loops through the bag file
while ~is_stopped
    % Enables looping
    if msg_num>num_msg
        break;
        msg_num = 1;
    end
    %disp(msg_num);
    % So we're not busy waiting...
    while is_paused
        pause(0.5);
        if is_stopped
            break;
        end
    end
    % Need this to clear first point set cleanly
    if exist('h_pts','var')
        delete(h_pts);
    end
    %% Read the message data
    % Image data
    im_msg = readMessages(im_top,msg_num);
    time = im_msg{1}.Header.Stamp.Sec;
    im = readImage(im_msg{1});
    % color extraction
    color_img = im;
    B = im(:,:,3);
    B = reshape(B, [640*480, 1]);
    [idx,D] = kmeans(double(B),2);
    idx = reshape(idx,[480,640]);
    classSky = median(median(idx(1:5,:)));
    points = zeros(640,2);
    for i = 1:640
        for j = 1:480
            if idx(j,i) ~= classSky
                points(i,:) = [i,j];
                break;
            end
        end
    end
    [bestParameter1,bestParameter2] = ransac_demo(transpose(points),2,100,1,0.1);
    rotateAngle = - (atand(0) - atand(bestParameter1)); %counter-clockwise rotation angle in degrees
    R = [cosd(rotateAngle) -sind(rotateAngle) 0;sind(rotateAngle) cosd(rotateAngle) 0;0 0 1];
    xAxis = 1:640;
    yAxis = bestParameter1*xAxis + bestParameter2;
    center=size(im)/2+.5;
    for i = 1:640
        xAxis(i) = xAxis(i) - center(2);
        yAxis(i) = yAxis(i) - center(1);
    end
    
    res = [xAxis(:),yAxis(:)];
    R2d = [cosd(rotateAngle) -sind(rotateAngle);sind(rotateAngle) cosd(rotateAngle)];
    res = res * R2d;
    xAxis = res(:,1);
    yAxis = res(:,2);
    io = im;
    im = imrotate(im,rotateAngle);
    center2=size(im)/2+.5;
    for i = 1:640
        xAxis(i) = xAxis(i) + center2(2);
        yAxis(i) = yAxis(i) + center2(1);
    end
    horizon = round(mean(yAxis));
    if ~init
        initialHorizon = horizon;
        init = true;
    end
    diff = (initialHorizon - center2(1))-(horizon - center2(1));
    horizon = initialHorizon;
    im = imtranslate(im,[0, diff]);
    
%     xAxis = 1:640; 
%     yAxis = bestParameter1*xAxis + bestParameter2;
% 
%     hold on;
%     imshow(im);
%     hold on;
%     plot(xAxis,yAxis,'r-','LineWidth',2);
     
    pc_msg = readMessages(pc_top,msg_num);
    pc_msg{1}.PreserveStructureOnRead = true;
    pts = readXYZ(pc_msg{1});
    [m, n, d] = size(pts);
    pts = reshape(pts, [m*n, d]);
    pts =  R * transpose(pts);
    pts = transpose(pts);
    pts = reshape(pts,[480,640,3]);
    copy_pts = pts; % copy a one for later analysis
    pts = reshape(pts, [m*n, d]);
    
    
    % Kill off NaNs and points at (0,0,0) in the point cloud
    %R = [1 0 0;0 cosd(90) -sind(90);0 sind(90) cosd(90)];
    %%Custom space end

    % Toss points outside of the volume of interest as well
    if ~isempty(voi)
        val = pts(:,1)>=voi(1) & pts(:,1)<=voi(2) & pts(:,2)>=voi(3) & ...
            pts(:,2)<=voi(4) & pts(:,3)>=voi(5) & pts(:,3)<=voi(6);
        pts = pts(val,:);
    end
    % color extraction
    reshaped_im = io;
    %color_img = reshaped_im;
    
    
    % height median vs depth
    cut_view = zeros(1,480-horizon);
    cut_view_index = zeros(1,480-horizon);
    chopped_pts = zeros(480-horizon,640,3);
    color_img = color_img(horizon+1:480,:,:);
    for i=1:(480-horizon)
        for j=1:640
            for k=1:3
                if k==2
                    chopped_pts(i,j,k) = 1.8 - copy_pts(i+horizon,j,k); 
                else
                    chopped_pts(i,j,k) = copy_pts(i+horizon,j,k); 
                end
            end
        end
    end
    chopped_pts(:,:,2) = imgaussfilt(chopped_pts(:,:,2),2);
    for i=1:(480-horizon)
        cut_view(i) = nanmedian(chopped_pts(i,:,2));
        cut_view_index(i) = i+horizon;
    end
    
    % color the horizon
    %disp(horizon);
    sizeim = size(im);
    ysize = sizeim(2);
    for i=horizon:horizon+1
        for j = 1:ysize
            im(i,j,1) = 0;
            im(i,j,2) = 0;
            im(i,j,3) = 255;
        end
    end
    
    % depth lookup table
    depth_view = zeros(1,480-horizon);
    depth_view_index = zeros(1,480-horizon);
    for i=1:(480-horizon)
        depth_view(i) = nanmedian(chopped_pts(i,:,3));
        depth_view_index(i) = i+horizon;
    end
    %interp1(depth_view_index, depth_view, 3,'nearest')
    depth_diff = abs(depth_view - desire_distance);
    minimal_depth_diff = min(depth_diff);
    [x, y] = find(depth_diff == minimal_depth_diff);
    if length(y) > 1
        y = y(1);
    end
    heightD = cut_view(y);
    f_to_h(end+1, 1:2) = [msg_num,heightD];
    X(end+1,:) = [heightD,time,0];
    % get the range inside of valide meter
    depth_diff = abs(depth_view - valid_distance);
    minimal_depth_diff = min(depth_diff);
    [x, y] = find(depth_diff == minimal_depth_diff);
    if length(y) > 1
        y = y(1);
    end
    valid_index = depth_view_index(y);
    
    %% get the range of 3 - 6 meters
    pointA = 3;
    pointB = 6;
    
    depth_diff = abs(depth_view - pointA);
    closest_to_a = min(depth_diff);
    [x, y] = find(depth_diff == closest_to_a);
    if length(y) > 1
        y = y(1);
    end
    index_at_a = depth_view_index(y);
    
    depth_diff = abs(depth_view - pointB);
    closest_to_b = min(depth_diff);
    [x, y] = find(depth_diff == closest_to_b);
    if length(y) > 1
        y = y(1);
    end
    index_at_b = depth_view_index(y);
    
    %% get 1m, 4m and 7m  depth index
    point1 = 2;
    point4 = 5;
    point7 = 8;
    
    depth_diff = abs(depth_view - point1);
    closest_to_1 = min(depth_diff);
    [x, y] = find(depth_diff == closest_to_1);
    if length(y) > 1
        y = y(1);
    end
    index_at_1 = depth_view_index(y);
    
    depth_diff = abs(depth_view - point4);
    closest_to_4 = min(depth_diff);
    [x, y] = find(depth_diff == closest_to_4);
    if length(y) > 1
        y = y(1);
    end
    index_at_4 = depth_view_index(y);
    
    depth_diff = abs(depth_view - point7);
    closest_to_7 = min(depth_diff);
    [x, y] = find(depth_diff == closest_to_7);
    if length(y) > 1
        y = y(1);
    end
    index_at_7 = depth_view_index(y);
    
    
    
    
    
    %% Find speed
    %cutTemp = cut_view(find(cut_view_index == index_at_b):find(cut_view_index == index_at_a));
    cutTemp = cut_view(valid_index-horizon:length(cut_view));
    thresTemp = find(cutTemp > danger_height,1);
    peakHeight = -99;
    peakIndex = -1;
    waveFrontHiehgt = -99;
    waveFrontIndex = -1;
    waveBackHiehgt = -99;
    waveBackIndex = -1;
    notSetBack = true;
    if ~isempty(thresTemp)
        for i = 1:length(cutTemp)% Get the maximum wave heghit and index
            if cutTemp(i) > peakHeight
                peakHeight = cutTemp(i);
                peakIndex = i;
            end
            if cutTemp(i) > danger_height
                if notSetBack
                    waveBackHiehgt = cutTemp(i);
                    waveBackIndex = i;
                    notSetBack = false;
                end
                waveFrontHiehgt = cutTemp(i);
                waveFrontIndex = i;
            end
        end
    
        peakIndex = peakIndex + valid_index -1;
        waveFrontIndex = waveFrontIndex + valid_index - 1;
        waveBackIndex = waveBackIndex + valid_index - 1;
        if ~isempty(waveTop) && ~isempty(waveFront)
            speedFront = (waveFront(end,2) - depth_view(find(depth_view_index == waveFrontIndex)))/(1/15);
            speedBack = (waveBack(end,2) - depth_view(find(depth_view_index == waveBackIndex)))/(1/15);
            
            if waveFront(end,4) == waveFrontIndex
                speedFront = 0;
            end
            
            if waveBack(end,4) == waveBackIndex
                speedBack = 0;
            end
            
            if waveTop(end,4) == peakIndex
                speedTop = speedBack;
            else
                speedTop = (waveTop(end,2) - depth_view(find(depth_view_index == peakIndex)))/(1/15);
            end


            if speedTop < 0
                speedTop = 0;
            end
            if speedFront < 0
                speedFront = 0;
            end
            if speedBack < 0
                speedBack = 0;
            end
            
            

        else
            speedTop = 0;
            speedFront = 0;
            speedBack = 0;
        end
    
    
        waveTop(end + 1, :) = [peakHeight,depth_view(peakIndex - horizon),speedTop,peakIndex,msg_num];
        waveFront(end + 1, :) = [waveFrontHiehgt,depth_view(waveFrontIndex - horizon),speedFront,waveFrontIndex,msg_num];
        waveBack(end + 1, :) = [waveBackHiehgt,depth_view(waveBackIndex - horizon),speedBack,waveBackIndex,msg_num];

    else
        waveTop(end + 1, :) = [0, 0, 0, 0, msg_num];
        waveFront(end + 1, :) = [0, 0, 0, 0, msg_num];
        waveBack(end + 1, :) = [0, 0, 0, 0, msg_num];
    end
    
    sectorized_chopping = chopped_pts(index_at_b-horizon:index_at_a-horizon,:,:);
    cut_view_cp = cut_view;
    cut_view_index_cp = cut_view_index;
    chopped_cp = chopped_pts;
    depth_view_cp = depth_view;
    depth_view_index_cp = depth_view_index;
    
    %% check if the horizon in the valid range
    if valid_index > horizon
        final = length(cut_view);
        cut_view = cut_view(valid_index-horizon:length(cut_view));
        cut_view_index = cut_view_index(valid_index-horizon:length(cut_view_index));
        depth_view = depth_view(valid_index-horizon:length(depth_view));
        chopped_pts = chopped_pts(valid_index-horizon:final,:,:);
    end
    
    
    
    % do a divide and valid
    minimal1_4 = [];
    minimal4_7 = [];
    covPts = [];
    threshold = find(cut_view > danger_height);
    [g, temp] = size(threshold);
    if temp > 0
        filled_value = zeros(1,temp);
        for i=1:temp
            filled_value(i) = cut_view(threshold(i));
        end
        
        threshold1_4 = find(cut_view_cp( find(cut_view_index_cp == index_at_4):find(cut_view_index_cp == index_at_1) ) > danger_height);
        [g, temp1_4] = size(threshold1_4);
        
        threshold4_7 = find(cut_view_cp( find(cut_view_index_cp == index_at_7):find(cut_view_index_cp == index_at_4) ) > danger_height);
        [g, temp4_7] = size(threshold4_7);
        
        if temp1_4 > 0
            covPts = chopped_cp( find(cut_view_index_cp == index_at_4):find(cut_view_index_cp == index_at_1),:,:);
            color_img = color_img(find(cut_view_index_cp == index_at_4):find(cut_view_index_cp == index_at_1),:,:);
            [minimal1_4, minimal_group, maxWaveHeight, firstWaveHeight, backWaveHeight, x, check] = divide_valid(chopped_cp( find(cut_view_index_cp == index_at_4):find(cut_view_index_cp == index_at_1),:,:), divide_num, danger_height, msg_num);
            count = 0;
            for i=1:length(minimal1_4)
                temp = depth_view_index_cp(find(depth_view_cp == minimal1_4(i,2)));
                if length(temp) == 0
                    depth_diff = abs(depth_view_cp - minimal1_4(i,2));
                    closest_one = min(depth_diff);
                    [x, y] = find(depth_diff == closest_one);
                    if length(y) > 1
                        y = y(1);
                    end
                    temp = depth_view_index_cp(y);
                end
                    
                if temp(1) >= index_at_1 - 5 
                    count = count + 1;
                end
            end
            if count >= 0.5*size(minimal1_4)
                wavePassed = 1;
                minimal1_4 = [];
                covPts = [];
            end
        end
        
        if temp4_7 > 0
            [minimal4_7, minimal_group, maxWaveHeight, firstWaveHeight, backWaveHeight, x, check] = divide_valid(chopped_cp( find(cut_view_index_cp == index_at_7):find(cut_view_index_cp == index_at_4),:,:), divide_num, danger_height, msg_num);
        end
    else
        minimal = [];
        
    end
    
    
    copy_cut_view = cut_view(find(cut_view_index == index_at_b):find(cut_view_index == index_at_a));
    window_threshold = find(copy_cut_view > danger_height);
    [g, temp] = size(window_threshold);
    if temp > 0
        [R, minimal_group, maxWaveHeight, firstWaveHeight, backWaveHeight, x, check] = divide_valid(sectorized_chopping, divide_num, danger_height, msg_num);
        
        for i=1:sum(check)
            max_height_tracker(minimal_group(i),index_count,1) = msg_num;
            max_height_tracker(minimal_group(i),index_count,2) = maxWaveHeight(i,2);
            max_height_tracker(minimal_group(i),index_count,3) = maxWaveHeight(i,3);
            max_height_tracker(minimal_group(i),index_count,5) = maxWaveHeight(i,1) + 480 - x;
            
            first_height_tracker(minimal_group(i),index_count,1) = msg_num;
            first_height_tracker(minimal_group(i),index_count,2) = firstWaveHeight(i,2);
            first_height_tracker(minimal_group(i),index_count,3) = firstWaveHeight(i,3);
            first_height_tracker(minimal_group(i),index_count,5) = firstWaveHeight(i,1) + 480 - x;
            
            back_height_tracker(minimal_group(i),index_count,1) = msg_num;
            back_height_tracker(minimal_group(i),index_count,2) = backWaveHeight(i,2);
            back_height_tracker(minimal_group(i),index_count,3) = backWaveHeight(i,3);
            back_height_tracker(minimal_group(i),index_count,5) = backWaveHeight(i,1) + 480 - x;
            
            
            if index_count > 1
                previous_depth = first_height_tracker(minimal_group(i),index_count-1,3);
                current_depth = first_height_tracker(minimal_group(i),index_count,3);
                if previous_depth > current_depth
                    first_height_tracker(minimal_group(i),index_count,4) = (previous_depth - current_depth)/(msg_num - first_height_tracker(minimal_group(i),index_count-1,1))*15;
                else
                    first_height_tracker(minimal_group(i),index_count,4) = 0;
                end
                
                previous_depth = back_height_tracker(minimal_group(i),index_count-1,3);
                current_depth = back_height_tracker(minimal_group(i),index_count,3);
                if previous_depth > current_depth
                    back_height_tracker(minimal_group(i),index_count,4) = (previous_depth - current_depth)/(msg_num - first_height_tracker(minimal_group(i),index_count-1,1))*15;
                else
                    back_height_tracker(minimal_group(i),index_count,4) = 0;
                end
                
                previous_depth = max_height_tracker(minimal_group(i),index_count-1,3);
                current_depth = max_height_tracker(minimal_group(i),index_count,3);
                previous_index = max_height_tracker(minimal_group(i),index_count-1,5);
                current_index = max_height_tracker(minimal_group(i),index_count,5);
                if previous_index == current_index
                    max_height_tracker(minimal_group(i),index_count,4) = back_height_tracker(minimal_group(i),index_count,4);
                else
                    if previous_depth > current_depth
                        max_height_tracker(minimal_group(i),index_count,4) = (previous_depth - current_depth)/(msg_num - max_height_tracker(minimal_group(i),index_count-1,1))*15;
                    else
                        max_height_tracker(minimal_group(i),index_count,4) = 0;
                    end
                end
                
            else
                max_height_tracker(minimal_group(i),index_count,4) = 0;
                first_height_tracker(minimal_group(i),index_count,4) = 0;
                back_height_tracker(minimal_group(i),index_count,4) = 0;
            end
        end
        index_count = index_count + 1;
    end
        
    
    if valid_index > horizon
        threshold = threshold + valid_index - 1;
    else
        threshold = threshold + horizon;
    end
    
    %% convert threshold from index tracing to depth tracing
    for i=1:length(threshold)
        threshold(i) = depth_view(find(cut_view_index == threshold(i)));
    end
    
    % color the height by threshold
    range = 0;
    if valid_index > horizon
        range = valid_index;
    else
        range = horizon;
    end
    for i=1:(480-range)
        for j=1:640
            if chopped_pts(i,j,2) > danger_height
                reshaped_im(i+range,j,1) = 255;
                reshaped_im(i+range,j,2) = 0;
                reshaped_im(i+range,j,3) = 0;
            end
        end
    end
    reshaped_im = imrotate(reshaped_im,rotateAngle);
    reshaped_im = imtranslate(reshaped_im,[0, diff]);
    %reshaped_im = reshape(reshaped_im, [480*640, 3]);
    
    %% Visualization
    % Point cloud plot
%     subplot(h1);
%     % So the last view is saved
%     [az,el] = view;
%     % Get the colors for the points (either solid colors or gradients)
%     if is_color
%         pts_color = colors(color_ind);
%     else
%         %pts_color = pts(:,grad_ind);
%         pts_color = double(reshaped_imCloud)/255;
%     end
%     h_pts = scatter3(pts(:,1),pts(:,2),pts(:,3),6,pts_color,'filled','s');
%     s = sprintf('Point Cloud\nMessage Number %d\n',msg_num);
%     title(s);
%     set(gca,'color','k','gridcolor','w','linewidth',2);
%     grid on;
    % RGB image display
    subplot(h2);
    set(h_im3,'cdata',im);
    set(h_im,'cdata',reshaped_im);
    s = sprintf('RGB Image\nMessage Number %d\n',msg_num);
    title(s, 'FontSize', 16);
    
    %% direction line
%     if exist('dl','var')
%        delete(dl);
%     end
%     if exist('raw_check','var')
%        delete(raw_check);
%     end
%     if size(minimal) > 0
%         [bestParameter1,bestParameter2] = ransac_demo(transpose(minimal),2,100,50,0.5);
%         xLine = 1:640;
%         yLine = bestParameter1*xLine + bestParameter2;
%         for i = 1:640
%             xLine(i) = xLine(i) - center(2);
%             yLine(i) = yLine(i) - center(1);
%         end
%         res = [xLine(:),yLine(:)];
%         R2d = [cosd(rotateAngle) -sind(rotateAngle);sind(rotateAngle) cosd(rotateAngle)];
%         res = res * R2d;
%         xLine = res(:,1);
%         yLine = res(:,2);
%         for i = 1:640
%             xLine(i) = xLine(i) + center2(2);
%             yLine(i) = yLine(i) + center2(1) + diff;
%         end
%        
%         dl = plot(xLine,yLine,'green','lineWidth',1);
%         raw_check = plot(minimal(:,1),minimal(:,2),'yellow','lineWidth',1);
%         
%     end
    
    % Set the az-el for the point cloud in case change under pause...
%     subplot(h1);
%     view(az,el);

    %% set the cut view of height vs somthing
    subplot(h3);
    plot(depth_view, cut_view,'k','lineWidth',2);
    hold on;
    %line([3 3],[0 1.2],'Color','g');
    %hold on;
    %line([6 6],[0 1.2],'Color','g');
    if peakIndex ~= -1
        hold on;
        peakIndex = depth_view(find(cut_view_index == peakIndex));
        %line([peakIndex peakIndex], [0 peakHeight]);
    end
    if waveFrontIndex ~= -1
        hold on;
        waveFrontIndex = depth_view(find(cut_view_index == waveFrontIndex));
        %line([waveFrontIndex waveFrontIndex], [0 waveFrontHiehgt]);
    end
    if waveBackIndex ~= -1
        hold on;
        waveBackIndex = depth_view(find(cut_view_index == waveBackIndex));
        %line([waveBackIndex waveBackIndex], [0 waveBackHiehgt]);
    end
    if temp > 0
        hold on;
        %mc = area(threshold, filled_value, 0);
        %mc.FaceColor = 'blue';
        %hold on;
        %ac = area(threshold, filled_value, danger_height);
        %ac.FaceColor = 'red';
        
        %fill(threshold, filled_value,'r');
        %hold on;
        cut_view_cp = cut_view;
        cut_view_cp(find(cut_view_cp >= 0.8)) = 0.8;
        mc = area(depth_view, cut_view_cp, 0);
        mc.FaceColor = 'blue';
        mc.LineStyle = 'none';
        hold on;
        
        ac = area(threshold, filled_value, 0.8);
        ac.FaceColor = 'red';
        ac.LineStyle = 'none';
        hold on;
        
        filled_value(find(filled_value >= 0.8)) = 0.8;
        mc = area(threshold, filled_value, 0);
        mc.LineStyle = 'none';
        mc.FaceColor = 'blue';
        
        
        hold on;
        
        hold off;
    else
        mc = area(depth_view, cut_view, 0);
        mc.FaceColor = 'blue';
        hold on;
    end
    
    
    
    
    xlim([2.8 10]);
    ylim([0 1.2]);
    %set(cut, 'Color', 'blue');
    s = sprintf('height vs distance\n');
    title(s, 'FontSize', 16);
    xlabel('depth(m)','FontSize',16); 
    ylabel('height(m)','FontSize',16); 
    hold off;
    
    %% plot f_to_h (depth vs frame rate)
%     subplot(h4);
%     if ~isempty(thresTemp)
%         f_to_d(end+1,1:2) = [msg_num, waveTop(end,2)];
%     else
%         f_to_d(end+1,1:2) = [msg_num, 0];
%     end
%     p = plot(f_to_d(:,1),f_to_d(:,2),'lineWidth',2);
%     set(p, 'Color', 'red');
%     s = sprintf('peak depth vs. frame rate \n');
%     title(s, 'FontSize', 16);
%     x = f_to_h(:,1);
%     if x(end) - 300 < 0
%         xLow = initial_num - 1;
%     else
%         xLow = x(end) - 300;
%     end
%     axis([xLow x(end) 0 inf]);
%     ylabel('depth(m)', 'FontSize',16); 
%     xlabel('frame number','FontSize',16);

    %% plot depth direction fit
%     subplot(h4);
%     if exist('dl','var')
%        delete(dl);
%     end
%     if exist('raw_check','var')
%        delete(raw_check);
%     end
%     line([0, 640], [point1, point1]);
%     hold on;
%     line([0, 640], [point4, point4]);
%     hold on;
%     if size(minimal1_4) > 0
%         [bestParameter1,bestParameter2] = ransac_demo(transpose(minimal1_4),2,100,0.5,0.2);
%         xLine = 1:640;
%         yLine = bestParameter1*xLine + bestParameter2;
%         
%        
%         dl = plot(xLine,yLine,'green','lineWidth',1);
%         hold on;
%         raw_check = scatter(minimal1_4(:,1),minimal1_4(:,2));
%         hold off;
%     else
%         bestParameter1 = 0;
%     end
%     
% %     if size(minimal4_7) < 0
% %         [bestParameter1,bestParameter2] = ransac_demo(transpose(minimal4_7),2,100,1,0.5);
% %         xLine = 1:640;
% %         yLine = bestParameter1*xLine + bestParameter2;
% %         
% %        
% %         dl = plot(xLine,yLine,'green','lineWidth',1);
% %         hold on;
% %         raw_check = scatter(minimal4_7(:,1),minimal4_7(:,2));
% %         hold off;
% %     else
% %         hold off;
% %     end
%     
%     xlim([0 640]);
%     ylim([0 6]);
%     ylabel('depth(m)', 'FontSize',16); 
%     xlabel('column','FontSize',16);
%     s = sprintf('Direction at depth, slope: %.2f\n', bestParameter1*640/6);
%     title(s, 'FontSize', 16);
    
    %% plot cov
    subplot(h5);
    if exist('co','var')
       delete(co);
    end
    if exist('el','var')
       delete(el);
    end
    if exist('di','var')
       delete(di);
    end
    if exist('nanWave','var')
       delete(nanWave);
    end
%     line([-3, 3], [point1, point1]);
%     hold on;
%     line([-3, 3], [point4, point4]);
%     hold on;
    slope = 0;
    if wavePassed == 1
        prev_slope = 0;
    end
    if(size(covPts) > 0)
        [m, n, k] = size(covPts);
        
        % doing a stupid wave check
        covPts_cp = covPts;
        for i=1:m
            for j=1:n
                if covPts_cp(i,j,2) < 0.8
                    covPts_cp(i,j,3) = nan;
                end
            end
        end
        depth_check = zeros(m, 1);
        for i=1:m
            depth_check(i) = nanmedian(covPts_cp(i,:,3));
        end
        
        index_nan = find(~isnan(depth_check));
        depth_check_cp = depth_check(index_nan);
        index1 = 1;
        index2 = 2;
        max_diff = depth_check_cp(1) - depth_check_cp(2);
        for i=3:length(depth_check_cp)
            temp = depth_check_cp(i-1) - depth_check_cp(i);
            if  temp > max_diff
                max_diff = depth_check_cp(i-1) - depth_check_cp(i);
                index1 = i-1;
                index2 = i;
            end
        end
        realIndex1 = find(depth_check == depth_check_cp(index1));
        realIndex2 = find(depth_check == depth_check_cp(index2));
        % 2 waves, we pick the one larger
        if max_diff >= 0.3
            temp = reshape(covPts, [m*n, k]);
            avg_depth = (depth_check_cp(index1) + depth_check_cp(index2))/2;
            count_col = 0;
            count_col1 = 0;
            for i=1:640
                if find( covPts_cp(:,i,3) < avg_depth )
                    count_col = count_col + 1;
                end
                if find( covPts_cp(:,i,3) >= avg_depth )
                    count_col1 = count_col1 + 1;
                end
            end
            if count_col < count_col1%abs(avg_depth-top_rep) > abs(avg_depth-bot_rep)
                covPts(realIndex2:length(depth_check),:,2) = 0.6666;
            else
                covPts(1:realIndex1,:,2) = 0.6666;
            end
        end
        
        covPts = reshape(covPts, [m*n, k]);
        color_img = reshape(color_img, [m*n, k]);
        includePts = find(covPts(:,2) >= 0.8);
        notIncPts = find(covPts(:,2) < 0.8);
        color_img = color_img(notIncPts,:);
        notWave = covPts(notIncPts,:);
        covPts = covPts(includePts,:);
        covPts_cp = covPts;
        covPts = covPts(:,[1, 3]);
        A = cov(covPts);
        [eigenvec, eigenval] = eig(A);
        
        % Get the index of the largest eigenvector
        [largest_eigenvec_ind_c, r] = find(eigenval == max(max(eigenval)));
        largest_eigenvec = eigenvec(:, largest_eigenvec_ind_c);

        % Get the largest eigenvalue
        largest_eigenval = max(max(eigenval));
        
        % Get the smallest eigenvector and eigenvalue
        if(largest_eigenvec_ind_c == 1)
            smallest_eigenval = max(eigenval(:,2));
            smallest_eigenvec = eigenvec(:,2);
        else
            smallest_eigenval = max(eigenval(:,1));
            smallest_eigenvec = eigenvec(1,:);
        end
        
        % Calculate the angle between the x-axis and the largest eigenvector
        angle = atan2(largest_eigenvec(2), largest_eigenvec(1));
        
        % This angle is between -pi and pi.
        % Let's shift it such that the angle is between 0 and 2pi
        if(angle < 0)
            angle = angle + 2*pi;
        end
        
        % Get the coordinates of the data mean
        avg = mean(covPts);
        
        % Get the 95% confidence interval error ellipse
        chisquare_val = 2.4477;
        theta_grid = linspace(0,2*pi);
        phi = angle;
        X0=avg(1);
        Y0=avg(2);
        a=chisquare_val*sqrt(largest_eigenval);
        b=chisquare_val*sqrt(smallest_eigenval);

        % the ellipse in x and y coordinates 
        ellipse_x_r  = a*cos( theta_grid );
        ellipse_y_r  = b*sin( theta_grid );
        
        %Define a rotation matrix
        R = [ cos(phi) sin(phi); -sin(phi) cos(phi) ];
        
        %let's rotate the ellipse to some angle phi
        r_ellipse = [ellipse_x_r;ellipse_y_r]' * R;
        
        % Draw the error ellipse
        el = plot(r_ellipse(:,1) + X0,r_ellipse(:,2) + Y0, 'red', 'lineWidth', 1);
        hold on;
        
        P0 = [X0, Y0];
        V = [ largest_eigenvec(1)*sqrt(largest_eigenval), largest_eigenvec(2)*sqrt(largest_eigenval)];
        P1 = P0 + V;
        slope = (P1(2) - P0(2)) ./ (P1(1) - P0(1));
        
        V = [ smallest_eigenvec(1)*sqrt(smallest_eigenval), smallest_eigenvec(2)*sqrt(smallest_eigenval)];
        P1 = P0 + V;
        slope_1 = (P1(2) - P0(2)) ./ (P1(1) - P0(1));
        
        if abs(slope1) < abs(slope)
            slope = slope_1;
        end
        
        if prev_slope ~= 0 
            if abs(slope-prev_slope) / abs(prev_slope) > 0.3
                slope = prev_slope;
            end
        end
        prev_slope = slope;
        x = -3:0.1:3;
        y = slope(1)*(x - X0) + Y0;
        di = plot(x,y,'green','lineWidth',1);
        
        %scatter([P0(1), P1(1)], [P0(2), P1(2)]);
        %quiver(X0, Y0, largest_eigenvec(1)*sqrt(largest_eigenval), largest_eigenvec(2)*sqrt(largest_eigenval), '-m', 'LineWidth',2);
        
        % calculate the color
        [m, n] = size(covPts_cp);
        pts_color = zeros(m, 1);
        
        pointsize = 8;
        max_h = max(covPts_cp(:,2));
        min_h = min(covPts_cp(:,2));
        %[counts, ccode] = histc(covPts_cp(:,2),[-inf, 0.3, 0.8, inf]);
        for i=1:m
            if covPts_cp(i,2) >= ((max_h - min_h)*0.7+min_h)
                pts_color(i) = 5;
            elseif covPts_cp(i,2) >= ((max_h - min_h)*0.5+min_h)
                pts_color(i) = 4;
            elseif covPts_cp(i,2) >= ((max_h - min_h)*0.3+min_h)
                pts_color(i) = 3;
            elseif covPts_cp(i,2) >= ((max_h - min_h)*0.1+min_h)
                pts_color(i) = 2;
            else
                pts_color(i) = 1;
            end
        end
        
        
        co = scatter3(covPts_cp(:,1), covPts_cp(:,3),covPts_cp(:,2),pointsize, pts_color);
        colormap(autumn);
        view(50,35);%10 80
        hold on;
        nanWave = scatter3(notWave(:,1), notWave(:,3),notWave(:,2),pointsize,double(color_img)/255);
        
        hold off;
    end
    
    slope_direction(msg_num,:) = [slope, bestParameter1*640/6];
    
    xlim([-3 3]);
    ylim([1.5 6]);
    zlim([0 1.2]);
    ylabel('depth(m)', 'FontSize',16); 
    xlabel('camera X-axis','FontSize',16);
    zlabel('height(m)','FontSize',16);
    s = sprintf('Direction at depth, slope: %.2f\n',slope);
    title(s, 'FontSize', 16);
    

    
    
    msg_num = msg_num+1;
    
    drawnow;
    if is_MAX
        warning('off');
        frame_h = get(handle(gcf),'JavaFrame');
        set(frame_h,'Maximized',1);
        pause(1);
        warning('on');
        fprintf('Maximized ... \n');
        is_MAX = false;
    else
        f = getframe(gcf);
        %writeVideo(v,f);
    end
%     save('./x.mat','X');
%     save('./waveTop.mat','waveTop');
%     save('./waveFront.mat','waveFront');
%     save('./waveBack.mat','waveBack');
%     save('./sectorTop.mat','max_height_tracker');
%     save('./sectorFront.mat','first_height_tracker');
%     save('./sectorBack.mat','back_height_tracker');
%     save('./depthFrame.mat','f_to_d');
%    save('./slope.mat','slope_direction');
end

%% Callbacks
    % Callback function for the pause button
    function pausecallback(~,~)
        is_paused = ~is_paused;
    end
    % Callback function for the stop button
    function stopcallback(~,~)
        is_stopped = true;
    end
    % Callback function for changing the pointcloud gradient coloring
    function gradcallback(~,~)
        grad_ind = grad_ind+1;
        if grad_ind>3
            grad_ind = 1;
        end
        is_color = false;
    end
    % Callback function for changing the pointcloud gradient coloring
    function colorcallback(~,~)
        color_ind = color_ind+1;
        if color_ind>numel(colors)
            color_ind = 1;
        end
        is_color = true;
    end
end


function addpushbutton( h_fun, h_fig, butt_type, varargin)
%% ADDPUSHBUTTON adds a quick button to the figure toolbar
%
%   ADDPUSHBUTTON( h_fun, h_fig, butt_type )
%   ADDPUSHBUTTON( h_fun, h_fig, butt_type, separator_on_off )

% Argument checks
if nargin==4
    separator = varargin{1};
else
    separator = 'off';
end
% Make the button image
butt = zeros(16,16,3);
if strcmpi(butt_type,'stop')
    butt(5:end-4,5:end-4,:) = 1;
    s = 'Stop the Application';
elseif strcmpi(butt_type,'pause')
    butt(5:end-4,5:7,:) = 1;
    butt(5:end-4,10:12,:) = 1;
    s = 'Pause the Application';
elseif strcmpi(butt_type,'gradient')
    butt = rand(16,16,3);
    s = 'Change the Point Cloud Gradient';
elseif strcmpi(butt_type,'color')
    butt(:,[1:4 13:end],1) = 1;
    butt(:,[5:8 13:end],2) = 1;
    butt(:,9:12,3) = 1;
    s = 'Change the Point Cloud Color';
else
    error('Specified button type not implemented.');
end
% Get the toolbar handle
tbar = findall(h_fig,'type','uitoolbar');
% Associate the image and the function handle for the callback
uipushtool(tbar,'cdata',butt,'handlevisibility','off','clickedcallback',...
    h_fun,'tooltipstring',s,'separator',separator);
end
function height = findHeight(A)
sumHeight = 0;
count = 0;
for i = 1:length(A)
    if A(i,3)< 15
        sumHeight = sumHeight + A(i,3);
        count = count + 1;
    end
end
height = sumHeight/count;
%disp(height);
end
function R = chopcloud(A, l, axis)
R = A;
for i = 1:length(A)
    if A(i,axis)< l
        R(i,:) =  A(i,:);
    else
        R(i,:) = [NaN NaN NaN];
    end
end
end
function R = fcn_RotationFromTwoVectors(A, B)
v = cross(A,B);
Norm = norm(v);
Dot = dot(A,B);
I=[1 0 0; 0 1 0; 0 0 1];
vH = [0 -v(3) v(2);v(3) 0 -v(1); -v(2) v(1) 0];
R = I + vH + vH^2 * 1/(1+Dot);
end
function [bestParameter1,bestParameter2] = ransac_demo(data,num,iter,threshDist,inlierRatio)
 % data: a 2xn dataset with #n data points
 % num: the minimum number of points. For line fitting problem, num=2
 % iter: the number of iterations
 % threshDist: the threshold of the distances between points and the fitting line
 % inlierRatio: the threshold of the number of inliers 
 
 %% Plot the data points
%  figure;plot(data(1,:),data(2,:),'o');hold on;
 number = size(data,2); % Total number of points
 bestInNum = 0; % Best fitting line with largest number of inliers
 bestParameter1=0;bestParameter2=0; % parameters for best fitting line
 for i=1:iter
 %% Randomly select 2 points
     idx = randperm(number,num); sample = data(:,idx);   
 %% Compute the distances between all points with the fitting line 
     kLine = sample(:,2)-sample(:,1);% two points relative distance
     kLineNorm = kLine/norm(kLine);
     normVector = [-kLineNorm(2),kLineNorm(1)];%Ax+By+C=0 A=-kLineNorm(2),B=kLineNorm(1)
     distance = normVector*(data - repmat(sample(:,1),1,number));
 %% Compute the inliers with distances smaller than the threshold
     inlierIdx = find(abs(distance)<=threshDist);
     inlierNum = length(inlierIdx);
 %% Update the number of inliers and fitting model if better model is found     
     if inlierNum>=round(inlierRatio*number) && inlierNum>bestInNum
         bestInNum = inlierNum;
         parameter1 = (sample(2,2)-sample(2,1))/(sample(1,2)-sample(1,1));
         parameter2 = sample(2,1)-parameter1*sample(1,1);
         bestParameter1=parameter1; bestParameter2=parameter2;
     end
 end
 
%  %% Plot the best fitting line
%  xAxis = -number/2:number/2; 
%  yAxis = bestParameter1*xAxis + bestParameter2;
%  plot(xAxis,yAxis,'r-','LineWidth',2);
end

% divide and valid
function [R, minimal_group, maxWaveHeight, firstWaveHeight, backWaveHeight, x, check] = divide_valid(points, divide_num, wave_height, msg_num)
    R = [];
    [x, y, z] = size(points);
    % divide points into divide_num parts
    divide_ma = zeros(divide_num, x, round(y/divide_num), z);
    for i=1:divide_num
        for j=1:x
            for k=1:round(y/divide_num)
                for m=1:3
                    divide_ma(i,j,k,m) = points(j,k+(i-1)*round(y/divide_num),m);
                end
            end
        end
    end
    % check if each section row is greater than danger height
    medHeight = zeros(divide_num, x);
    medIndex = zeros(divide_num, x);
    medDepth = zeros(divide_num, x);
    for i=1:divide_num
        for j=1:x
            medHeight(i,j) = nanmedian(divide_ma(i,j,:,2));
            medDepth(i,j) = nanmedian(divide_ma(i,j,:,3));
            medIndex(i,j) = x+j;
        end
    end
    [row, col] = find(medHeight > wave_height);
    
    % all the sections have waves beyond threshold
    check = zeros(divide_num,1);
    for i=1:divide_num
        check(i,1) = ismember(i, row);
    end
    [r, t] = find(check > 0);
    
    minimal = [sum(check),2];
    minimal_group = r;
    combine = horzcat(row, col);
    combine = sortrows(combine, 1);
    
    % store index, height, depth to the max and first wave height
    maxWaveHeight = [sum(check), 3];
    firstWaveHeight = [sum(check), 3];
    backWaveHeight = [sum(check),3];
    for i=1:sum(check)
        maxWaveHeight(i,2) = max(medHeight(r(i),:));
        maxWaveHeight(i,1) = find(medHeight(r(i),:) == max(medHeight(r(i),:)));
        maxWaveHeight(i,3) = medDepth(r(i),maxWaveHeight(i,1));
    end
    
    
    count = 1;
    backWaveHeight(1,2) = medHeight(combine(1,1),combine(1,2));
    backWaveHeight(1,1) = find(medHeight(combine(1,1),:) == backWaveHeight(1,2));
    backWaveHeight(1,3) = medDepth(combine(1,1), backWaveHeight(1,1));
    for i=1:size(col)
        if count >= size(minimal_group)
            break;
        end
        if combine(i,1) == minimal_group(count+1)
            minimal(count,2) = medDepth(combine(i-1,1),combine(i-1,2));%combine(i-1,2)+(480-x);
            firstWaveHeight(count,2) = medHeight(combine(i-1,1),combine(i-1,2));
            firstWaveHeight(count,1) = find(medHeight(combine(i-1,1),:) == firstWaveHeight(count,2));
            firstWaveHeight(count,3) = medDepth(combine(i-1,1), firstWaveHeight(count,1));
            
            backWaveHeight(count+1,2) = medHeight(combine(i,1),combine(i,2));
            backWaveHeight(count+1,1) = find(medHeight(combine(i,1),:) == backWaveHeight(count+1,2));
            backWaveHeight(count+1,3) = medDepth(combine(i,1), backWaveHeight(count+1,1));
            
            count = count + 1;
        end
        
    end
    minimal(sum(check),2) = medDepth(combine(end,1),combine(end,2));%combine(end,2)+480-x;
    minimal_group(sum(check)) = combine(end,1);
    firstWaveHeight(sum(check),2) = medHeight(combine(end,1),combine(end,2));
    firstWaveHeight(sum(check),1) = find(medHeight(combine(end,1),:) == firstWaveHeight(sum(check),2));
    firstWaveHeight(sum(check),3) = medDepth(combine(end,1), firstWaveHeight(sum(check),1));
   
    for i=1:sum(check)
        minimal(i,1) = 640/divide_num/2+640/divide_num*(minimal_group(i)-1);
    end
    % return just the direction matrix
    R = minimal;
    
    % we store the information to the real divide tracker matrix for each
    % group
    
    
    
        
%         valid_pts = [];
%         for i=1:size(col)
            % all points xz
%             temp_row_vector = find(divide_ma(row(i), col(i),:,2) > wave_height);
%             size(temp_row_vector)
%             for j=1:length(temp_row_vector)
%                 valid_pts(end+1, 1:2) = [divide_ma(row(i),col(i),temp_row_vector(j),1), divide_ma(row(i),col(i),temp_row_vector(j),3)];
%             end
            % only median points
%             diff = divide_ma(row(i),col(i),:,2) - medHeight(row(i),col(i));
%             median_index = find(diff == min(diff));
%             valid_pts(end+1, 1:2) = [divide_ma(row(i),col(i),median_index(1),1),divide_ma(row(i),col(i),median_index(1),3)];
%         end
%         p = polyfit(valid_pts(:,1),valid_pts(:,2),1);
%         f = polyval(p,valid_pts(:,1));
%         plot(valid_pts(:,1),valid_pts(:,2),'o',valid_pts(:,1),f,'-');
    
end

