bagFolderPath = '/home/alaa/ros_bags/Calibration/2023-06-04_calibration-20231113T170459Z-002/2023-06-04_calibration/2023-06-04_calibration_0-001.db3';
bagReader = ros2bagreader(bagFolderPath);

baginfo = ros2("bag","info","2023-06-04_calibration_0-001.db3");

lidarTopic = '/lidar/center/bottom'; % Access the 9th element from the cell array

cameraTopic = '/camera/center_wide/compressed';

LidarSel = select(bagReader,"Topic",lidarTopic);

CameraSel = select(bagReader,"Topic",cameraTopic);

cameramsgsFiltered = readMessages(CameraSel);

lidarmsdsFiltered = readMessages(LidarSel);


cameraIndices2 = [];
lidarIndices2 = [];
timeThreshold = 0.03;

minindex = 1;  % Initialize minindex outside the loop

for camIndx = 10:10:length(cameramsgsFiltered)
    closestTimeDiffSec = inf;
    closestTimeDiffNano = inf;
    currCamMsg = cameramsgsFiltered(camIndx);
    currCamTimeSec = currCamMsg{1,1}.header.stamp.sec;
    currCamTimeNano = currCamMsg{1,1}.header.stamp.nanosec;

    for lidarIdx = minindex:length(lidarmsdsFiltered)
        currLidMsg = lidarmsdsFiltered(lidarIdx);
        currLidTimeSec = currLidMsg{1,1}.header.stamp.sec;
        currLidTimeNano = currLidMsg{1,1}.header.stamp.nanosec;

        secdiff = abs(currCamTimeSec - currLidTimeSec);
        nanodiff = abs(currCamTimeNano - currLidTimeNano);

        if (double(secdiff) + double(nanodiff) * 1e-9) <= timeThreshold
            if secdiff <= closestTimeDiffSec && nanodiff <= closestTimeDiffNano
                closestTimeDiffSec = secdiff;
                closestTimeDiffNano = nanodiff;
                minindex = lidarIdx;  % Update minindex for the next iteration
            end
        elseif currLidTimeSec > currCamTimeSec  % Optimize: Break if LIDAR time exceeds camera time
            break;
        end
    end

    cameraIndices2 = [cameraIndices2, camIndx];
    lidarIndices2 = [lidarIndices2, minindex];
end

disp('Camera Indices:');
disp(cameraIndices2);
disp('Lidar Indices:');
disp(lidarIndices2);

imageOutputFolder = '/home/alaa/AutoCalib/MatlabCalibration-002/camera';  % Set your desired output folder
lidarOutputFolder = '/home/alaa/AutoCalib/MatlabCalibration-002/lidar';
for pairIdx = 1:length(cameraIndices2)
    camIdx = cameraIndices2(pairIdx);
    lidIdx = lidarIndices2(pairIdx);

    % Extract camera image
    camMsg = cameramsgsFiltered{camIdx};
    img = rosReadImage(camMsg, 'ImageFieldName');  % Replace 'ImageFieldName' with the appropriate field name
    imgFilename = fullfile(imageOutputFolder, [num2str(camIdx), '.png']);
    imwrite(img, imgFilename);

    % Extract LIDAR point cloud
    lidMsg = lidarmsdsFiltered{lidIdx};
    lidarData = rosReadXYZ(lidMsg);
    intensityData = rosReadField(lidMsg, 'intensity');  % Extract intensity data
    lidarFilename = fullfile(lidarOutputFolder, [num2str(camIdx), '.pcd']);
    % Save point cloud to PCD file
    pcwrite(pointCloud(lidarData, 'Intensity', intensityData), lidarFilename, 'Encoding', 'ascii');
end

    


