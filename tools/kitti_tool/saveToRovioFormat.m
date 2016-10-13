%%%%%%%% This file is uesed to save kitti dataset into rovio data format %%
% clear and close everything
close all; dbstop error; clc;
disp('======= KITTI transform IMU to Camera Demo =======');

% options (modify this to select your sequence)
  base_dir  = '/home/bob/source_code/data/kitti/rovio_longer/cam0';
  calib_dir = '/home/bob/source_code/data/kitti/rovio_longer/calibration';
  
   imu_dir  = '/home/bob/source_code/data/kitti/rovio_longer/imu0';
cam       = 0; % 0-based index
% load calibration
calib = loadCalibrationCamToCam(fullfile(calib_dir,'calib_cam_to_cam.txt'));
Tr_velo_to_cam = loadCalibrationRigid(fullfile(calib_dir,'calib_velo_to_cam.txt'));
Tr_imu_to_velo = loadCalibrationRigid(fullfile(calib_dir,'calib_imu_to_velo.txt'));
Tr_imu_to_cam  = Tr_velo_to_cam * Tr_imu_to_velo;

%load raw image and save them into the format of rovio
%load and display image
ts = loadTimestamps(base_dir);
for (i= 1 : length(ts))
    origin = char(ts{i});
    time1 = regexp(origin, ' ', 'split');
    tem = time1{2};
    time2 = regexp(tem, ':', 'split');
   
    str = char(time2{3});
    secTem = str2num(str(1:2));
    
    sec = str2num(char(time2{1}))*3600 + str2num(char(time2{2}))*60 + secTem;
    nansec = str(4:end);
    filename = sprintf('%s/data/%s',base_dir,strcat(num2str(sec),nansec, '.png'));
    bef = sprintf('%s/data/%010d.png',base_dir,i-1);
    movefile(bef,filename);
    
end

%%%%%%%%%%%%%%%%%%%%%load imu data
  ts = loadTimestamps(imu_dir);
      fid = fopen(sprintf('%s/data.csv', imu_dir),'wt');
  for i=1:length(ts)
    if ~isempty(ts{i})
      oxts = dlmread([imu_dir '/data/' num2str(i-1,'%010d') '.txt']);
    else
      oxts = [];
    end
    
    origin = char(ts{i});
    time1 = regexp(origin, ' ', 'split');
    tem = time1{2};
    time2 = regexp(tem, ':', 'split');
   
    str = char(time2{3});
    secTem = str2num(str(1:2));
    
    sec = str2num(char(time2{1}))*3600 + str2num(char(time2{2}))*60 + secTem;
    nansec = str(4:end);
    
    a = [oxts(12) oxts(13) oxts(14)];
    w = [oxts(18) oxts(19) oxts(20)];
    
    line = strcat(num2str(sec),nansec,',',num2str(w(1)),',',num2str(w(2)),',',num2str(w(3)),',',num2str(a(1)),',',num2str(a(2)),',',num2str(a(3)));
    
    fprintf(fid, '%s\r\n', line);
  end
