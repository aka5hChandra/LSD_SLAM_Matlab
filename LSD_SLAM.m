classdef LSD_SLAM < handle
   
    methods (Static)
        function demo
            clear;
            close all;
            
            offset_time = 10;
            %offset_time = 10.5;
            numImages = 100;
            verbose = false;
            MATLAB_minimize = false;
            
            bagfiles{1}='/home/akash/freiburg_data/rgbd_dataset_freiburg1_room.bag';
            bagfiles{2}='/home/arwillis/freiburg_data/rgbd_dataset_freiburg2_dishes.bag';
            bagfiles{3}='/home/arwillis/freiburg_data/rgbd_dataset_freiburg3_structure_notexture_far.bag';
            bagfiles{4}='/home/arwillis/freiburg_data/rgbd_dataset_freiburg3_structure_notexture_near.bag';
            bag = rosbag(bagfiles{1});
            
            timeInterval = offset_time + [bag.StartTime bag.StartTime+5];
            select_cam_info = select(bag, 'Time', ...
                timeInterval,'Topic','/camera/rgb/camera_info');
            select_rgb = select(bag, 'Time', ...
                timeInterval,'Topic','/camera/rgb/image_color');
            select_depth = select(bag, 'Time', ...
                timeInterval,'Topic','/camera/depth/image');
            
            
            cam_info_Msgs = readMessages(select_cam_info,1:numImages);
            rgb_Msgs = readMessages(select_rgb,1:numImages);
            depth_Msgs = readMessages(select_depth,1:numImages);
            
            
            myK = [254.327,0,267.382,0,375.938,231.599,0,0,1]';
            %cam_info_Msgs{1}.K = myK;
            
            
            cam_info.D = reshape(cam_info_Msgs{1}.D,1,5);
            cam_info.K = reshape(cam_info_Msgs{1}.K,3,3)';
            cam_info.P = reshape(cam_info_Msgs{1}.P,4,3)';
            
            
            
           
            
                slamSystem = SlamSystem(cam_info.K,480,640);
            
            prevT = [];
            %map = DepthMap(K,w,h);
            
            
            imageIdx = 12;
            stampA = depth_Msgs{imageIdx}.Header.Stamp.Sec+depth_Msgs{imageIdx}.Header.Stamp.Nsec*10^-9;
            fprintf(1,'Image A TimeStamp %15f\n', stampA);
            imageA = reshape(typecast(depth_Msgs{imageIdx}.Data,'single'),640,480)';
            imageA1 = reshape(typecast(rgb_Msgs{imageIdx}.Data,'uint8'),[3,640*480]);
            imageA2 = cat(3,reshape(imageA1(1,:),640,480)',reshape(imageA1(2,:),640,480)',reshape(imageA1(3,:),640,480)');
            rgbdImg1 = RGBDImage(imageA,imageA2,cam_info.K);
            
            initFrame =  Frame(imageIdx, imageA,imageA2,cam_info.K);
            %slamSystem.randomInit(initFrame,1);
            slamSystem.depthInit(initFrame,1);
            
            keyFrame = imageA2;
            
            imageIdx = 13;
            stampA = depth_Msgs{imageIdx}.Header.Stamp.Sec+depth_Msgs{imageIdx}.Header.Stamp.Nsec*10^-9;
            fprintf(1,'Image A TimeStamp %15f\n', stampA);
            imageA = reshape(typecast(depth_Msgs{imageIdx}.Data,'single'),640,480)';
            imageA1 = reshape(typecast(rgb_Msgs{imageIdx}.Data,'uint8'),[3,640*480]);
            imageA2 = cat(3,reshape(imageA1(1,:),640,480)',reshape(imageA1(2,:),640,480)',reshape(imageA1(3,:),640,480)');
            rgbdImg1 = RGBDImage(imageA,imageA2,cam_info.K);
            
             
            curFrame =  Frame(imageIdx, imageA,imageA2,cam_info.K);
            slamSystem.trackFrame(curFrame);
               
            for imageIdx=2:numImages-1
                stampA = depth_Msgs{imageIdx}.Header.Stamp.Sec+depth_Msgs{imageIdx}.Header.Stamp.Nsec*10^-9;
                fprintf(1,'Image A TimeStamp %15f\n', stampA);
                imageA = reshape(typecast(depth_Msgs{imageIdx}.Data,'single'),640,480)';
                imageA1 = reshape(typecast(rgb_Msgs{imageIdx}.Data,'uint8'),[3,640*480]);
                imageA2 = cat(3,reshape(imageA1(1,:),640,480)',reshape(imageA1(2,:),640,480)',reshape(imageA1(3,:),640,480)');
                rgbdImg1 = RGBDImage(imageA,imageA2,cam_info.K);
                
                %{
                stampB = depth_Msgs{imageIdx+1}.Header.Stamp.Sec+depth_Msgs{imageIdx+1}.Header.Stamp.Nsec*10^-9;
                fprintf(1,'Image B TimeStamp %15f\n', stampB);
                imageB = reshape(typecast(depth_Msgs{imageIdx+1}.Data,'single'),640,480)';
                imageB1 = reshape(typecast(rgb_Msgs{imageIdx+1}.Data,'uint8'),[3,640*480]);
                imageB2 = cat(3,reshape(imageB1(1,:),640,480)',reshape(imageB1(2,:),640,480)',reshape(imageB1(3,:),640,480)');
                rgbdImg2 = RGBDImage(imageB,imageB2,cam_info.K);
                
                %}
                
                curFrame =  Frame(imageIdx, imageA,imageA2,cam_info.K);
                slamSystem.trackFrame(curFrame);
                %cp = cameraParameters('IntrinsicMatrix',cameraParams.IntrinsicMatrix,'RadialDistortion',cameraParams.RadialDistortion,'ImageSize',cameraParams.ImageSize);  cameraParams = cp;
           
                
                
                %{
                                
                [rows,cols]=size(imageA);
                a_k = [0 0 0 0 0 0];
                T = rgbd_odom.parametersToTransform(a_k);

                %rgbd_odom.checkGradient([120,120],rgbdImg1, rgbdImg2);
                
                imgB_Z = rgbdImg2.getDepthImage();
                imgB_I = rgbdImg2.getGrayImage();
                warpA = rgbdImg1.warpImage(T);
                warpA_Z = warpA.getDepthImage();
                warpA_I = warpA.getGrayImage();
                
                
                %%slamSystem.
                
                %figure(2), subplot(2,3,1), imshow(imgB_Z,[0 8]);
                %figure(2), subplot(2,3,2), imshow(warpA_Z,[0 8]);
                %figure(2), subplot(2,3,3), imshow(abs(warpA_Z - imgB_Z),[ ]);
                figure(2), subplot(2,3,1), imshow(imgB_I,[0 255]);
                figure(2), subplot(2,3,2), imshow(warpA_I, [0 255]);
                figure(2), subplot(2,3,3), imshow(abs(warpA_I - imgB_I),[ ]);
                for downsample_level=3:-1:3
                    %a_k = [0 0 0 0 0 0];
                    downsample_factor=2^downsample_level;
                    rgbdImg1a = rgbdImg1.downsample( downsample_factor);
                    rgbdImg2a = rgbdImg2.downsample( downsample_factor);
                    
                    if (verbose==true)
                        T = rgbd_odom.parametersToTransform(a_k);
                        warpA = rgbdImg1a.warpImage(T);
                        warpA_Z = warpA.getDepthImage();
                        imgB_Z = rgbdImg2a.getDepthImage();
                        warpA_I = warpA.getGrayImage();
                        imgB_I = rgbdImg2a.getGrayImage();
                        %figure(1), subplot(2,3,1), imshow(imgB_Z,[0 8]);
                        %figure(1), sublot(2,3,2), imshow(warpA_Z,[0 8]);
                        %figure(1), subplot(2,3,3), imshow(abs(warpA_Z - imgB_Z),[ ]);
                        figure(1), subplot(2,3,1), imshow(imgB_I);
                        figure(1), subplot(2,3,2), imshow(warpA_I);
                        figure(1), subplot(2,3,3), imshow(abs(warpA_I - imgB_I),[ ]);
                    end
                    
                    if (MATLAB_minimize == true)
                        odomError = @(a_k0) rgbd_odom.errorFunc(rgbdImg1a, rgbdImg2a, a_k0);
                        options = optimoptions('lsqnonlin','Display','iter');
                        options.Algorithm = 'levenberg-marquardt';
                        options.FiniteDifferenceStepSize = [0.005 0.005 0.005 0.002 0.002 0.002];
                        %options.FiniteDifferenceStepSize = [0.05 0.05 0.05 0.001 0.001 0.001];
                        a_k0 = lsqnonlin(odomError,a_k,[],[],options)
                    end

                    if (false)
                        %options.FiniteDifferenceStepSize = [0.005 0.005 0.005 0.002 0.002 0.002];
                        options.FiniteDifferenceStepSize = 1e-6*ones(1,6);
                        odomError = @(a_k0) rgbd_odom.errorFunc(rgbdImg1a, rgbdImg2a, a_k0);
                        gradVal = 0.5*rgbd_odom.numericalGradient(odomError, a_k, options.FiniteDifferenceStepSize);
                        gradVal
                    end
                    
                    a_k = rgbd_odom.estimateOdometry(rgbdImg1a, rgbdImg2a, a_k);
                    a_k = a_k;j
                    
                    if (verbose==true)
                        T = rgbd_odom.parametersToTransform(a_k);
                        warpA2 = rgbdImg1a.warpImage(T);
                        warpA2_Z = warpA2.getDepthImage();
                        warpA2_I = warpA2.getGrayImage();
                        %figure(1), subplot(2,3,4), imshow(imgB_Z,[0 8]);
                        %figure(1), subplot(2,3,5), imshow(warpA2_Z,[0 8]);
                        %figure(1), subplot(2,3,6), imshow(abs(warpA2_Z - imgB_Z),[ ]);
                        figure(1), subplot(2,3,4), imshow(imgB_I);
                        figure(1), subplot(2,3,5), imshow(warpA2_I);
                        figure(1), subplot(2,3,6), imshow(abs(warpA2_I - imgB_I),[ ]);
                    end
                    breakhere = 1;
                end
                a_k
                T = rgbd_odom.parametersToTransform(a_k);
                %{
                if size(prevT,1) ~= 0
                    updateKeyframe(obj, referanceFrames)
                end
                prevT = T;
                %}
                
                slamSystem.trackFrame(curFrame);
                imgB_Z = rgbdImg2.getDepthImage();
                imgB_I = rgbdImg2.getGrayImage();
                warpA2 = rgbdImg1.warpImage(T);
                warpA2_Z = warpA2.getDepthImage();
                warpA2_I = warpA2.getGrayImage();
                %figure(2), subplot(2,3,4), imshow(imgB_Z,[0 8]);
                %figure(2), subplot(2,3,5), imshow(warpA2_Z,[0 8]);
                %figure(2), subplot(2,3,6), imshow(abs(warpA2_Z - imgB_Z),[ ]);
                figure(2), subplot(2,3,4), imshow(imgB_I,[0 255]);
                figure(2), subplot(2,3,5), imshow(warpA2_I,[0 255]);
                figure(2), subplot(2,3,6), imshow(abs(warpA2_I - imgB_I),[ ]);
                breakhere = 1;
                pause(5);
                %}
            end % loop over image pairs
        end
        
     
        
       
    end
end