classdef SlamSystem < handle
    properties
        trackingReference;
        map;
        K;
        width, height;
        currentKeyFrame;
    end
    methods 
       function obj = SlamSystem(K, w , h)
            obj.K = K;
            obj.width = w;
            obj.height = h;
            obj.map = DepthMap(K,w,h);
            obj.currentKeyFrame = [];
            obj.trackingReference = TrackingReferance();
        end
         
        function updateKeyframe(obj,referance)
            %%%%loads of code goes here 
            obj.map.updateKeyframe(referance);
        end
        
        function randomInit(obj, frame , id)
            obj.currentKeyFrame = frame;
            obj.map.initializeRandomly(obj.currentKeyFrame);
            %obj.map.initializeFromGTDepth(obj.currentKeyFrame);
            
            
        end
        
        function depthInit(obj, frame , id)
            obj.currentKeyFrame = frame;
            obj.map.initializeFromGTDepth(obj.currentKeyFrame);
           
            
        end
        
        function trackFrame(obj,trackingNewFrame)
            
            %{
            %%check if currenframe and keyframe are same and update
            if size(obj.trackingReference.keyFrame ,1) == 0 || obj.trackingReference.keyFrame ~= obj.currentKeyFrame
                obj.trackingReference.importFrame(obj.currentKeyFrame)
            end
            %work on keyframes instead of every frame
            
             %%only se3 for now , need to extend to sim3
            trackingReferancePose = obj.trackingReference.keyFrame.pose;
            
            obj.updateKeyframe(trackingNewFrame);
            
            obj.map.observeDepthRow( 3 , obj.height - 3)
            
            %}
            
            thisToOther = obj.rgbdODAMTrack( trackingNewFrame);
            
            %{
            thisToOther = [0.9999    0.0053    0.0101   -0.0052;
                -0.0055    0.9998    0.0173    0.0139;
                    -0.0100   -0.0174    0.9998   -0.0044;
                0         0         0    1.0000;]
            %}
            
            mSFM = matlabSFM();
            cameraParams = cameraParameters('IntrinsicMatrix',obj.K','RadialDistortion',[0,0,0],'ImageSize',[640,480]); 
            [orient, loc] =  mSFM.process( obj.currentKeyFrame.imgRGB ,  trackingNewFrame.imgRGB , cameraParams );
           
            %thisToOther = [orient, loc';0,0,0,1];
            
            %obj.map.updateKeyframe(obj.currentKeyFrame);
            trackingNewFrame.prepareForStereoWith( obj.currentKeyFrame , thisToOther, obj.K, 1);
            [y , x] = meshgrid(3:obj.height-3,3 :obj.width - 3);
            [flag,pepx,pepy, x, y] = obj.map.makeAndCheckEPLMat( x , y ,trackingNewFrame);
            %trackingNewFrame.P3_to_P2 * T
            
            
            
            obj.currentKeyFrame = trackingNewFrame;
            
        end
        
        function T = rgbdODAMTrack(obj,  trackingNewFrame)
                %rgbdImg1 = RGBDImage(obj.currentKeyFrame.imgZ,obj.currentKeyFrame.imgRGB,obj.currentKeyFrame.tempK);
                %rgbdImg2 = RGBDImage(trackingNewFrame.imgZ, trackingNewFrame.imgRGB,trackingNewFrame.tempK);
                verbose = true;
                rgbdImg1 = trackingNewFrame;
                rgbdImg2 = obj.currentKeyFrame;
                a_k = [0 0 0 0 0 0];
                T = rgbd_odom.parametersToTransform(a_k);
                if (false==true)
                     imgB_Z = rgbdImg2.getDepthImage();
                imgB_I = rgbdImg2.getGrayImage();
                warpA = rgbdImg1.warpImage(T);
                warpA_Z = warpA.getDepthImage();
                warpA_I = warpA.getGrayImage();
               figure(2), subplot(2,3,1), imshow(imgB_I,[0 255]);
                figure(2), subplot(2,3,2), imshow(warpA_I, [0 255]);
                figure(2), subplot(2,3,3), imshow(abs(warpA_I - imgB_I),[ ]);
                end
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
                        %figure(1), subplot(2,3,2), imshow(warpA_Z,[0 8]);
                        %figure(1), subplot(2,3,3), imshow(abs(warpA_Z - imgB_Z),[ ]);
                        figure(1), subplot(2,3,1), imshow(imgB_I);
                        figure(1), subplot(2,3,2), imshow(warpA_I);
                        figure(1), subplot(2,3,3), imshow(abs(warpA_I - imgB_I),[ ]);
                    end
                     MATLAB_minimize = false;
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
                    a_k = a_k;
                    
                    if (false ==true)
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
                if (verbose==true)
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
                end
        end
    end
end