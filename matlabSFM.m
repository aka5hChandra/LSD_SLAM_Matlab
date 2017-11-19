classdef matlabSFM < handle
    methods (Static)
        function demo()
            imageDir = fullfile(toolboxdir('vision'), 'visiondata','upToScaleReconstructionImages');
            images = imageDatastore(imageDir);
            I1 = readimage(images, 1);
            I2 = readimage(images, 2);
            
            mSFM = matlabSFM();
               % Load precomputed camera parameters
            load upToScaleReconstructionCameraParameters.mat
           [orient, loc] = mSFM.process(I1 , I2, cameraParams )
           
        end
        
        function obj = matlabSFM()
        end
        
        function [orient, loc] = process( I1 , I2 ,cameraParams )
            debug = false;
           
            if debug
                figure
                imshowpair(I1, I2, 'montage');
                title('Original Images');
            end
            
         
            
            %{
            cp = cameraParameters('IntrinsicMatrix',cameraParams.IntrinsicMatrix,'RadialDistortion',cameraParams.RadialDistortion,'ImageSize',cameraParams.ImageSize);  cameraParams = cp;
            cameraParameters =  cp;
            cp(IntrinsicMatrix , cameraParams.IntrinsicMatrix)
            cp.FocalLength = cameraParams.FocalLength
            cp.PrincipalPoint = cameraParams.PrincipalPoint
            cp.Skew = cameraParams.Skew
            cp.RadialDistortion = cameraParams.RadialDistortion
            cp.TangentialDistortion = cameraParams.TangentialDistortion
            cp.ImageSize = cameraParams.ImageSize
            cameraParams = cp
            %}
            I1 = undistortImage(I1, cameraParams);
            I2 = undistortImage(I2, cameraParams);
            if debug
                figure
                imshowpair(I1, I2, 'montage');
                title('Undistorted Images');
            end
            % Detect feature points
            imagePoints1 = detectMinEigenFeatures(rgb2gray(I1), 'MinQuality', 0.1);
            if debug
                % Visualize detected points
                figure
                imshow(I1, 'InitialMagnification', 50);
                title('150 Strongest Corners from the First Image');
                hold on
                plot(selectStrongest(imagePoints1, 150));
            end
            % Create the point tracker
            tracker = vision.PointTracker('MaxBidirectionalError', 1, 'NumPyramidLevels', 5);
            
            % Initialize the point tracker
            imagePoints1 = imagePoints1.Location;
            initialize(tracker, imagePoints1, I1);
            
            % Track the points
            [imagePoints2, validIdx] = step(tracker, I2);
            matchedPoints1 = imagePoints1(validIdx, :);
            matchedPoints2 = imagePoints2(validIdx, :);
            
            if debug
                % Visualize correspondences
                figure
                showMatchedFeatures(I1, I2, matchedPoints1, matchedPoints2);
                title('Tracked Features');
            end
            % Estimate the fundamental matrix
            [E, epipolarInliers] = estimateEssentialMatrix(...
                matchedPoints1, matchedPoints2, cameraParams, 'Confidence', 99.99);
            
            % Find epipolar inliers
            inlierPoints1 = matchedPoints1(epipolarInliers, :);
            inlierPoints2 = matchedPoints2(epipolarInliers, :);
            
            if debug
                % Display inlier matches
                figure
                showMatchedFeatures(I1, I2, inlierPoints1, inlierPoints2);
                title('Epipolar Inliers');
            end
            [orient, loc] = relativeCameraPose(E, cameraParams, inlierPoints1, inlierPoints2);
        end
    end
end