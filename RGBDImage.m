classdef RGBDImage < handle
    properties (SetAccess = private)
        % RGBD camera parameters
        width                     % width of the image
        height                    % height of the image
        numPixels                 % number of pixels in the image
        region_of_interest        % top-left and bottom-right corners define
                                  % a rectangular subset of the image for processing
        MAXRANGE                  % maximum range (in m.) of the sensor
        % The distortion parameters, size depending on the distortion model.
        % For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).
        D                         % vector of distortion coefficients
        % Intrinsic camera matrix for the raw (distorted) images.
        %     [fx  0 cx]
        % K = [ 0 fy cy]
        %     [ 0  0  1]
        K
        % Rectification matrix (stereo cameras only)
        % A rotation matrix aligning the camera coordinate system to the ideal
        % stereo image plane so that epipolar lines in both stereo images are
        % parallel.
        R
        % Projection/camera matrix
        %     [fx'  0  cx' Tx]
        % P = [ 0  fy' cy' Ty]
        %     [ 0   0   1   0]
        % By convention, this matrix specifies the intrinsic (camera) matrix
        %  of the processed (rectified) image. That is, the left 3x3 portion
        %  is the normal camera intrinsic matrix for the rectified image.
        % It projects 3D points in the camera coordinate frame to 2D pixel
        %  coordinates using the focal lengths (fx', fy') and principal point
        %  (cx', cy') - these may differ from the values in K.
        % For monocular cameras, Tx = Ty = 0. Normally, monocular cameras will
        %  also have R = the identity and P[1:3,1:3] = K.
        P
        f                          % focal length (in pixels) in (x,y)
        c                          % principal/center point of image
        P2_to_P3
        P3_to_P2
        % for XYZ point cloud fitting and depends on measured data
        imgZ                         % sensed depth image        
        imgRGB                       % sensed color image        
    end
    
    methods (Static)
        function demo()
        end
    end % Static methods
    
    methods
        function obj = RGBDImage( imgZ, imgRGB, cam_info_K)
            obj.imgRGB = imgRGB;
            obj.imgZ = double(imgZ);
            [obj.height, obj.width] = size(imgZ);
            obj.f.x = cam_info_K(1,1);
            obj.f.y = cam_info_K(2,2);
            obj.c.x = cam_info_K(1,3)+1; % matlab indexing
            obj.c.y = cam_info_K(2,3)+1; % matlab indexing
            obj.numPixels = obj.width*obj.height;
            obj.P2_to_P3=[1/obj.f.x 0 0 -obj.c.x/obj.f.x;
                0 1/obj.f.y 0 -obj.c.y/obj.f.y;
                0 0 1 0;
                0 0 0 1];
            obj.P3_to_P2 = inv(obj.P2_to_P3);
        end
        
        function rgbdImg = downsample(obj, factor)
            imgZ = obj.imgZ(1:factor:obj.height,1:factor:obj.width);
            imgRGB = obj.imgRGB(1:factor:obj.height,1:factor:obj.width,:);
            f.x = obj.f.x / factor;
            f.y = obj.f.y / factor;
            c.x = obj.c.x / factor;
            c.y = obj.c.y / factor;
            cam_info = [f.x 0 c.x; 0 f.y c.y; 0 0 1];
            rgbdImg = RGBDImage( imgZ, imgRGB, cam_info);
        end
        
        function imgZ = getDepthImage(obj)
            imgZ = obj.imgZ;
        end
        
        function imgRGB = getRGBImage(obj)
            imgRGB = obj.imgRGB;
        end
        
        function imgRGB = getGrayImage(obj)
            imgRGB = double(rgb2gray(obj.imgRGB));
        end
        
        function p3d = getPoint3d(obj, pt, z)
            p3d = [pt(1); pt(2); z; 1.0];
            p3d = obj.P2_to_P3*p3d;
            p3d(1:2,:) = p3d(1:2,:).*p3d(3,:);            
        end
        
        function p3d = getPointCloud(obj)
            [xi, yi]=meshgrid( 1:obj.width, 1:obj.height);
            p3d = [reshape( xi, 1, obj.numPixels);
                reshape( yi, 1, obj.numPixels);
                reshape( obj.imgZ, 1, obj.numPixels);
                ones( 1, obj.numPixels)];
            p3d = obj.P2_to_P3*p3d;
            p3d(1:2,:) = p3d(1:2,:).*p3d(3,:);            
        end
        
        function pts2d = project(obj, pts3d)
            pts3d(1:2,:) = pts3d(1:2,:)./pts3d(3,:);
            pts2d = obj.P3_to_P2*pts3d;
        end
        
        function [Jx_w, Jy_w, Jz_w] = getJacobian(obj, pt)
            Jx_w = obj.f.x/pt(3) * [1.0 ...
                0.0 ...
                -pt(1)/pt(3) ...
                -pt(1)*pt(2)/pt(3) ...
                pt(3) + (pt(1)*pt(1)/pt(3)) ...
                -pt(2)];
            Jy_w = obj.f.y./pt(3) * [0.0 ...
                1.0 ...
                -pt(2)/pt(3) ...
                -pt(3) + (pt(2)*pt(2)/pt(3)) ...
                pt(1)*pt(2)/pt(3) ...
                pt(1)];
            Jz_w = [0.0 0.0 1.0 pt(2) -pt(1) 0.0];
        end
        
        function [Jx_w, Jy_w, Jz_w] = getJacobians(obj)
            pts = obj.getPointCloud();
            ONEVEC = ones(obj.numPixels,1);
            ZEROVEC = zeros(obj.numPixels,1);
            Jx_w = (obj.f.x./pts(3,:))' .* [ONEVEC ...
                ZEROVEC ...
                -(pts(1,:)./pts(3,:))' ...
                -(pts(1,:).*pts(2,:)./pts(3,:))' ...
                (pts(3,:) + (pts(1,:).*pts(1,:)./pts(3,:)))' ...
                -pts(2,:)'];
            Jy_w = (obj.f.y./pts(3,:))' .* [ZEROVEC ...
                ONEVEC ...
                -(pts(2,:)./pts(3,:))' ...
                -(pts(3,:) + (pts(2,:).*pts(2,:)./pts(3,:)))' ...
                (pts(1,:).*pts(2,:)./pts(3,:))' ...
                pts(1,:)'];
            Jz_w = [ZEROVEC ...
                ZEROVEC ...
                ONEVEC ...
                pts(2,:)' ...
                -pts(1,:)' ...
                ZEROVEC];
        end        
        
        function [gradImages, mask, Jz_w] = computeGradientImages(obj)
            [Jx_w, Jy_w, Jz_w] = obj.getJacobians();
            [xgrad, ygrad] = gradient(obj.imgZ);
            gradXVals = reshape(xgrad, obj.numPixels, 1);
            gradYVals = reshape(ygrad, obj.numPixels, 1);           
            gradImages = gradXVals.*Jx_w + gradYVals.*Jy_w;
            zVals = reshape(obj.imgZ, obj.numPixels, 1);
            badIdxs = isnan(zVals(3,:)) | isnan(gradXVals') | isnan(gradYVals');
            mask = zeros(1, obj.numPixels);
            mask(badIdxs) = 1;
        end
        
        function imgWarp = warpImage(obj, T)
            NEAREST_NEIGHBOR = true;
            BILINEAR_INTERPOLATION = false;
            pts3d = obj.getPointCloud();
            pts3d = T*pts3d;
            p2D_prime = pts3d;
            p2D_prime = obj.P3_to_P2*p2D_prime;
            p2D_prime = obj.project(pts3d);            
            %imgWarpZ = measurement_count( obj.height, obj.width);
            imgWarpZ = NaN*ones( obj.height, obj.width);
            imgWarpI = zeros( obj.height, obj.width, 3, 'uint8');
            for idx=1:obj.numPixels
                if (~isnan(pts3d(3,idx))) % has value in imageA
                            p2D_prime(1:2,idx) = round(p2D_prime(1:2,idx));
                    if (p2D_prime(1,idx) >= 0 && p2D_prime(1,idx) < obj.width ...
                            && p2D_prime(2,idx) >= 0 && p2D_prime(2,idx) < obj.height)
                        if (NEAREST_NEIGHBOR == true)
                            p2D_prime(1:2,idx) = round(p2D_prime(1:2,idx));
                            idxB = sub2ind( [obj.height obj.width], p2D_prime(2,idx)+1, p2D_prime(1,idx)+1);
                            imgWarpZ(idxB) = pts3d(3,idx);
                            imgWarpI(p2D_prime(2,idx)+1, p2D_prime(1,idx)+1,:) = obj.imgRGB(p2D_prime(2,idx)+1, p2D_prime(1,idx)+1,:);
                        elseif (BILINEAR_INTERPOLATION == true)
                             aaa=1;
                        end
                    end
                end
            end
            cam_info = [obj.f.x 0 obj.c.x; 0 obj.f.y obj.c.y; 0 0 1];
            imgWarp = RGBDImage( imgWarpZ, imgWarpI, cam_info); 
        end        
    end
end