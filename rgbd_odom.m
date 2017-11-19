classdef rgbd_odom < handle
    
    properties
    end
    
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
            
            cam_info.D = reshape(cam_info_Msgs{1}.D,1,5);
            cam_info.K = reshape(cam_info_Msgs{1}.K,3,3)';
            cam_info.P = reshape(cam_info_Msgs{1}.P,4,3)';
            for imageIdx=1:numImages-1
                stampA = depth_Msgs{imageIdx}.Header.Stamp.Sec+depth_Msgs{imageIdx}.Header.Stamp.Nsec*10^-9;
                fprintf(1,'Image A TimeStamp %15f\n', stampA);
                imageA = reshape(typecast(depth_Msgs{imageIdx}.Data,'single'),640,480)';
                imageA1 = reshape(typecast(rgb_Msgs{imageIdx}.Data,'uint8'),[3,640*480]);
                imageA2 = cat(3,reshape(imageA1(1,:),640,480)',reshape(imageA1(2,:),640,480)',reshape(imageA1(3,:),640,480)');
                rgbdImg1 = RGBDImage(imageA,imageA2,cam_info.K);
                
                stampB = depth_Msgs{imageIdx+1}.Header.Stamp.Sec+depth_Msgs{imageIdx+1}.Header.Stamp.Nsec*10^-9;
                fprintf(1,'Image B TimeStamp %15f\n', stampB);
                imageB = reshape(typecast(depth_Msgs{imageIdx+1}.Data,'single'),640,480)';
                imageB1 = reshape(typecast(rgb_Msgs{imageIdx+1}.Data,'uint8'),[3,640*480]);
                imageB2 = cat(3,reshape(imageB1(1,:),640,480)',reshape(imageB1(2,:),640,480)',reshape(imageB1(3,:),640,480)');
                rgbdImg2 = RGBDImage(imageB,imageB2,cam_info.K);
                                
                [rows,cols]=size(imageA);
                a_k = [0 0 0 0 0 0];
                T = rgbd_odom.parametersToTransform(a_k);

                %rgbd_odom.checkGradient([120,120],rgbdImg1, rgbdImg2);
                
                imgB_Z = rgbdImg2.getDepthImage();
                imgB_I = rgbdImg2.getGrayImage();
                warpA = rgbdImg1.warpImage(T);
                warpA_Z = warpA.getDepthImage();
                warpA_I = warpA.getGrayImage();
                figure(2), subplot(2,3,1), imshow(imgB_Z,[0 8]);
                figure(2), subplot(2,3,2), imshow(warpA_Z,[0 8]);
                figure(2), subplot(2,3,3), imshow(abs(warpA_Z - imgB_Z),[ ]);
                %figure(2), subplot(2,3,1), imshow(imgB_I,[0 255]);
                %figure(2), subplot(2,3,2), imshow(warpA_I, [0 255]);
                %figure(2), subplot(2,3,3), imshow(abs(warpA_I - imgB_I),[ ]);
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
                imgB_Z = rgbdImg2.getDepthImage();
                imgB_I = rgbdImg2.getGrayImage();
                warpA2 = rgbdImg1.warpImage(T);
                warpA2_Z = warpA2.getDepthImage();
                warpA2_I = warpA2.getGrayImage();
                figure(2), subplot(2,3,4), imshow(imgB_Z,[0 8]);
                figure(2), subplot(2,3,5), imshow(warpA2_Z,[0 8]);
                figure(2), subplot(2,3,6), imshow(abs(warpA2_Z - imgB_Z),[ ]);
                %figure(2), subplot(2,3,4), imshow(imgB_I,[0 255]);
                %figure(2), subplot(2,3,5), imshow(warpA2_I,[0 255]);
                %figure(2), subplot(2,3,6), imshow(abs(warpA2_I - imgB_I),[ ]);
                breakhere = 1;
                pause(5);
            end % loop over image pairs
        end
                
        function a_k = estimateOdometry(imageA, imageB, a_k)
            imgZ_a = imageA.getDepthImage();
            [rows, cols] = size(imgZ_a);
            numPts = rows*cols;
            imgRGB_a = imageA.getGrayImage()/255.0;
            %imgRGB_a = reshape(imgRGB_a,numPts,3);
            imgI_a = reshape(imgRGB_a,1,numPts);
            
            imgZ_b = double(imageB.getDepthImage());
            [gradZ_x, gradZ_y] = gradient(imgZ_b);
            imgRGB_b = imageB.getGrayImage()/255.0;
            [gradI_x, gradI_y] = gradient(imgRGB_b);
            imgZ_b = reshape(imgZ_b,1,numPts);
            %imgRGB_b = reshape(imgRGB_b,numPts,3);
            imgI_b = reshape(imgRGB_b,1,numPts);
            
            pts = imageA.getPointCloud();
            T = rgbd_odom.parametersToTransform(a_k);
            prev_sqError = Inf;
            MAXITER=20;
            for iter=1:MAXITER
                %warpA = imageA.warpImage(T);
                %warpA_Z = warpA.getDepthImage();
                %imgB_Z = imageB.getDepthImage();
                %figure(1), subplot(1,3,1), imshow(imgB_Z,[0 8]);
                %figure(1), subplot(1,3,2), imshow(warpA_Z,[0 8]);
                %figure(1), subplot(1,3,3), imshow(abs(warpA_Z - imgB_Z),[ ]);
                %error = abs(warpA_Z - imgB_Z);
                %warpA_Z = reshape(warpA_Z,1,numPts);
                %error = reshape(error,1,numPts);
                %error_mask=zeros(1,numPts);
                %error_mask(mask==1)=1;
                %error_mask(isnan(warpA_Z))=1;
                %error(error_mask==1)=[];
                %errorEst = sum(error.*error)
                %numValid = length(error)
                
                acc_hessian = zeros(6,6);
                depth_grad = zeros(6,1);
                intensity_grad = zeros(6,1);
                numEqs = 0;
                sqError = 0;
                for ptIdx=1:numPts
                    if (~isnan(pts(3,ptIdx)))
                        pt = T*pts(1:4,ptIdx);
                        intensity = imgI_a(ptIdx);
                        pt2d = imageB.project(pt);
                        if (pt2d(1)>= 1 && pt2d(1) < cols && ...
                                pt2d(2)>= 1 && pt2d(2) < rows)
                            x0y0 = floor(pt2d(1:2));
                            x1y1 = floor(pt2d(1:2)+1);
                            x0y1 = [x0y0(1); x1y1(2)];
                            x1y0 = [x1y1(1); x0y0(2)];
                            corners = [x0y0 x1y0 x0y1 x1y1];
                            alpha_xy = [pt2d(1)-x0y0(1); pt2d(2)-x0y0(2)];                            
                            idxs = sub2ind(size(imgZ_a), corners(2,:), corners(1,:));
                            fc = [imgZ_b(idxs)' gradZ_x(idxs)' gradZ_y(idxs)' ...
                                imgI_b(idxs)' gradI_x(idxs)' gradI_y(idxs)'];
                            fc_interp = (fc(1,:)*(1-alpha_xy(1)) + fc(2,:)*alpha_xy(1)) * (1-alpha_xy(2)) + ...
                                (fc(3,:)*(1-alpha_xy(1)) + fc(4,:)*alpha_xy(1)) * alpha_xy(2);
                            if (~isnan(sum(fc_interp)))
                                %ipt3 = imageB.getPoint3d(pt2d, fc_interp(1));
                                %[Jx, Jy, Jz] = imageB.getJacobian(ipt3);
                                [Jx, Jy, Jz] = imageB.getJacobian(pt);
                                residual_Z = fc_interp(1) - pt(3);
                                residual_I = fc_interp(4) - intensity;
                                gradZ_interp = fc_interp(2:3)*[Jx; Jy] - Jz;
                                gradI_interp = fc_interp(5:6)*[Jx; Jy];
                                depth_grad = depth_grad + residual_Z*gradZ_interp';
                                intensity_grad = intensity_grad + residual_I*gradI_interp';
                                acc_hessian = acc_hessian + gradZ_interp'*gradZ_interp;
                                acc_hessian = acc_hessian + gradI_interp'*gradI_interp;
                                sqError = sqError + residual_Z*residual_Z;
                                sqError = sqError + residual_I*residual_I;
                                numEqs = numEqs + 1;
                            end
                        else
                            %disp('error');
                        end
                    end
                end
                
                %numEqs
                sqError
                %sqError/numEqs
                if (sqError > prev_sqError || numEqs < 6)
                    break;
                end
                prev_sqError = sqError;
                acc_hessian = acc_hessian;% + numEqs*numEqs*(0.5)*eye(6);
                %acc_hessian = acc_hessian - 0*numEqs*numEqs*(0.1^iter)*eye(6);
                delta_a_k = inv(acc_hessian)*depth_grad;
                %delta_a_k
                
                delta_T = rgbd_odom.parametersToTransform(delta_a_k');
                % store previous R,t in case error increases
                prev_T = T;
                T=inv(delta_T)*T;
            end
            iter
            % if error went up get minimum as previous R and t
            if (iter < MAXITER)
                T = prev_T;
            end
            a_k = rgbd_odom.transformToParameters(T);
        end
        
        function checkGradient(pt, rgbdImg1, rgbdImg2)
           a_k = [0 0 0 0 0 0];
           imgZ_a = rgbdImg2.getDepthImage();
           imgI_a = rgbdImg2.getGrayImage()/255.0;
           imgZ_b = rgbdImg1.getDepthImage();
           imgI_b = rgbdImg1.getGrayImage()/255.0;
           
           [gradZ_x, gradZ_y] = gradient(imgZ_b);
           [gradI_x, gradI_y] = gradient(imgI_b);
           pt3d = rgbdImg1.getPoint3d(pt, imgZ_b(pt(2), pt(1)));
           [Jx, Jy, Jz] = rgbdImg1.getJacobian(pt3d);
           gradZ = [gradZ_x(pt(2),pt(1)) gradZ_y(pt(2),pt(1))];
           gradZ_Error = (gradZ*[Jx; Jy] - Jz)';
           gradI = [gradI_x(pt(2),pt(1)) gradI_y(pt(2),pt(1))];
           gradI_Error = (gradI*[Jx; Jy])';
           resZ = imgZ_b(pt(2),pt(1)) - imgZ_a(pt(2),pt(1));
           %resZ*(gradZ_Error)
           resI = imgI_b(pt(2),pt(1)) - imgI_a(pt(2),pt(1));
           %resI*(gradI_Error)
           resZ*(gradZ_Error)+resI*(gradI_Error)
           
           options.FiniteDifferenceStepSize = 1e-9*ones(1,6);
           imageErrorAtPoint = @(a_k0) rgbd_odom.errorFuncAtPoint(pt, rgbdImg1, rgbdImg2, a_k0);
           Jval = 0.5*rgbd_odom.numericalGradient(imageErrorAtPoint, a_k, options.FiniteDifferenceStepSize);
           Jval
           breakhere = 1;
        end

        function sqErrors = errorFuncAtPoint(cpt, imageA, imageB, a_kVals)            
            numVals = size(a_kVals,1);
            sqErrors = zeros(numVals,1);
            imgZ_a = imageA.getDepthImage();
            [rows, cols] = size(imgZ_a);
            numPts = rows*cols;
            imgZ_a = reshape(imgZ_a,1,numPts);
            imgRGB_a = imageA.getGrayImage()/255.0;
            %imgRGB_a = reshape(imgRGB_a,numPts,3);
            imgI_a = reshape(imgRGB_a,1,numPts);
            
            imgZ_b = double(imageB.getDepthImage());
            imgZ_b = reshape(imgZ_b,1,numPts);
            imgRGB_b = imageB.getGrayImage()/255.0;
            %imgRGB_b = reshape(imgRGB_b,numPts,3);
            imgI_b = reshape(imgRGB_b,1,numPts);
            
            pts = imageB.getPointCloud();
            
            for a_k_idx = 1:numVals
                a_k = a_kVals(a_k_idx,:);
                T = rgbd_odom.parametersToTransform(a_k);
                ptIdx = sub2ind([rows, cols],cpt(2), cpt(1));
                if (~isnan(pts(3,ptIdx)))
                    pt = T*pts(1:4,ptIdx);
                    intensity = imgI_b(ptIdx);
                    pt2d = imageA.project(pt);
                    if (pt2d(1)>= 1 && pt2d(1) < cols && ...
                            pt2d(2)>= 1 && pt2d(2) < rows)
                        x0y0 = floor(pt2d(1:2));
                        x1y1 = floor(pt2d(1:2)+1);
                        x0y1 = [x0y0(1); x1y1(2)];
                        x1y0 = [x1y1(1); x0y0(2)];
                        corners = [x0y0 x1y0 x0y1 x1y1];
                        alpha_xy = [pt2d(1)-x0y0(1); pt2d(2)-x0y0(2)];
                        idxs = sub2ind([rows, cols], corners(2,:), corners(1,:));
                        fc = [imgZ_a(idxs)' imgI_a(idxs)'];
                        fc_interp = (fc(1,:)*(1-alpha_xy(1)) + fc(2,:)*alpha_xy(1)) * (1-alpha_xy(2)) + ...
                            (fc(3,:)*(1-alpha_xy(1)) + fc(4,:)*alpha_xy(1)) * alpha_xy(2);
                        if (~isnan(sum(fc_interp)))
                            residual_Z = pt(3) - fc_interp(1);
                            residual_I = intensity - fc_interp(2);
                            sqErrors(a_k_idx) = sqErrors(a_k_idx) + residual_Z*residual_Z;
                            sqErrors(a_k_idx) = sqErrors(a_k_idx) + residual_I*residual_I;
                        end
                    end
                end
            end
        end
        
        function sqErrors = errorFunc(imageA, imageB, a_kVals)
            numVals = size(a_kVals,1);
            sqErrors = zeros(numVals,1);
            imgZ_a = double(imageA.getDepthImage());
            [rows, cols] = size(imgZ_a);
            numPts = rows*cols;
            imgRGB_a = double(rgb2gray(imageA.getRGBImage()))/255.0;
            %imgRGB_a = reshape(imgRGB_a,numPts,3);
            imgI_a = reshape(imgRGB_a,1,numPts);
            
            imgZ_b = double(imageB.getDepthImage());
            [xgrad, ygrad] = gradient(imgZ_b);
            gradXVals = reshape(xgrad, 1, numPts);
            gradYVals = reshape(ygrad, 1, numPts);           
            imgZ_b = reshape(imgZ_b,1,numPts);
            imgRGB_b = double(rgb2gray(imageB.getRGBImage()))/255.0;
            %imgRGB_b = reshape(imgRGB_b,numPts,3);
            imgI_b = reshape(imgRGB_b,1,numPts);

            pts = double(imageB.getPointCloud());
            
            for a_k_idx = 1:numVals
                a_k = a_kVals(a_k_idx,:);
                T = rgbd_odom.parametersToTransform(a_k);
                for ptIdx=1:numPts
                    if (~isnan(pts(3,ptIdx)))
                        pt = T*pts(1:4,ptIdx);
                        intensity = imgI_b(ptIdx);
                        pt2d = imageA.project(pt);
                        if (pt2d(1)>= 1 && pt2d(1) < cols && ...
                                pt2d(2)>= 1 && pt2d(2) < rows)
                            x0y0 = floor(pt2d(1:2));
                            x1y1 = floor(pt2d(1:2)+1);
                            x0y1 = [x0y0(1); x1y1(2)];
                            x1y0 = [x1y1(1); x0y0(2)];
                            corners = [x0y0 x1y0 x0y1 x1y1];
                            alpha_xy = [pt2d(1)-x0y0(1); pt2d(2)-x0y0(2)];                            
                            idxs = sub2ind([rows, cols], corners(2,:), corners(1,:));
                            fc = [imgZ_a(idxs)' imgI_a(idxs)'];
                            fc_interp = (fc(1,:)*(1-alpha_xy(1)) + fc(2,:)*alpha_xy(1)) * (1-alpha_xy(2)) + ...
                                (fc(3,:)*(1-alpha_xy(1)) + fc(4,:)*alpha_xy(1)) * alpha_xy(2);
                            if (~isnan(sum(fc_interp)))
                                residual_Z = pt(3) - fc_interp(1);
                                residual_I = intensity - fc_interp(2);
                                sqErrors(a_k_idx) = sqErrors(a_k_idx) + residual_Z*residual_Z;
                                sqErrors(a_k_idx) = sqErrors(a_k_idx) + residual_I*residual_I;
                            end
                        end
                    end
                end
            end
        end
        
        function T = parametersToTransform(a_k)            
            theta_k = sqrt(a_k(4:6)*a_k(4:6)');
            if (theta_k > 1e-10)
                n_k = a_k(4:6)/theta_k;
                n_cross_k = [0 -n_k(3) n_k(2); n_k(3) 0 -n_k(1); -n_k(2) n_k(1) 0];
                R = cos(theta_k)*eye(3) + (1-cos(theta_k))*(n_k'*n_k) + sin(theta_k)*n_cross_k;
                V = eye(3) + ((1-cos(theta_k))/theta_k)*n_cross_k + (1 - sin(theta_k)/theta_k)*(n_k'*n_k);
                t = V*a_k(1:3)';
            else
                R = eye(3);
                t = a_k(1:3)';
            end
            
            T=[R t; 0 0 0 1];
        end
        
        function a_k = transformToParameters(T)
            R = T(1:3,1:3);
            t = T(1:3,4);
            theta = acos(0.5*(trace(R)-1));
            
            if (theta > 1e-10)
                a_k(4:6) = theta*(1/(2*sin(theta)))*[R(3,2)-R(2,3), R(1,3)-R(3,1), R(2,1)-R(1,2)];
                w = a_k(4:6);
                w_cross = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];
                V_inv = eye(3) - 0.5*w_cross + ((1 - theta*sin(theta)/(2*(1-cos(theta))))/theta^2)*w'*w;
                a_k(1:3) = (V_inv*t)';
            else
                a_k(4:6) = [0 0 0];
                a_k(1:3) = t;
            end
            
        end 
        
        function Jval = numericalGradient(func, x, stepsizes)
            % Jacobian functor
            J = @(x,h,func)(func(repmat(x,size(x'))+diag(h))-func(repmat(x,size(x'))-diag(h)))./(2*h');
            % Step to take on each dimension (has to be small enough for precision)
            h = stepsizes.*ones(size(x));
            % Compute the jacobian
            Jval = J(x,h,func);
        end
        
        function symbolicExp
            
        end
    end
        
    methods
        function RGBDOdom(obj)
        end
    end
end