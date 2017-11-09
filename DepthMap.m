classdef DepthMap < handle
    properties
        %intresnic camera parameters
        fx,fy,cx,cy,fxi,fyi,cxi,cyi;
        width, height;
        activeFrame;
        newest_referenceFrame;
        oldest_referenceFrame;
        depthHypothesis;
        K;
        priorDepth;
    end
    methods 
        function obj = DepthMap(K,w, h)
            obj.K = K;
            obj.fx = K(1,1);
            obj.fy = K(2,2);
            obj.cx = K(1,3);
            obj.cy = K(2,3);

            KInv = inv(K);
            obj.fxi = KInv(1,1);
            obj.fyi = KInv(2,2);
            obj.cxi = KInv(1,3);
            obj.cyi = KInv(2,3);
            
            obj.width = w;
            obj.height = h;
            
 
            
        end
        
        function initializeRandomly(obj, new_frame)
            obj.activeFrame = new_frame;
            %populate random / rgbd depthmap hypothis 
        end
        
        function initializeFromGTDepth(obj, new_frame)
            obj.activeFrame = new_frame;
            
            intialVar = ones(size(new_frame.imgRGB,1),size(new_frame.imgRGB,2)) * globalParams.VAR_GT_INIT_INITIAL;
            obj.depthHypothesis = DepthMapPixelHypothesis(new_frame.imgZ, new_frame.imgZ, intialVar, intialVar);
            
            %%using this avoid nans
            obj.priorDepth = obj.depthHypothesis.idepth;
            obj.depthHypothesis.visualize(0);
        end
        
        function updateKeyframe(obj, referanceFrames)
            %%%for now previous frame is active frame
            if size(obj.activeFrame,1) == 0
               obj.activeFrame = referanceFrames;
            end
            
            referanceFrames.prepareForStereoWith(obj.activeFrame, inv(obj.activeFrame.getScaledCamToWorld()), obj.K,0);
            obj.newest_referenceFrame = referanceFrames;
            %%obj.activeFrame = referanceFrames;
        end
        %{
        function [flag,pepx,pepy] = makeAndCheckEPL(obj,x , y , ref)
          %%%replace with matrix maniuplation 
            epx = - obj.fx * ref.thisToOther_t(1) + ref.thisToOther_t(3)*(x - obj.cx);
            epy = - obj.fy * ref.thisToOther_t(2) + ref.thisToOther_t(3)*(y - obj.cy);

            if isNAN(epx+epy)
                flag = false;
            end

            %%======== check epl length =========
            eplLengthSquared = epx*epx+epy*epy;
            if eplLengthSquared < globalParams.MIN_EPL_LENGTH_SQUARED 
                flag = false;
            end

            %% ===== check epl-grad magnitude ======
            gx =  obj.activeFrame.image(x + 1,y) - obj.activeFrame.image(x - 1,y) ;
            gy =  obj.activeFrame.image(x, y + 1) - obj.activeFrame.image(x,y - 1) ;
            eplGradSquared = gx * epx + gy * epy;
            eplGradSquared = (eplGradSquared*eplGradSquared) / eplLengthSquared;	

           if eplGradSquared < globalParams.MIN_EPL_GRAD_SQUARED
                flag= false;
           end


            %% ===== check epl-grad angle ======
            if (eplGradSquared / (gx*gx+gy*gy)) < globalParams.MIN_EPL_ANGLE_SQUARED
                flag = false;
            end
    

            %% ===== DONE - return "normalized" epl =====
            float fac = GRADIENT_SAMPLE_DIST / sqrt(eplLengthSquared);
            pepx = epx * fac;
            pepy = epy * fac;

            flag = true;

            
        end
        %}
         function [flag,pepx,pepy, x, y ] = makeAndCheckEPLMat(obj,x , y , ref)
          %%%replace with matrix maniuplation 
            %x = reshape(x , [] , 1);
            %y = reshape(y , [] , 1);
            
            %%from c++ implementaion between 1 and 2 frame
           %{
            ref.thisToOther_t = [0.0730602 0.110193 -0.0344612];%[-0.00686165 , -0.01699,0.00674572];
            ref.thisToOther_R = [ 0.998751 -0.0408764 -0.0287494 ;  0.04397  0.99218 0.116814  ; 0.0237496 -0.117932  0.992738]';
            
             %ref.thisToOther_t = [0.01416, 0.0125024 , -0.002687];%[-0.00686165 , -0.01699,0.00674572];
             %ref.thisToOther_R = [0.99986 ,-0.000353276, -0.0163908; 0.00790092, 0.999644,0.0266513;0.0163755,-0.0266607,0.9951]';
             ref.K_otherToThis_R  = obj.K * inv( ref.thisToOther_R );
            %}
            epx = (- obj.fx * ref.thisToOther_t(1)) + (ref.thisToOther_t(3)*(x - obj.cx));
            epy = (- obj.fy * ref.thisToOther_t(2)) + (ref.thisToOther_t(3)*(y - obj.cy));
            
             [failedIDs] = (isnan(epx+epy));
             epx(failedIDs) = 0;
             epy(failedIDs) = 0;
             x(failedIDs) = 0;
             y(failedIDs) = 0;
            if size(failedIDs, 1) > 0
                flag = false;
            end
            
          

            %%======== check epl length =========
            eplLengthSquared = (epx.*epx)+(epy.*epy);
            
            [failedIDs] = (eplLengthSquared < globalParams.MIN_EPL_LENGTH_SQUARED );
             epx(failedIDs) = 0;
             epy(failedIDs) = 0;
             x(failedIDs) = 0;
             y(failedIDs) = 0;
            if size(failedIDs, 1) > 0 
                flag = false;
            end

            %% ===== check epl-grad magnitude ======
            activeImageGray = (obj.activeFrame.imgGray);
            filter = [-1 0 1];
            [w,h] = size(activeImageGray);
            gx =  double(imfilter(activeImageGray,filter));
            gy =  double(imfilter(activeImageGray,filter'));
            gx = gx(3:w-3 , 3 : h - 3);
            gy = gy(3:w-3 , 3 : h - 3);
            %{
            gx =  obj.activeFrame.image(x + 1,y) - obj.activeFrame.image(x - 1,y) ;
            gy =  obj.activeFrame.image(x, y + 1) - obj.activeFrame.image(x,y - 1) ;
            %}
            eplGradSquared = (gx .* epx) + (gy .* epy);
            eplGradSquared = (eplGradSquared.*eplGradSquared)./ eplLengthSquared;	
            %{
            [failedX , failedY ] = find( eplGradSquared < globalParams.MIN_EPL_GRAD_SQUARED);
             epx(failedX , failedY) = 0;
             epy(failedX , failedY) = 0;
             
             gx(failedX , failedY)  = 0;
             gy(failedX , failedY)  = 0;
            
            if size(failedX, 1) > 0 
                flag = false;
            end
        %}
          
        [validIDs ] = ( ~(eplGradSquared < globalParams.MIN_EPL_GRAD_SQUARED));
             epx = epx(validIDs);
             epy = epy(validIDs);
             
             gx = gx(validIDs );
             gy = gy(validIDs );
             
             eplGradSquared = eplGradSquared(validIDs);
             eplLengthSquared = eplLengthSquared(validIDs);
             
             x = x(validIDs) ;
             y = y(validIDs) ;
             %{
             epx = reshape(epx , [] ,1);
             epy = reshape(epy , [] ,1);
             gx = reshape(gx , [] ,1);
             gy = reshape(gy , [] ,1);
             
             eplGradSquared = reshape(eplGradSquared , [] ,1);
             eplLengthSquared = reshape(eplLengthSquared , [] ,1);
             %}
             %{
            if size(validIDs, 1) > 0 
                flag = false;
            end
%}
            %% ===== check epl-grad angle ======
             [validIDs  ] = ( ~((eplGradSquared ./ ((gx.*gx)+(gy.*gy))) < globalParams.MIN_EPL_ANGLE_SQUARED));
            %{
             if size(validIDs, 1) > 0 
                flag = false;
            end
            %}
            eplLengthSquared = eplLengthSquared(validIDs );
            epx = epx(validIDs  );
            epy = epy(validIDs  );
             x = x(validIDs) ;
             y = y(validIDs) ;
            %% ===== DONE - return "normalized" epl =====
            fac = globalParams.GRADIENT_SAMPLE_DIST ./ sqrt(eplLengthSquared);
            pepx = epx .* fac;
            pepy = epy .* fac;

            flag = true;
            %{
            min_idepth = 0;
            prior_idepth = 0.5;
            max_idepth = 1;
            %}
            obj.doLineStereo(x,y,pepx,pepy,ref)
         end
        
         function [flag] = doLineStereo(obj,u,v,epxn,epyn,referenceFrame)
             
              
          %{
              referenceFrame.thisToOther_t = [0.0730602 0.110193 -0.0344612];%[-0.00686165 , -0.01699,0.00674572];
             referenceFrame.thisToOther_R = [ 0.998751 -0.0408764 -0.0287494 ;  0.04397  0.99218 0.116814  ; 0.0237496 -0.117932  0.992738]';
             referenceFrame.K_otherToThis_R  = obj.K * inv(referenceFrame.thisToOther_R);
               
            obj.priorDepth(3,3) = 0.605377;
            u(1,1) = 3;
            v(1,1) = 3;
           
             %}
             
                KinvP = ones(size(u,1) , 3);
                KinvP(:,1) = obj.fxi*u+obj.cxi;
                KinvP(:,2) = obj.fyi*v+obj.cyi;
                %%KinvP = [obj.fxi*u+obj.cxi,obj.fyi*v+obj.cyi,1.0];
                
                %pInf = referenceFrame.K_otherToThis_R * KinvP';
                pInf = zeros(size(KinvP));
                pReal = zeros(size(KinvP));
                rescaleFactor = zeros(size(KinvP,1),1);
                %replace by effident mathod 
                for i = 1 : size(u,1)
                    pInf(i,:) = referenceFrame.K_otherToThis_R * KinvP(i,:)';
                    priorDepth = obj.priorDepth(v(i),u(i));
                    if priorDepth ~= 0
                        pReal(i,:) = pInf(i,:) ./ priorDepth + referenceFrame.K_otherToThis_t';
                        rescaleFactor(i) = pReal(i,3) * priorDepth;
                    else
                        pReal(i,:) = -1;
                        rescaleFactor(i) = -1;
                    end
                  
                end
                
               
                
                
                validIDs = int32(find(rescaleFactor ~=-1));
                rescaleFactor = rescaleFactor(validIDs);
                u = u(validIDs);
                v = v(validIDs);
                epxn = epxn(validIDs);
                epyn = epyn(validIDs);
                pInf = pInf(validIDs,:);
                pReal = pReal(validIDs,:);
                KinvP = KinvP(validIDs,:);
                %{
                firstX = u - 2*epxn*rescaleFactor;
                firstY = v - 2*epyn*rescaleFactor;
                lastX = u + 2*epxn*rescaleFactor;
                lastY = v + 2*epyn*rescaleFactor;
                %}
                %firstX , firstY , lastX , lastY
                
                
                
                eplLines = [ u - 2*epxn.*rescaleFactor, v - 2*epyn.*rescaleFactor , u + 2*epxn.*rescaleFactor,v + 2*epyn.*rescaleFactor];
                %%width - 2 and height - 2 comes from the one-sided gradient calculation at the bottom
                validIDs = int32( find(~(eplLines(:,1) <= 0 | eplLines(:,1) >= obj.width - 2 | eplLines(:,2) <= 0 | eplLines(:,2) >= obj.height - 2 | eplLines(:,3) <= 0 | eplLines(:,3) >= obj.width - 2 | eplLines(:,4) <= 0 | eplLines(:,4) >= obj.height - 2)));
                eplLines = eplLines(validIDs,:);
                rescaleFactor = rescaleFactor(validIDs);
                epxn = epxn(validIDs);
                epyn = epyn(validIDs);
                 pInf = pInf(validIDs,:);
                pReal = pReal(validIDs,:);
                 u = u(validIDs);
                v = v(validIDs);
                KinvP = KinvP(validIDs,:);
                %{
                %%not wroking 
                validIDs = int32(find(~(rescaleFactor > 0.7 & rescaleFactor < 1.4)));
                eplLines = eplLines(validIDs,:);
                rescaleFactor = rescaleFactor(validIDs);
                epxn = epxn(validIDs);
                epyn = epyn(validIDs);
                u = u(validIDs);
                v = v(validIDs);
                 pInf = pInf(validIDs,:);
                pReal = pReal(validIDs,:);
                %}
                
                %{
                %%draw line 
                imshow(referenceFrame.imgRGB)
                l = 20;
                line([eplLines(l,1),eplLines(l,2)],[eplLines(l,3),eplLines(l,4)])
                %}
                
                %%interploation 
                
                [x , y] = meshgrid(1:obj.height,1 :obj.width );
                imgForIntepolation = im2double(obj.activeFrame.imgGray);
                
                
                epxnRf = epxn.*rescaleFactor;
                epynRf = epyn.*rescaleFactor;
                realVal_p1 = interp2(x,y,imgForIntepolation,u + epxnRf, v + epynRf, 'cubic');
                realVal_m1 = interp2(x,y,imgForIntepolation,u - epxnRf, v - epynRf, 'cubic');
                realVal = interp2(x,y,imgForIntepolation,u,v,'cubic') ; 
                realVal_m2 = interp2(x,y,imgForIntepolation,u - epxnRf, v - epynRf, 'cubic');
                realVal_p2 = interp2(x,y,imgForIntepolation,u + epxnRf, v + epynRf, 'cubic');
                
                
            
                
                
                 sv = sqrt(obj.depthHypothesis.idepth_var_smoothed);
                min_idepth = obj.depthHypothesis.idepth_smoothed - sv*globalParams.STEREO_EPL_VAR_FAC;
                max_idepth = obj.depthHypothesis.idepth_smoothed + sv*globalParams.STEREO_EPL_VAR_FAC;
                min_idepth(min_idepth < 0) = 0;
                oneByMin_Depth = 1/globalParams.MIN_DEPTH;
                max_idepth(max_idepth > oneByMin_Depth) = oneByMin_Depth;
                
                
                pClose = zeros(size(pInf,1),3);
                pFar = zeros(size(pInf,1),3);
                for i = 1: size(pInf,1)
                    pClose(i,:) = pInf(i,:) + referenceFrame.K_otherToThis_t'*max_idepth(u(i),v(i));
                    
                    if(pClose(i,3) < 0.001)
                        max_idepth(u,v) = (0.001-pInf(i,3)) / referenceFrame.K_otherToThis_t(3);
                        pClose(i,:)  = pInf(i,:) + referenceFrame.K_otherToThis_t'*max_idepth(u(i),v(i));
                    end
                    pClose(i,:)  = pClose(i,:)  ./ pClose(i,3); 
                    
                    pFar(i,:)  = pInf(i,:) + referenceFrame.K_otherToThis_t'*min_idepth(u(i),v(i));
                end
                
                
                validIDs = ~(pFar(:,3) < 0.001 );%%%%% | max_idepth < min_idepth);
                pFar = pFar(validIDs,:);
                pClose = pClose(validIDs,:);
                
                pFar = pFar ./pFar(3);
                
                validIDs = ~(isnan(pFar(:,1)+pClose(:,1)));
                pFar = pFar(validIDs,:);
                pClose = pClose(validIDs,:);
                
                realVal_p1 = realVal_p1(validIDs);
                realVal_m1 = realVal_m1(validIDs);
                realVal = realVal(validIDs) ; 
                realVal_m2 = realVal_m2(validIDs);
                realVal_p2 = realVal_p2 (validIDs);
                KinvP = KinvP(validIDs,:);
                u = u(validIDs);
                v = v(validIDs);
                
                %%calculate increments in which we will step through the epipolar line. they are sampleDist (or half sample dist) long
                incx = pClose(:,1) - pFar(:,1);
                incy = pClose(:,2) - pFar(:,2);
                eplLength = sqrt(incx.*incx+incy.*incy);
                validIDs = ~(eplLength < 0  | eplLength >= Inf);
                pFar = pFar(validIDs,:);
                pClose = pClose(validIDs,:);
                u = u(validIDs);
                v = v(validIDs);
                
                longerEplIds =  eplLength > globalParams.MAX_EPL_LENGTH_CROP;
	
                pClose(longerEplIds,1) = pFar(longerEplIds,1) + incx(longerEplIds).*(globalParams.MAX_EPL_LENGTH_CROP./eplLength(longerEplIds));
                pClose(longerEplIds,2) = pFar(longerEplIds,2) + incy(longerEplIds).*(globalParams.MAX_EPL_LENGTH_CROP./eplLength(longerEplIds));
                
                
                incx = incx .* (globalParams.GRADIENT_SAMPLE_DIST./eplLength);
                incy = incy .* (globalParams.GRADIENT_SAMPLE_DIST./eplLength);
	
                
                
                pFar(:,1) = pFar(:,1) - incx;
                pFar(:,2) =  pFar(:,2) - incy;
                pClose(:,1)  =  pClose(:,1) + incx;
                pClose(:,2) = pClose(:,2) + incy;
                
                %% make epl long enough (pad a little bit).
                shoterEplIds = eplLength < globalParams.MIN_EPL_LENGTH_CROP;
	
                pad = (globalParams.MIN_EPL_LENGTH_CROP - (eplLength(shoterEplIds))) / 2.0;
                pFar(shoterEplIds,1) = pFar(shoterEplIds,1) - incx(shoterEplIds).*pad;
                pFar(shoterEplIds,2) = pFar(shoterEplIds,2) - incy(shoterEplIds).*pad;

                pClose(shoterEplIds,1) =  pClose(shoterEplIds,1) + incx(shoterEplIds).*pad;
                pClose(shoterEplIds,2) = pClose(shoterEplIds,2) +incy(shoterEplIds).*pad;
	
                
                %%If inf point is outside of image: skip pixel.
                eplInsideIDs = ~(pFar(:,1) <= globalParams.SAMPLE_POINT_TO_BORDER | pFar(:,1) >= obj.width-globalParams.SAMPLE_POINT_TO_BORDER | pFar(:,2) <= globalParams.SAMPLE_POINT_TO_BORDER | pFar(:,2) >= obj.height-globalParams.SAMPLE_POINT_TO_BORDER)
                
                pFar = pFar(eplInsideIDs , :);
                pClose = pClose(eplInsideIDs , :);
                incx = incx(eplInsideIDs);
                incy = incy(eplInsideIDs);
                u = u(eplInsideIDs);
                v = v(eplInsideIDs);
                
                realVal_p1 = realVal_p1(eplInsideIDs);
                realVal_m1 = realVal_m1(eplInsideIDs);
                realVal = realVal(eplInsideIDs) ; 
                realVal_m2 = realVal_m2(eplInsideIDs);
                realVal_p2 = realVal_p2 (eplInsideIDs);
                KinvP = KinvP(eplInsideIDs,:);
                
                %% if near point is outside: move inside, and test length again.

			%%eplInsideIDs = ~(pClose(:,1) <= globalParams.SAMPLE_POINT_TO_BORDER | pClose(:,1) >= obj.width-globalParams.SAMPLE_POINT_TO_BORDER | pClose(:,2) <= globalParams.SAMPLE_POINT_TO_BORDER | pClose(:,2) >= obj.height-globalParams.SAMPLE_POINT_TO_BORDER)
	
            eplInsideIDs = pClose(:,1) <= globalParams.SAMPLE_POINT_TO_BORDER;
            toAdd = (globalParams.SAMPLE_POINT_TO_BORDER - pClose(eplInsideIDs,1)) ./ incx(eplInsideIDs);
			pClose(eplInsideIDs,1) =  pClose(eplInsideIDs,1) + (toAdd .* incx(eplInsideIDs));
			pClose(eplInsideIDs,2) =  pClose(eplInsideIDs,2) + (toAdd .* incy(eplInsideIDs));
		
            eplInsideIDs = pClose(:,1) >= obj.width-globalParams.SAMPLE_POINT_TO_BORDER;
			toAdd = (obj.width-globalParams.SAMPLE_POINT_TO_BORDER - pClose(eplInsideIDs,1)) ./ incx(eplInsideIDs);
			pClose(eplInsideIDs,1)  = pClose(eplInsideIDs,1)  + (toAdd .* incx(eplInsideIDs));
			pClose(eplInsideIDs,2)  = pClose(eplInsideIDs,1)  + (toAdd .* incy(eplInsideIDs));
		

            eplInsideIDs =  pClose(:,2) <= globalParams.SAMPLE_POINT_TO_BORDER ;
            toAdd = (globalParams.SAMPLE_POINT_TO_BORDER - pClose(eplInsideIDs,1)) ./ incy(eplInsideIDs);
			pClose(eplInsideIDs,1)  = pClose(eplInsideIDs,1)  + (toAdd .* incx(eplInsideIDs));
			pClose(eplInsideIDs,2)  = pClose(eplInsideIDs,2)  + (toAdd .* incy(eplInsideIDs));
		
            eplInsideIDs = pClose(:,2)  >= obj.height-globalParams.SAMPLE_POINT_TO_BORDER;
			toAdd = (obj.height-globalParams.SAMPLE_POINT_TO_BORDER - pClose(eplInsideIDs,1)) / incy(eplInsideIDs);
			pClose(eplInsideIDs,1)  = pClose(eplInsideIDs,1) + (toAdd .* incx(eplInsideIDs));
			pClose(eplInsideIDs,2)  = pClose(eplInsideIDs,2) + (toAdd .* incy(eplInsideIDs));
		
            
             %get new epl length
              fincx = pClose(:,1) - pFar(:,1);
              fincy = pClose(:,2) - pFar(:,2);
		      newEplLength = sqrt(fincx.*fincx+fincy.*fincy);
              
              eplInsideIDs = ~(pClose(:,1) <= globalParams.SAMPLE_POINT_TO_BORDER | pClose(:,1) >= obj.width-globalParams.SAMPLE_POINT_TO_BORDER | pClose(:,2) <= globalParams.SAMPLE_POINT_TO_BORDER | pClose(:,2) >= obj.height-globalParams.SAMPLE_POINT_TO_BORDER | newEplLength < 8.0);
              pFar = pFar(eplInsideIDs , :);
              pClose = pClose(eplInsideIDs , :);
                incx = incx(eplInsideIDs);
                incy = incy(eplInsideIDs);
               
                realVal_p1 = realVal_p1(eplInsideIDs);
                realVal_m1 = realVal_m1(eplInsideIDs);
                realVal = realVal(eplInsideIDs) ; 
                realVal_m2 = realVal_m2(eplInsideIDs);
                realVal_p2 = realVal_p2 (eplInsideIDs);
                KinvP = KinvP(eplInsideIDs,:);
                
                  u = u(eplInsideIDs);
                v = v(eplInsideIDs);
                %{
                imshow(referenceFrame.imgRGB) 
                hold on;
                l = 1;
                for l = 1: size(pClose , 1)
                line([pClose(l,1),pFar(l,1)],[pClose(l,2),pFar(l,2)])
                %line([pClose(l,1),pClose(l,2)],[pFar(l,1),pFar(l,2)])
                end
              %}
                %{
                if (firstX <= 0 || firstX >= obj.width - 2 || firstY <= 0 || firstY >= obj.height - 2 || lastX <= 0 || lastX >= obj.width - 2 || lastY <= 0 || lastY >= obj.height - 2) 
                    flag = -1;
                    return
                end
                %}
              
              
              % from here on:
              % - pInf: search start-point
              % - p0: search end-point
              % - incx, incy: search steps in pixel
              % - eplLength, min_idepth, max_idepth: determines search-resolution, i.e. the result's variance.
    
              
             imgForIntepolation = im2double(referenceFrame.imgGray); 
              
             val_cp_m2 = interp2(x,y,imgForIntepolation,pFar(:,1)-2.0*incx,pFar(:,2)-2.0*incy, 'cubic');
             val_cp_m1 = interp2(x,y,imgForIntepolation,pFar(:,1)-incx,pFar(:,2)-incy, 'cubic');
             val_cp = interp2(x,y,imgForIntepolation,pFar(:,1),pFar(:,2), 'cubic');
             val_cp_p1 = interp2(x,y,imgForIntepolation,pFar(:,1)+incx,pFar(:,2)+incy, 'cubic');
             val_cp_p2 = zeros(size(val_cp_p1,1),1);
         
              e1A = zeros(size(val_cp_p1,1),1);
              e2A = zeros(size(val_cp_p1,1),1);
              e3A = zeros(size(val_cp_p1,1),1);
              e4A = zeros(size(val_cp_p1,1),1);
              e5A = zeros(size(val_cp_p1,1),1);
              
              e1B = zeros(size(val_cp_p1,1),1);
              e2B = zeros(size(val_cp_p1,1),1);
              e3B = zeros(size(val_cp_p1,1),1);
              e4B = zeros(size(val_cp_p1,1),1);
              e5B = zeros(size(val_cp_p1,1),1);
              
              
              ee = zeros(size(val_cp_p1,1),1);
              loopCounter =  zeros(size(val_cp_p1,1),1);
              eeLast =  ones(size(val_cp_p1,1),1) .* -1;
              loopCBest= zeros(size(val_cp_p1,1),1);
              loopCSecond = zeros(size(val_cp_p1,1),1);
              best_match_x = zeros(size(val_cp_p1,1),1);
             best_match_y = zeros(size(val_cp_p1,1),1);
             best_match_err = ones(size(val_cp_p1,1),1) * 1e50;
             second_best_match_err = ones(size(val_cp_p1,1),1) * 1e50;
             bestWasLastLoop = ones(size(val_cp_p1,1),1) * false;
             best_match_errPost = zeros(size(val_cp_p1,1),1);
             best_match_DiffErrPost = zeros(size(val_cp_p1,1),1);
                    
                    cpx = pFar(:,1);
                    cpy = pFar(:,2);
                    
              iteratroID = (incx < 0) == (cpx > pClose(:,1)) & (incy < 0) == (cpy > pClose(:,2));      
                    loopCounterAll = 0;
             while(any(iteratroID) | all(loopCounter == 0))
	
             val_cp_p2(iteratroID) = interp2(x,y,imgForIntepolation,cpx(iteratroID)+2 * incx(iteratroID),cpy(iteratroID)+ 2 *incy(iteratroID), 'cubic');
             
             if mod(loopCounterAll , 2) ==0
                 e1A(iteratroID)  = val_cp_p2(iteratroID)  - realVal_p2(iteratroID) ;ee(iteratroID)  = ee(iteratroID)  + e1A(iteratroID) .*e1A(iteratroID) ;
                 e2A(iteratroID)  = val_cp_p1(iteratroID)  - realVal_p1(iteratroID) ;ee(iteratroID)  = ee(iteratroID)  + e2A(iteratroID) .*e2A(iteratroID) ;
                 e3A(iteratroID)  = val_cp(iteratroID)  - realVal(iteratroID) ;      ee(iteratroID)  = ee(iteratroID)  + e3A(iteratroID) .*e3A(iteratroID) ;
                 e4A(iteratroID)  = val_cp_m1(iteratroID)  - realVal_m1(iteratroID) ;ee(iteratroID)  = ee(iteratroID)  + e4A(iteratroID) .*e4A(iteratroID) ;
                 e5A(iteratroID)  = val_cp_m2(iteratroID)  - realVal_m2(iteratroID) ;ee(iteratroID)  = ee(iteratroID)  + e5A(iteratroID) .*e5A(iteratroID) ;
                 
             else
                e1B(iteratroID)  = val_cp_p2(iteratroID)  - realVal_p2(iteratroID) ;ee(iteratroID)  = ee(iteratroID)  + e1B(iteratroID) .*e1B(iteratroID) ;
                e2B(iteratroID)  = val_cp_p1(iteratroID)  - realVal_p1(iteratroID) ;ee(iteratroID)  = ee(iteratroID)  + e2B(iteratroID) .*e2B(iteratroID) ;
                e3B(iteratroID)  = val_cp(iteratroID)  - realVal(iteratroID) ;      ee(iteratroID)  = ee(iteratroID)  + e3B(iteratroID) .*e3B(iteratroID) ;
                e4B(iteratroID)  = val_cp_m1(iteratroID)  - realVal_m1(iteratroID) ;ee(iteratroID)  = ee(iteratroID)  + e4B(iteratroID) .*e4B(iteratroID) ;
                e5B(iteratroID)  = val_cp_m2(iteratroID)  - realVal_m2(iteratroID) ;ee(iteratroID)  = ee(iteratroID)  + e5B(iteratroID) .*e5B(iteratroID) ;
             end
             
             
            %% do I have a new winner??
            %%if so: set.
             bestMacthErrIds = ee(iteratroID) < best_match_err(iteratroID);
		if any(bestMacthErrIds)
			%%put to second-best
			second_best_match_err(bestMacthErrIds)=best_match_err(bestMacthErrIds);
			loopCSecond(bestMacthErrIds)= loopCBest(bestMacthErrIds);

			%% set best.
			best_match_err(bestMacthErrIds) = ee(bestMacthErrIds);
			loopCBest(bestMacthErrIds) = loopCounter(bestMacthErrIds);

			best_match_errPre = eeLast(bestMacthErrIds);
			best_match_DiffErrPre = e1A(bestMacthErrIds).*e1B(bestMacthErrIds) + e2A(bestMacthErrIds).*e2B(bestMacthErrIds) + e3A(bestMacthErrIds).*e3B(bestMacthErrIds) + e4A(bestMacthErrIds).*e4B(bestMacthErrIds) + e5A(bestMacthErrIds).*e5B(bestMacthErrIds);
			best_match_errPost(bestMacthErrIds) = -1;
			best_match_DiffErrPost(bestMacthErrIds) = -1;

			best_match_x(bestMacthErrIds) = cpx(bestMacthErrIds);
			best_match_y(bestMacthErrIds) = cpy(bestMacthErrIds);
			bestWasLastLoop(bestMacthErrIds) = true;
	
		%% otherwise: the last might be the current winner, in which case i have to save these values.
            NotbestMacthErrIds = ~bestMacthErrIds;
            if any(NotbestMacthErrIds)
                if any(bestWasLastLoop)
			
				best_match_errPost(NotbestMacthErrIds & bestWasLastLoop) = ee(NotbestMacthErrIds &  bestWasLastLoop);
				best_match_DiffErrPost(NotbestMacthErrIds &  bestWasLastLoop) = e1A(NotbestMacthErrIds &  bestWasLastLoop).*e1B(NotbestMacthErrIds &  bestWasLastLoop) + e2A(NotbestMacthErrIds &  bestWasLastLoop).*e2B(NotbestMacthErrIds &  bestWasLastLoop) + e3A(NotbestMacthErrIds &  bestWasLastLoop).*e3B(NotbestMacthErrIds &  bestWasLastLoop) + e4A(NotbestMacthErrIds &  bestWasLastLoop).*e4B(NotbestMacthErrIds &  bestWasLastLoop) + e5A(NotbestMacthErrIds &  bestWasLastLoop).*e5B(NotbestMacthErrIds &  bestWasLastLoop);
				bestWasLastLoop(NotbestMacthErrIds &  bestWasLastLoop) = false;
                end

			%% collect second-best:
			%% just take the best of all that are NOT equal to current best.
                secondBestMatchIds = ee(NotbestMacthErrIds)  < second_best_match_err;
			
				second_best_match_err(NotbestMacthErrIds & secondBestMatchIds)=ee(secondBestMatchIds);
				loopCSecond(NotbestMacthErrIds & secondBestMatchIds) = loopCounter(secondBestMatchIds);
		
            end
        end

		%% shift everything one further.
		eeLast(iteratroID)  = ee(iteratroID) ;
		val_cp_m2(iteratroID)  = val_cp_m1(iteratroID) ;
        val_cp_m1(iteratroID)  = val_cp(iteratroID) ; 
        val_cp(iteratroID)  = val_cp_p1(iteratroID) ; 
        val_cp_p1(iteratroID)  = val_cp_p2(iteratroID) ;
        cpx(iteratroID)  = cpx(iteratroID)  + incx(iteratroID) ;
		cpy(iteratroID)  = cpx(iteratroID)  + incy(iteratroID) ;
        loopCounter(iteratroID)  = loopCounter(iteratroID)  + 1;
        
      
        
        iteratroID = (incx < 0) == (cpx > pClose(:,1)) & (incy < 0) == (cpy > pClose(:,2)); 
        loopCounterAll = loopCounterAll + 1;
    end
            %    val_cp_p2;
            
     validIDs =  ~(best_match_err > 4.0 * globalParams.MAX_ERROR_STEREO);
	 pFar = pFar(validIDs , :);
     pClose = pClose(validIDs , :);
     loopCBest = loopCBest(validIDs);    
     loopCSecond = loopCSecond(validIDs);
     rescaleFactor = rescaleFactor(validIDs);
        second_best_match_err =  second_best_match_err(validIDs);
        KinvP = KinvP(validIDs,:);
        
         u = u(validIDs);
                v = v(validIDs);
        
	 %%check if clear enough winner
	validIDs =  ~(abs(loopCBest - loopCSecond) > 1.0 & globalParams.MIN_DISTANCE_ERROR_STEREO .* best_match_err > second_best_match_err);
    pFar = pFar(validIDs , :);
     pClose = pClose(validIDs , :);
	 rescaleFactor = rescaleFactor(validIDs);     
      KinvP = KinvP(validIDs,:);
      
      
         u = u(validIDs);
                v = v(validIDs);
     
     %% have to implement if(useSubpixelStereo)
     
     %% sampleDist is the distance in pixel at which the realVal's were sampled
	 sampleDist = globalParams.GRADIENT_SAMPLE_DIST*rescaleFactor;

	gradAlongLine = zeros(size(realVal_p2,1),1);
	tmp = realVal_p2 - realVal_p1;  
    gradAlongLine = gradAlongLine + (tmp.*tmp);
	tmp = realVal_p1 - realVal;  
    gradAlongLine = gradAlongLine + (tmp.*tmp);
	tmp = realVal - realVal_m1;  
    gradAlongLine = gradAlongLine + (tmp.*tmp);
	tmp = realVal_m1 - realVal_m2;  
    gradAlongLine =gradAlongLine + (tmp.*tmp);

	gradAlongLine = gradAlongLine ./ (sampleDist.*sampleDist);

	%% check if interpolated error is OK. use evil hack to allow more error if there is a lot of gradient.
	validIDs = ~(best_match_err > globalParams.MAX_ERROR_STEREO + sqrt( gradAlongLine).*20);
	    pFar = pFar(validIDs , :);
     pClose = pClose(validIDs , :);
	 rescaleFactor = rescaleFactor(validIDs); 
     KinvP = KinvP(validIDs,:);
     
         u = u(validIDs);
                v = v(validIDs);

	%% ================= calc depth (in KF) ====================
    %% * KinvP = Kinv * (x,y,1); where x,y are pixel coordinates of point we search for, in the KF.
	%% * best_match_x = x-coordinate of found correspondence in the reference frame.
     %idnew_best_match;	%% depth in the new image
	%alpha;  %% d(idnew_best_match) / d(disparity in pixel) == conputed inverse depth derived by the pixel-disparity.
    
    dot0 = zeros(size(incx,1),1 );
    dot2 = zeros(size(incx,1) ,1);
    idnew_best_match = zeros(size(incx,1),1);
    alpha = zeros(size(incx,1) ,1);
	XGrtY = (incx.*incx>incy.*incy);
    YGrtX = ~XGrtY;
	ids = find(XGrtY);
		%oldX = obj.fxi*best_match_x(XGrtY)+obj.cxi;
		%nominator = (oldX*referenceFrame.otherToThis_t(3) - referenceFrame.otherToThis_t(1));
       
        
        for i = 1: size(dot0,1)
        
         if(XGrtY(i))
        oldX = obj.fxi*best_match_x(i,:)+obj.cxi;
		nominator = (oldX*referenceFrame.otherToThis_t(3) - referenceFrame.otherToThis_t(1));
		dot0(i,:) = KinvP(i,:) * (referenceFrame.otherToThis_R(1,:)');
		dot2(i,:) = KinvP(i,:) *(referenceFrame.otherToThis_R(3,:)');
        idnew_best_match(i) = (dot0(i,:) - oldX*dot2(i,:)) / nominator;
        
        alpha (i,:)= incx(i,:).*obj.fxi.*(dot0(i,:)*referenceFrame.otherToThis_t(3) - dot2(i,:)*referenceFrame.otherToThis_t(1)) ./ (nominator*nominator);

        
         else
         
        oldY = obj.fyi*best_match_y(i,:)+obj.cyi;

		nominator = (oldY*referenceFrame.otherToThis_t(3) - referenceFrame.otherToThis_t(2));
		 dot1(i,:) = KinvP(i,:) * (referenceFrame.otherToThis_R(2,:)');
		dot2(i,:) = KinvP(i,:) * (referenceFrame.otherToThis_R(3,:)');

		idnew_best_match(i) = (dot1(i,:)  - oldY*dot2(i,:) ) / nominator;
		alpha = incy(i,:)*obj.fyi*(dot1(i,:)*referenceFrame.otherToThis_t(3)- dot2(i,:)*referenceFrame.otherToThis_t(2,:)) / (nominator*nominator);

       
		
        end
        end
	
	
  
	bestMatchIDs = ~(idnew_best_match < 0);
    epxn = epxn(bestMatchIDs);
    epyn = epyn(bestMatchIDs);
	gradAlongLine = gradAlongLine(bestMatchIDs);

	%% ================= calc var (in NEW image) ====================
%{
    photoDispError = 4.0 *  globalParams.cameraPixelNoise2 / (gradAlongLine + globalParams.DIVISION_EPS);

	trackingErrorFac = 0.25*(1.0f+referenceFrame.initialTrackedResidual);

	%% calculate error from geometric noise (wrong camera pose / calibration)
	Eigen::Vector2f gradsInterp = getInterpolatedElement42(activeKeyFrame->gradients(0), u, v, width);
	float geoDispError = (gradsInterp[0]*epxn + gradsInterp[1]*epyn) + DIVISION_EPS;
	geoDispError = trackingErrorFac*trackingErrorFac*(gradsInterp[0]*gradsInterp[0] + gradsInterp[1]*gradsInterp[1]) / (geoDispError*geoDispError);


	%%geoDispError *= (0.5 + 0.5 *result_idepth) * (0.5 + 0.5 *result_idepth);

	%% final error consists of a small constant part (discretization error),
	%% geometric and photometric error.
	result_var = alpha*alpha*((didSubpixel ? 0.05f : 0.5f)*sampleDist*sampleDist +  geoDispError + photoDispError);	// square to make variance

    
	%% calculate error from photometric noise

	%}
         
end
         function observeDepthRow(obj, yMin , yMax)
             %%loads of code goes here
             [x , y] = meshgrid(yMin:yMax,3 :obj.width - 3 );
             obj.observeDepthCreate(x,y);
            
         end
        
         function observeDepthCreate(obj , x, y)
             obj.makeAndCheckEPLMat( x , y , obj.newest_referenceFrame);
         end
         
         function observeDepthUpdate(obj)
             %called if depth hypothisis is aviable 
         end
    end
end