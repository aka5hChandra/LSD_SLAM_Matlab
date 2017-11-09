classdef Frame < RGBDImage
    properties
        imgGray
        id
        pose
        
        KK ;
        Kinv;
		fx, fy, cx, cy;
		fxInv, fyInv, cxInv, cyInv;
		
		image,  imageValid;
		
		gradients, gradientsValid;
		
		maxGradients, maxGradientsValid;
	    
        hasIDepthBeenSet;

	    idepth, idepthValid;
		
        idepthVar, idepthVarValid;
        
        K_otherToThis_R,otherToThis_t,K_otherToThis_t;
        thisToOther_t,K_thisToOther_t,thisToOther_R,distSquared,referenceID,referenceLevel;
        otherToThis_R
        %%to be remvoed
        tempK;
 
    end
    methods
        function obj = Frame(id, imgZ, imgRGB, cam_info_K)
            obj = obj@RGBDImage(imgZ, imgRGB, cam_info_K);
            obj.imgGray = rgb2gray(imgRGB);
            obj.id = id;
            obj.pose = FramePoseStruct(obj);
            
            obj.KK = cell(globalParams.pyramidLevel,1);
            obj.Kinv = cell(globalParams.pyramidLevel,1);
            obj.fx = cell(globalParams.pyramidLevel,1); 
            obj.fy = cell(globalParams.pyramidLevel,1); 
            obj.cx= cell(globalParams.pyramidLevel,1);
            obj.cy = cell(globalParams.pyramidLevel,1);
            obj.fxInv = cell(globalParams.pyramidLevel,1); 
            obj.fyInv = cell(globalParams.pyramidLevel,1); 
            obj.cxInv = cell(globalParams.pyramidLevel,1); 
            obj.cyInv = cell(globalParams.pyramidLevel,1);
            obj.image = cell(globalParams.pyramidLevel,1);  
            obj.imageValid = cell(globalParams.pyramidLevel,1);
            obj.gradients = cell(globalParams.pyramidLevel,1); 
            obj.gradientsValid = cell(globalParams.pyramidLevel,1);
            obj.maxGradients = cell(globalParams.pyramidLevel,1); 
            obj.maxGradientsValid = cell(globalParams.pyramidLevel,1);
            obj.hasIDepthBeenSet = cell(globalParams.pyramidLevel,1);
            obj.idepth = cell(globalParams.pyramidLevel,1); 
            obj.idepthValid = cell(globalParams.pyramidLevel,1);
            obj.idepthVar = cell(globalParams.pyramidLevel,1); 
            obj.idepthVarValid = cell(globalParams.pyramidLevel,1);
            
            obj.tempK = cam_info_K;
        end
        
        function calculateMeanInformation(obj)
             nonZeroIDs = (obj.idepthVar > 0) ;
             total = sum(sum(sqrt(1./a)));
             obj.meanInformation = total ./ sum(sum(nonZeroIDs));
             
        end
        function setDepthFromGroundTruth(obj , depth , cov_scale)
        end
        
        %%Sim3 to keyframe
        function prepareForStereoWith(obj, other , thisToOther, K, level)
            otherToThis = inv(thisToOther);
            
           
            obj.K_otherToThis_R = K * otherToThis(1:3,1:3) ;%%* otherToThis.scale();
            obj.otherToThis_t =  otherToThis(1:3,4);
            obj.K_otherToThis_t = K * obj.otherToThis_t;

        	obj.thisToOther_t = thisToOther(1:3,4);
            obj.K_thisToOther_t = K * obj.thisToOther_t;
            obj.thisToOther_R = thisToOther(1:3,1:3);%% thisToOther.scale();
            
            obj.otherToThis_R = obj.thisToOther_R';
            %{
            obj.otherToThis_R_row0 = thisToOther_R.col(0);
            obj.otherToThis_R_row1 = thisToOther_R.col(1);
            obj.otherToThis_R_row2 = thisToOther_R.col(2);
            %}
            obj.distSquared = dot(obj.otherToThis_t , obj.otherToThis_t);

            obj.referenceID = other.id;
            obj.referenceLevel = level;
        end
        function camToWorld = getScaledCamToWorld(obj)
            camToWorld = obj.pose.getCamToWorld(1);
        end
    end
end

