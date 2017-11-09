classdef SE3Tracker < handle
    properties 
        width;
        height;
        K;
        kInv;
    end
    methods
        function obj = SE3Tracker(w , h , k)
            width = w;
            height = h;
            K = k;
            kInv = inv(k);
        end
        function calcResidualAndBuffers(refPoint , refColVar, frame,referanceToframe,level)
            w = frame.widht{level};
            h = frame.height{level};
            KLvl = frame.K{level};
            fx_l = Klvl(1,1);
            fy_l = KLvl(2,2);
            cx_l = KLvl(1,3);
            cy_l = KLvl(2,3);
            
            rotMat = referanceToframe(1:3,1:3);
            trasVec = referanceToframe(: , 4);
        end
        function trackFrame(reference , frame , frameToReference_initialEstimate )
            referenceToFrame = inv(frameToReference_initialEstimate);
            
            for lvl = globalParams.SE3TRACKING_MIN_LEVEL : globalParams.SE3TRACKING_MAX_LEVEL
                
            end
        end
    end
end