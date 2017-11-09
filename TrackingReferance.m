classdef TrackingReferance < handle
  %{
 * Point cloud used to track frame poses.
 * 
 * Basically this stores a point cloud generated from known frames. It is used to
 * track a new frame by finding a projection of the point cloud which makes it
 * look as much like the new frame as possible.
 * 
 * It is intended to use more than one old frame as source for the point cloud.
 * Also other data like Kinect depth data could be imported.
 * 
 * ATTENTION: as the level zero point cloud is not used for tracking, it is not
 * fully calculated. Only the weights are valid on this level!
 %}
    
    properties 
        posData 
        gradData 
        colarAndVarData 
        keyFrame
        %numData
        frameID
        wh_allocated;
    end
    methods
        function obj = TrackingReferance()
            obj.posData = cell(globalParams.pyramidLevel);
            obj.gradData = cell(globalParams.pyramidLevel);
            obj.colarAndVarData = cell(globalParams.pyramidLevel);
            %obj.numData = cell(globalParams.pyramidLevel);
            obj.keyFrame = [];
            obj.frameID = -1;
            obj.wh_allocated = 0;
        end
        function importFrame(obj,sourceKF)
            obj.keyFrame = sourceKF;
            obj.frameID = obj.keyFrame.id;
            
            if(size(sourceKF,1) * size(sourceKF,2) ~= obj.wh_allocated)
                obj.releaseAll();
                obj.wh_allocated = size(sourceKF,1) * size(sourceKF,2);
            end
            obj.clearAll();
        end
        function makePointCloud(obj,level)
            assert(obj.keyframe ~= 0)
            numDataMat = obj.posData{level};
            if(size(numDataMat,1) >0)
                return
            end
            
            w = obj.keyFrame.width{level};
            h = obj.keyFrame.height{level};
            
            fxInvLevel = obj.keyFrame.fxInv{level};
            fyInvLevel = obj.keyFrame.fyInv{level};
            cxInvLevel = obj.keyFrame.cxInv{level};
            cyInvLevel = obj.keyFrame.cyInv{level};
            
            pyrIdepthSource = obj.keyFrame.idepth{level};
            pyrIdepthVarSource = obj.keyFrame.idepthVar{level};
            pyrColorSource = obj.keyFrame.image{level};
            pyrGradSource = obj.keyFrame.gradients{level}; %%vec4
            %{
            posDataPt =obj.posData{level};
            gradDataPt = obj.gradData{level};
            %%%idxPT = 
            colorAndVarDataPt = obj.colarAndVarData{level};
            %}
            %{
            for x = 1 : w -1
                for y = 1 : h -1
                   idx =  sub2ind(size(frame.image{level}) , x, y);
                    
                    if(pyrIdepthSource(idx) <= 0 ||pyrIdepthVarSource(idx) == 0)
                        continue
                    end
                    
                    obj.posData{level} = (1.0 / pyrIdepthSource(idx)) * [fxInvLevel*x+cxInvLevel,fyInvLevel*y+cyInvLevel,1];
                    obj.gradData{level} = pyrGradSource(idx).head<2>();
                    obj.colarAndVarData{level} = [pyrColorSource(idx), pyrIdepthVarSource(idx)];
			
                    
                  
                   
                end
            end
            %}
            
            %%could get rid of one D array and can do all in 2 D array
             nonZeroIDs = ~(pyrIdepthSource <= 0 |pyrIdepthVarSource == 0);
             [x,y] = ind2sub(size(nonZeroIDs),find(nonZeroIDs));
             %vec3
             obj.posData{level} =reshape( (1.0 / pyrIdepthSource(nonZeroIDs)) * [fxInvLevel*x+cxInvLevel,fyInvLevel*y+cyInvLevel,1],3,[])';
             %vec4
             temp= reshape(pyrGradSource(nonZeroIDs),4,[])';
             %vec2
             obj.gradData{level} = temp(:,1:2);%,:2);
             %vec2
             obj.colarAndVarData{level} = [pyrColorSource(nonZeroIDs), pyrIdepthVarSource(nonZeroIDs)];
			
            
        end
        function releaseAll(obj)
            obj.posData = cell(globalParams.pyramidLevel);
            obj.gradData = cell(globalParams.pyramidLevel);
            obj.colarAndVarData = cell(globalParams.pyramidLevel);
            %obj.numData = cell(globalParams.pyramidLevel);
            obj.wh_allocated = 0;
        end
        function invalidate(obj)
            obj.keyFrame = 0;
        end
        
        function clearAll(obj)
             %obj.numData = cell(globalParams.pyramidLevel);
        end
    end
end