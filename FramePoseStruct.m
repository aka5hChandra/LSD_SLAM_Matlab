classdef FramePoseStruct < handle
    properties
        Frame
        camToWorld
        camToWorld_new
        trackingParent %parent frame
        thisToParent_raw
    end
    methods
        function obj = FramePoseStruct(frame)
            Frame = frame;
            camToWorld = eye(4);
            camToWorld_new = eye(4);
        end
         
        function camToWorld = getCamToWorld(obj, recursionDepth)
            assert(recursionDepth < 5000)
            
            if size(obj.trackingParent,1) == 0
                obj.camToWorld = eye(4);
            else
                obj.camToWorld = obj.trackingParent.getCamToWorld(recursionDepth + 1) * obj.thisToParent_raw;
            end
            camToWorld =   obj.camToWorld ;
        end
    end
end