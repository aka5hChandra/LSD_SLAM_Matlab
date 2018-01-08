classdef FramePoseStruct < handle
    properties
        Frame
        camToWorld
        camToWorld_new
        trackingParent %// parent, the frame originally tracked on. never changes.
        thisToParent_raw
    end
    methods
        function obj = FramePoseStruct(frame)
            obj.Frame = frame;
            obj.camToWorld = eye(4);
            obj.camToWorld_new = eye(4);
            obj.thisToParent_raw = eye(4);
            %obj.trackingParent = 0;
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