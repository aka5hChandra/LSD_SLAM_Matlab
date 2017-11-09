classdef DepthMapPixelHypothesis < handle
    properties
        isValid
        blackListed
        idepth
        idepth_var
        
        idepth_smoothed
        idepth_var_smoothed
    end
    methods
        function obj =  DepthMapPixelHypothesis(my_idepth, my_idepth_smoothed, my_idepth_var, my_idepth_var_smoothed)
            obj.isValid = ones(size(my_idepth));
            obj.blackListed = zeros(size(my_idepth));
            
            obj.idepth = my_idepth;
            obj.idepth_var = my_idepth_var;
            
            obj.idepth_smoothed = my_idepth_smoothed;
            obj.idepth_var_smoothed = my_idepth_var_smoothed;
            
            inValidIds = find((isnan(my_idepth) | my_idepth < 0));
            
            obj.idepth(inValidIds) = 0;  
            obj.idepth_smoothed(inValidIds) = 0;
          
            obj.blackListed(inValidIds) = 1;
            obj.idepth(inValidIds) = 0;
           
            
            
        end
        
        function [visualized] = visualize(obj,debugDisplay)
            visualized = zeros([size(obj.idepth),3]);
            if(debugDisplay == 0 || debugDisplay == 1)
                
                depth = [];
                if(debugDisplay == 0)
                    depth= obj.idepth_smoothed;
                elseif(debugDisplay == 1)
                    depth= obj.idepth;
                end
      
              %% rainbow between 0 and 4
                r = (0-depth) * 255 / 1.0; 
                id = r < 0;
                r(id) = -1 * r(id);
                g = (1-depth) * 255 / 1.0; 
                id = g < 0;
                g(id) = -1 * g(id);
                b = (2-depth) * 255 / 1.0; 
                id = b < 0;
                b(id) = -1 * b(id);
          
                id = r < 0;
                r(id) = 0;
                id = r > 255;
                r(id) = 255;
            
            
                id = g < 0;
                g(id) = 0;
                id = g > 255;
                g(id) = 255;
            
            
                id = b < 0;
                b(id) = 0;
                id = b > 255;
                b(id) = 255;
		
                visualized(:,:,1) = 255 - r;
                visualized(:,:,2) = 255 - g;
                visualized(:,:,3) = 255 - b;
            
                visualized((depth < 0),1) = 255;
                visualized((depth < 0),2) = 255;
                visualized((depth < 0),3) = 255;
            end
        end
%{
	 %% plot validity counter
	if(debugDisplay == 2)
	
		float f = validity_counter * (255.0 / (VALIDITY_COUNTER_MAX_VARIABLE+VALIDITY_COUNTER_MAX));
		uchar v = f < 0 ? 0 : (f > 255 ? 255 : f);
		return cv::Vec3b(0,v,v);
    end

	%%plot var
	if(debugDisplay == 3 || debugDisplay == 4)
	
		float idv;
		if(debugDisplay == 3)
			idv= idepth_var_smoothed;
		else
			idv= idepth_var;

		float var = - 0.5 * log10(idv);

		var = var*255*0.333;
		if(var > 255) var = 255;
		if(var < 0)
			return cv::Vec3b(0,0, 255);

		return cv::Vec3b(255-var,var, 0);// bw
        end

	%%plot skip
	if(debugDisplay == 5)
	
		float f = (nextStereoFrameMinID - lastFrameID) * (255.0 / 100);
		uchar v = f < 0 ? 0 : (f > 255 ? 255 : f);
		return cv::Vec3b(v,0,v);
    end

	return cv::Vec3b(255,255,255);



        end
        
        %}
    end
end