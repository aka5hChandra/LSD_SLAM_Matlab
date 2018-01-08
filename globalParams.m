classdef globalParams 
    properties (Constant)
        SE3TRACKING_MIN_LEVEL = 1
        SE3TRACKING_MAX_LEVEL = 5

        pyramidLevel = globalParams.SE3TRACKING_MAX_LEVEL

        MIN_EPL_LENGTH_SQUARED = 1.0 * 1.0
        MIN_EPL_GRAD_SQUARED = 2.0 * 2.0
        MIN_EPL_ANGLE_SQUARED = 0.3 * 0.3
        GRADIENT_SAMPLE_DIST = 1.0
        DIVISION_EPS =  1e-10
        cameraPixelNoise2 = 4*4
        
        MAX_VAR = 0.5 * 0.5 %% initial variance on creation - if variance becomes larter than this, hypothesis is removed.

        VAR_GT_INIT_INITIAL = 0.01 * 0.01	%%initial variance vor Ground Truth Initialization
        VAR_RANDOM_INIT_INITIAL = 0.5 * globalParams.MAX_VAR	
        
        STEREO_EPL_VAR_FAC = 2.0
        MIN_DEPTH = 0.05
        
        MAX_EPL_LENGTH_CROP  = 30.0 %%maximum length of epl to search.
        MIN_EPL_LENGTH_CROP =  3.0 %%minimum length of epl to search.
        
        SAMPLE_POINT_TO_BORDER = 7
        MAX_ERROR_STEREO =  1300.0%4000.0%
        MIN_DISTANCE_ERROR_STEREO =  1.5%6%
        SUCC_VAR_INC_FAC = 1.01
        
        
        
        MAX_DIFF_CONSTANT = 40.0 * 40.0
        MAX_DIFF_GRAD_MULT = 0.5 * 0.5 
        
         MIN_ABS_GRAD_CREATE = 5
         MIN_ABS_GRAD_DECREASE = 5
         
         VALIDITY_COUNTER_MAX =  5.0		% validity will never be higher than this
         VALIDITY_COUNTER_MAX_VARIABLE =  250.0
         
         % define how strict the merge-processes etc. are.
         % are multiplied onto the difference, so the larger, the more restrictive.
         DIFF_FAC_SMOOTHING = 1.0 *1.0
         DIFF_FAC_OBSERVE = 1.0 *1.0
         DIFF_FAC_PROP_MERGE = 1.0 *1.0
         DIFF_FAC_INCONSISTENT = 1.0  * 1.0

    end
end