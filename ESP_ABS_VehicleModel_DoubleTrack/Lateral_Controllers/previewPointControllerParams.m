function previewPointParams = previewPointControllerParams()

    % ----------------------------------------------------------------
    %% Function purpose: define the preview point lateral controller parameters
    % ----------------------------------------------------------------
    
    % ke error to position
    previewPointParams.ke = 2; 
    % kp error to angle
    previewPointParams.kp = 2; 
    % lookhead
    previewPointParams.lookAhead = 10;

end

