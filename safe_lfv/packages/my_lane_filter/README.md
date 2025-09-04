
Modified the baseline lane_filter package to compensate for variability in the fps on different devices.
Original gaussian blur on histogram works well for 30 fps, but if we have much higher/lower fps then we 
need different to blur the histogram less/more respectively. We scale the gaussian blur covariance based on fps.
