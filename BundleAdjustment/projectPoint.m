# projects a point
function p_img = projectPoint(Xr, Xl, img_width, img_height, K)
    iXr = inv(Xr);
    p_img = [-1;-1];
    pw = iXr(1:3,1:3)*Xl + iXr(1:3,4);
    if (pw(3) < 0)
        return;
    endif;
    p_cam = K*pw;
    iz = 1./p_cam(3);
    p_cam *= iz;
    if (p_cam(1) < 0 || p_cam(1) > img_width ||
        p_cam(2) < 0 || p_cam(2) > img_height)
        return;
    endif;
    p_img = p_cam(1:2);
endfunction
