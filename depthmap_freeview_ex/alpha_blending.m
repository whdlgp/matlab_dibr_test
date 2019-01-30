function blended_image = alpha_blending(im_l, im_r, occ_l,occ_r, alpha)
    [height, width, chan] = size(im_l);
    blended_image = uint8(zeros(height, width, chan));
    
    for w = 1 : width
        for h = 1 : height
            if (occ_l(h, w) == 0) && (occ_r(h, w) == 0)
                blended_image(h, w, :) = (1 - alpha)*im_l(h, w, :) + alpha*im_r(h, w, :);                      
            elseif (occ_l(h, w) == 0) && (occ_r(h, w) == 1)
                blended_image(h, w, :) = im_l(h, w, :);
            elseif (occ_l(h, w) == 1) && (occ_r(h, w) == 0)
                blended_image(h, w, :) = im_r(h, w, :);
            end
        end
    end
end

