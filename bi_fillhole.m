function filled = bi_fillhole(M, radius)
    % here M is the matrix to be filled. using biinterp.
    % radius is the r used to search
    % M : row * col * 4 (1st is the attribute, 2:4 is the value)
    % radius: real valued
    [H, W, ~] = size(M);
    filled = M;
    
    blockwidths = linspace(1, radius, radius);
    blockwidths = [-blockwidths, 0, blockwidths];
    [nu, nv] = meshgrid(blockwidths, blockwidths);
    [u, v] = meshgrid(1:W, 1:H);
    % find zero point
    in_all = find(M(:,:,4) == 0); 
    % valid bound
    % note that matlab is stored by columns
    h_bound = 20;
    %w_bound = 10;
    
    for k = in_all(:)'
        % check whether this point should be filled
        % a point should have value 
        h = v(k); 
        w = u(k);
        if h < H/3
            continue;
        end
        h_above = h - h_bound;
        h_below = h + h_bound;
        if h_above <= 0
            h_above = 1;
        end
        if h_below > H;
            h_below = H;
        end
        %w_left = w - w_bound;
        %w_right = w + w_bound;
        %if w_left <= 0
        %    w_left = 1;
        %end
        %if w_right > W
        %    w_right = W;
        %end
        % it means that if there are one side that not zero. it is a valid
        % point
        %lcond = sum(M(h, w_left:w, 4) ~=0 );
        %rcond = sum(M(h, w:w_right, 4) ~= 0);
        acond = sum(M(h_above:h, w, 4) ~= 0);
        bcond = sum(M(h:h_below, w, 4) ~= 0);
%         if ~((lcond ~= 0 && rcond ~= 0) || (acond ~= 0 && bcond ~= 0)) 
%             continue;
%         end
        if acond == 0 || bcond == 0
            continue;
        end
        
        u2 = u(k) + nu;
        v2 = v(k) + nv;
        valid = u2 >0 & v2 > 0 & u2 < W & v2 < H;
        u2 = u2(valid);
        v2 = v2(valid);
        ind2 = v2 + (u2-1)*H;
        % check the flag with majority
        flags = M(:,:,1);
        flag = flags(ind2);
        uniflag = unique(flag);
        max_flag = -1;
        max_num = -1;
        for i = 1 : length(uniflag)
            if uniflag(i) == 0
                continue;
            end
            flagnum = sum(flag == uniflag(i));
            if flagnum > max_num
                max_num = flagnum;
                max_flag = uniflag(i);
            end
        end
        % assign the point with max flag. and 
        filled(h, w, 1) = max_flag;
        X_value = M(:,:,2);
        Y_value = M(:,:,3);
        Z_value = M(:,:,4);
        X_value = X_value(ind2);
        Y_value = Y_value(ind2);
        Z_value = Z_value(ind2);
        index = find(flag == max_flag);
        value1 = sum(X_value(index)) / max_num;
        value2 = sum(Y_value(index)) / max_num;
        value3 = sum(Z_value(index)) / max_num;
        filled(h, w, 2:4) = [value1,value2, value3];
    end
    
end