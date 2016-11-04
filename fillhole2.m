function filled = fillhole2(M, radius)
    [H, W, ~] = size(M);
    left_undo = M(1:round(H/3),:,:);
    handle = M(round(H/3)+1:H,:,:);
    filled = handle;
    [new_H, new_W,~] = size(handle);
    x_mask = handle(:,:,1);
    y_mask = handle(:,:,2);
    z_mask = handle(:,:,3);
    
    votewidths = linspace(1, radius, radius);
    votewidths = [-votewidths, 0, votewidths];
    [vote_v, vote_u] = meshgrid(votewidths, votewidths);
    
    [u, v] = meshgrid(1:new_W, 1:new_H);
     in_all = find(x_mask == 0 & y_mask == 0);
     valid_count = 5;
     for k = in_all(:)'
        vote_u2 = u(k) + vote_u;
        vote_v2 = v(k) + vote_v;
        valid = vote_u2 > 0 & vote_v2 > 0 & vote_u2 < new_W & vote_v2 < new_H;
        vote_u2 = vote_u2(valid);
        vote_v2 = vote_v2(valid);
        vote_ind = vote_v2 + (vote_u2 - 1) * new_H;
        
        vote_x_block = x_mask(vote_ind);
        vote_y_block = y_mask(vote_ind);
        vote_z_block = z_mask(vote_ind);
        n = sum(vote_x_block ~= 0);
        if n < valid_count
            continue;
        end
        x_value = sum(vote_x_block);
        y_value = sum(vote_y_block);
        z_value = sum(vote_z_block);
        filled(v(k), u(k), 1) = x_value / n;
        filled(v(k), u(k), 2) = y_value / n;
        filled(v(k), u(k), 3) = z_value / n;
     end
     filled = [left_undo; filled];
end