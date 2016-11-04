function filled_label = filllabel(M, radius)
    % assign label to those zero point
    % notice the zero label is invalid. so just leave it alone
    % but should assign with thos valid point
    % how to judge whether the point should be filled?
    
    % M is the matrix to be voted 
    % radius is the vote radius
    % valid_bound is the radius to be judge
    
    [H, W, ~] = size(M);
    left_undo = M(1:round(H/3),:,:);
    
    
    handle = M(round(H/3)+1:H,:,:);
    filled_label = handle;
    [new_H, new_W,~] = size(handle);
    label_mask = handle(:,:,1);
    x_mask = handle(:,:,2);
    y_mask = handle(:,:,3);
    z_mask = handle(:,:,4);
    
    
    votewidths = linspace(1, radius, radius);
    votewidths = [-votewidths, 0, votewidths];
    [vote_v, vote_u] = meshgrid(votewidths, votewidths);
    
    [u, v] = meshgrid(1:new_W, 1:new_H);
    % find all zero points. since the upper is totally invalid. pick it out
    in_all = find(label_mask == 0);
    valid_count = 5;
    for k = in_all(:)'
        % check it is a hole?
        vote_u2 = u(k) + vote_u;
        vote_v2 = v(k) + vote_v;
        valid = vote_u2 > 0 & vote_v2 > 0 & vote_u2 < new_W & vote_v2 < new_H;
        vote_u2 = vote_u2(valid);
        vote_v2 = vote_v2(valid);
        vote_ind = vote_v2 + (vote_u2 - 1) * new_H;
        
        vote_label_block = label_mask(vote_ind);
        vote_x_block = x_mask(vote_ind);
        vote_y_block = y_mask(vote_ind);
        vote_z_block = z_mask(vote_ind);
        
        if sum(vote_label_block ~= 0) < valid_count
            %filled_label(v(k), u(k)) = NaN;
            continue;
        end
       % do the vote job
       vote_block = vote_label_block(vote_label_block ~= 0);
       x_block = vote_x_block(vote_label_block ~= 0);
       y_block = vote_y_block(vote_label_block ~= 0);
       z_block = vote_z_block(vote_label_block ~= 0);
       [label, F] = mode(vote_block(:));
       x_value = sum(x_block(vote_block == label));
       y_value = sum(y_block(vote_block == label));
       z_value = sum(z_block(vote_block == label));
       % do the assign job
       filled_label(v(k), u(k),1) = label;
       filled_label(v(k), u(k), 2) = x_value / F;
       filled_label(v(k), u(k), 3) = y_value / F;
       filled_label(v(k), u(k), 4) = z_value / F;
    end
    filled_label = [left_undo; filled_label];
     
end