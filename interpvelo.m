function interpted = interpvelo(velo, bound)
    sorted_bounds = sortrows(velo);
    interpted = [];
    % do interplation for the points
    uniques_idx = unique(sorted_bounds(:,1));
    for i = 1 : length(uniques_idx)
        index = find(sorted_bounds(:,1) == uniques_idx(i));
        selected = sorted_bounds(index, :);
        % again the unique
        uniques_value = unique(selected(:,2));
        for vi = 1 : length(uniques_value)
            index2 = find(selected(:,2) == uniques_value(vi));
            selected_value = selected(index2, :); % now have the value with same idx and same one index
            % pick out those with same index of both two
            % maybe do the segment?
            segments = segment(selected_value, bound);
            for seg = 1 : length(segments)
                matrixs = segments{seg};
                x = matrixs(:, 3);
                xq = linspace(x(1), x(length(x)), x(length(x))-x(1)+1);
                interp_matrix = [];
                if length(x) > 1
                    for cc = 1 : size(matrixs, 2)
                        v = matrixs(:, cc);
                        in = interp1(x, v, xq)';
                        interp_matrix = [interp_matrix, in];
                    end
                else
                    interp_matrix = matrixs;
                end
                interpted = [interpted; interp_matrix];
            end
        end
    end
end
        
        
        
        
        
        
    
