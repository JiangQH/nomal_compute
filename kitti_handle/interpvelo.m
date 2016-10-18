function interpted = interpvelo(velo, bound)
    sorted_bounds = sortrows(velo);
    interpted = [];
    % do interplation for the points
    uniques = unique(sorted_bounds(:,1));
    for i = 1 : length(uniques)
        index = find(sorted_bounds(:,1) == uniques(i));
        selected = sorted_bounds(index, :);
        segments = segment(selected, bound);
        % do interpolation for every part of segments
        for j = 1 :length(segments)
            matrixs = segments{j};
            x = matrixs(:, 2);
            xq = double(int16(linspace(x(1),x(length(x)), x(length(x))-x(1)+1/2)))';
            interp_matrixs = [];
            if length(x) > 1
                for k = 1 : size(matrixs,2)
                    v = matrixs(:,k);
                    in = interp1(x, v, xq);
                    interp_matrixs = [interp_matrixs, in];
                end
            else
                interp_matrixs = matrixs;
            end
            interpted = [interpted; interp_matrixs];
        end
    end
end