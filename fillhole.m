function filled = fillhole(M, ratio)
    %fill the hole with 5 radius
    filled = M;
    [m, n,~] = size(M);
    
    for row = 1 : m
        for col = 1 : n
            sample = M(row,col, :);
            value = sample(1,1,2:4);
            if value(:) == [0;0;0]
                continue;
            end
            % get the left, right, top, bottom corner
            left = col - ratio;
            right = col + ratio;
            top = row - ratio;
            bottom = row + ratio;
            while(left <= 0)
                left = left + 1;
            end
            while(right > n)
                right = right - 1;
            end
            while(top <= 0)
                top = top + 1;
            end
            while(bottom > m)
                bottom = bottom - 1;
            end
            % do the assign work
            for srow = top : bottom
                for scol = left : right
                    % what if it has a value?
                    % right now ignore it
                    filled(srow, scol, :) = sample;
                end
            end
            
        end
    end
    
end