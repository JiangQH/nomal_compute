function filled = fillhole(M)
    %fill the hole with 5 radius
    filled = M;
    [m, n,~] = size(M);
    for row = 1 : m
        for col = 1 : n
            sample = M(row,col, :);
            if sample(:) == [0;0;0]
                continue;
            end
            % get the left, right, top, bottom corner
            left = col - 3;
            right = col + 3;
            top = row - 3;
            bottom = row + 3;
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