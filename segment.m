function segs = segment(A, bound)
% segment the index and pick out the bad points which have the same
% index
% return the segs and rectified matrix
    rectified = A;
    start = 1;
    count = 1;
    ends = 2;
    while (ends <= size(rectified, 1))
        % judge if encounter duplicate rows
        if (ends ~= start) && rectified(ends, 2) == rectified(ends-1, 2)
            % judge the vaule of them
            diff1 = 0;
            diff2 = 0;
            if ends - 2 > 0
                diff1 = diff1 + abs(rectified(ends-1, 3) - rectified(ends-2, 3));
                diff2 = diff2 + abs(rectified(ends, 3) - rectified(ends-2, 3));
            end
            if ends + 1 < size(rectified, 1)
                diff1 = diff1 + abs(rectified(ends+1, 3) - rectified(ends-1, 3));
                diff2 = diff2 + abs(rectified(ends+1, 3) - rectified(ends, 3));
            end
            diff1 = diff1 / 2;
            diff2 = diff2 / 2;
            delta = abs(rectified(ends, 3) - rectified(ends-1, 3));
            % judge between delta and diff
            if delta < diff1 && delta < diff2
                rectified(ends,:) = (rectified(ends-1,:) + rectified(ends, :)) ./ 2;
            elseif delta > diff1
                rectified(ends, :) = rectified(ends-1, :);
            else
                rectified(ends, :) = rectified(ends, :);
            end
            rectified(ends-1,:) = [];
            ends = ends - 1;
            continue;
        end

        if (ends ~= start) && (rectified(ends, 2) - rectified(ends-1, 2)) > bound
            segs{count} = rectified(start:ends-1, :);
            start = ends;
            count = count + 1;
        end
        ends = ends + 1;
    end
    segs{count} = rectified(start:ends-1,:);
end