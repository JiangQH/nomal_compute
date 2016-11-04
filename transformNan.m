function N = transformNan(M)
    N = M;
    [n1, n2] = size(M); %[375, 1242]
    for i = 1 : n2
        for j = 1 : n1
            if M(j, i) == 0
                if ((j+10) < n1) && ((j-10) > 0)
                    num1 = nnz(M(j:j+10, i));
                    num2 = nnz(M(j-10:j, i));
                    if (num1 ~= 0) && (num2 ~= 0)
                        N(j, i) = NaN;
                    end
                end
            end
        end
    end
end