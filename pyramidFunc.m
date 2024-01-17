function z = pyramidFunc(x, y, base_size, pyramid_height)
    z = zeros(size(x));
    for i = 1:size(x, 1)
        for j = 1:size(x, 2)
            if abs(x(i, j)) <= base_size/2 && abs(y(i, j)) <= base_size/2
                z(i, j) = max(0, 1 - (abs(x(i, j)) / (base_size/2)) - (abs(y(i, j)) / (base_size/2))) * (pyramid_height / 2);
            else
                z(i, j) = 0;
            end
        end
    end
end