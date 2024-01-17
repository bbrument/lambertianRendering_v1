function normals = pyramidNormalsFunc(x, y, base_size, pyramid_height)
    % Ensure X and Y are column vectors
    x = x(:);
    y = y(:);

    % Points
    pointRight = [base_size/2; 0; 0];
    pointTop = [0; base_size/2; 0];
    pointLeft = [-base_size/2; 0; 0];
    pointBottom = [0; -base_size/2; 0];
    pointApex = [0; 0; pyramid_height/2];

    % Calculate normals
    normals = zeros(3,length(x));
    for ii = 1:length(x)
        x_ii = x(ii);
        y_ii = y(ii);
        z_ii = pyramidFunc(x_ii, y_ii, base_size, pyramid_height);

        if z_ii > 0
            if x_ii <= base_size/2 && x_ii >= 0
                % Right side
                if y_ii <= base_size/2 && y_ii >= 0
                    % Top-right part
                    normal = cross(pointApex-pointRight, pointApex-pointTop);
                elseif y_ii >= -base_size/2 && y_ii <= 0
                    % Bottom-right part
                    normal = cross(pointApex-pointRight, pointApex-pointBottom);
                end
            elseif x_ii >= -base_size/2 && x_ii <= 0
                % Left side
                if y_ii <= base_size/2 && y_ii >= 0
                    % Top-left part
                    normal = cross(pointApex-pointLeft, pointApex-pointTop);
                elseif y_ii >= -base_size/2 && y_ii <= 0
                    % Bottom-left part
                    normal = cross(pointApex-pointLeft, pointApex-pointBottom);
                end
            end
            normal = normal/norm(normal);
        else
            normal = [0;0;1];
        end

        % Check sign
        if normal(3) < 0
            normal = -normal;
        end

        % Display
        normals(:,ii) = normal;
    end
end