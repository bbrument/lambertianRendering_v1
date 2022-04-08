function newX = mapToRange(X,fromRange,toRange)

    newX = (X - fromRange(1)) * (toRange(2) - toRange(1)) ...
        / (fromRange(2) - fromRange(1)) + toRange(1);
