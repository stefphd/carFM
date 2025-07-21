function mustBeEqualSize1(a,b)
    % MUSTBEEQUALSIZE1 Test for equal size(1).
    if ~isequal(size(a,1),size(b,1))
        eid = 'carfm:notEqual';
        msg = 'Size of inputs mismatch.';
        error(eid,msg)
    end
end