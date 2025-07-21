function mustBeEqualSize2(a,b)
    % MUSTBEEQUALSIZE2 Test for equal size(2).
    if ~isequal(size(a,2),size(b,2))
        eid = 'carfm:notEqual';
        msg = 'Size of inputs mismatch.';
        error(eid,msg)
    end
end