function mustBeEqualSize(a,b)
    % MUSTBEEQUALSIZE Test for equal size.
    if ~isequal(size(a),size(b))
        eid = 'carfm:notEqual';
        msg = 'Size of inputs mismatch.';
        error(eid,msg)
    end
end