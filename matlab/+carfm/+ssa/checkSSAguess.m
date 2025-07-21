function checkSSAguess(guess,u_long0)
%checkSSAguess

% Do not check if guess is empty (i.e. not provided)
if isempty(fieldnames(guess))
    return
end

if numel(guess)==1
    guess = repmat(guess, [1 size(u_long0,2)]);
end

if ~isequal(size(guess), [1 size(u_long0,2)])
    eid = 'carfm:notEqual';
    msg = 'Wrong size of guess input.';
    error(eid,msg)
end