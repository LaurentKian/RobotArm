    function sample = Sample_ball()
        sample = zeros(1,2);
        for i = 1:2
            sample(i) = random("Normal",0,1);
        end
        sample = random("Uniform",0,1)*sample/sqrt((sum(sample.^2,2)));
        sample = sample';
    end