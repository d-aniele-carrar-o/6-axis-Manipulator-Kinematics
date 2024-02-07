function [lerp_p] = lerp_path( p_i, p_f, N )
    t = linspace( 0, N, N+1 );

    lerp_p = zeros( [size(p_i),N] );

    for i=1:N
        lerp_p(:,i) = lerp( t(i), p_i, p_f, N );
    end
end
