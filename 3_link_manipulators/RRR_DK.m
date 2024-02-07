function [T04] = RRR_DK( q )
    T04 = RRR_trasf_i_1_i( 1, q(1) ) * RRR_trasf_i_1_i( 2, q(2) ) * RRR_trasf_i_1_i( 3, q(3) ) * RRR_trasf_i_1_i( 4, 0 );
end
