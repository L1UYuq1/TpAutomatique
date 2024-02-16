#define SEUIL_D0Q 1e-12
#include "tustin.hpp"
double oneStepFilter(Filter &filter, double input) {
    if (!filter.ok) {
        return 0;
    }
    if (filter.order == 0) {
        filter.en = input;
        filter.sn = filter.n0q*input;
    } else if (filter.order == 1) {
        filter.sn_1 = filter.sn;
        filter.en_1 = filter.en;
        filter.en = input;
        filter.sn = filter.n0q * filter.en + filter.n1q * filter.en_1 - filter.d1q * filter.sn_1;
    } else if (filter.order == 2) {
        filter.sn_2 = filter.sn_1;
        filter.en_2 = filter.en_1;
        filter.sn_1 = filter.sn;
        filter.en_1 = filter.en;
        filter.en = input;
        filter.sn = filter.n0q * filter.en + filter.n1q * filter.en_1 + filter.n2q * filter.en_2 - filter.d1q * filter.sn_1 - filter.d2q * filter.sn_2;
    }
    return filter.sn;

}

bool init_tustin_order0(Filter &filter, double n0p, double d0p, double Te) {

    double n0q, d0q;
    n0q = n0p;
    d0q = d0p;
    filter.ok = (d0q<-SEUIL_D0Q) || (d0q > SEUIL_D0Q);
    filter.Te = Te;
    if (filter.ok) {
        filter.order = 0;
        filter.n0q = n0q / d0q;
        filter.d0q = d0q / d0q;
    }
    filter.sn = 0;
    filter.en = 0;
    return filter.ok;
}

bool init_tustin_order1(Filter &filter, double n0p, double n1p, double d0p, double d1p, double Te) {

    double n0q, n1q, d0q, d1q;
    double t2 = Te*d0p;
    double t3 = Te*n0p;
    double t4 = d1p * 2.0;
    double t5 = n1p * 2.0;
    n0q = t3 + t5;
    n1q = t3 - t5;
    d0q = t2 + t4;
    d1q = t2 - t4;
    filter.ok = (d0q<-SEUIL_D0Q) || (d0q > SEUIL_D0Q);
    filter.Te = Te;
    if (filter.ok) {
        filter.order = 1;
        filter.n1q = n1q / d0q;
        filter.n0q = n0q / d0q;
        filter.d1q = d1q / d0q;
        filter.d0q = d0q / d0q;
    }
    filter.sn_2 = 0;
    filter.sn_1 = 0;
    filter.sn = 0;
    filter.en_2 = 0;
    filter.en_1 = 0;
    filter.en = 0;

    return filter.ok;
}

bool init_tustin_order2(Filter &filter, double n0p, double n1p, double n2p, double d0p, double d1p, double d2p, double Te) {
    double n0q, n1q, n2q, d0q, d1q, d2q;
    double t2 = Te*Te;
    double t3 = d2p * 4.0;
    double t4 = n2p * 4.0;
    double t5 = Te * d1p * 2.0;
    double t6 = Te * n1p * 2.0;
    double t7 = d0p*t2;
    double t8 = n0p*t2;
    n0q = t4 + t6 + t8;
    n1q = n2p * (-8.0) + t8 * 2.0;
    n2q = t4 - t6 + t8;
    d0q = t3 + t5 + t7;
    d1q = d2p * (-8.0) + t7 * 2.0;
    d2q = t3 - t5 + t7;
    filter.ok= (d0q<-SEUIL_D0Q)||(d0q>SEUIL_D0Q);
    filter.Te = Te;
    if (filter.ok) {
        filter.order = 2;
        filter.n2q = n2q / d0q;
        filter.n1q = n1q / d0q;
        filter.n0q = n0q / d0q;
        filter.d2q = d2q / d0q;
        filter.d1q = d1q / d0q;
        filter.d0q = d0q / d0q;

    }
    filter.sn_1 = 0;
    filter.sn = 0;
    filter.en_1 = 0;
    filter.en = 0;

    return filter.ok;
}
bool initFilter(Filter &filter,double n0p,double Te){
    // tustin Approx of F(p) = n0p/d0p;
    return init_tustin_order0(filter, n0p, 1,Te);
}
bool initFilter(Filter &filter,double n0p,double n1p,double d0p,double d1p,double Te){
    // tustin Approx of F(p) = (n0p+n1p*p)/(d0p+d1p*p);
    return init_tustin_order1(filter, n0p, n1p, d0p, d1p, Te);
} 
bool initFilter(Filter &filter,double n0p,double n1p,double n2p,double d0p,double d1p,double d2p,double Te) {
    return init_tustin_order2(filter, n0p, n1p, n2p, d0p, d1p, d2p, Te);
}

