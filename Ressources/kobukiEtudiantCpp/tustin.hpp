#ifndef _TUSTIN_
#define _TUSTIN_
typedef struct  {
   bool ok; 
   double Te;
   int order;
   double n0p, n1p, n2p, d0p, d1p, d2p;
   double n0q,n1q,n2q,d0q,d1q,d2q;
   // memoires
   double sn_2,sn_1,sn,en_2,en_1,en;
} Filter;
double oneStepFilter( Filter &filter,double input); // compute the filter ouput 
// filters of orders 0 ,1, 2 with tustin approx at sample time Te, not optimized at all
bool initFilter(Filter &filter,double n0p,double Te); // tustin Approx of F(p) = n0p, with sample time Te;
bool initFilter(Filter &filter,double n0p,double n1p,double d0p,double d1p,double Te); // tustin Approx of F(p) = (n0p+n1p*p)/(d0p+d1p*p) , ...;
bool initFilter(Filter &filter,double n0p,double n1p,double n2p,double d0p,double d1p,double d2p,double Te) ;// tustin Approx of F(p) = (n0p+n1p*p+n2p*p^2)/(d0p+d1p*p+d2p*p^2);

#endif