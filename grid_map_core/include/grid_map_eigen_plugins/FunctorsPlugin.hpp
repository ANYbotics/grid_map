#include <math.h>

template<typename Scalar> struct scalar_sum_op_of_finites {
  EIGEN_EMPTY_STRUCT_CTOR(scalar_sum_op_of_finites)
  EIGEN_STRONG_INLINE const Scalar operator() (const Scalar& a, const Scalar& b) const {
    using std::isfinite;
    if (isfinite(a) && isfinite(b)) return a + b;
    if (isfinite(a)) return a;
    if (isfinite(b)) return b;
    return a + b;
  }
};
template<typename Scalar>
struct functor_traits<scalar_sum_op_of_finites<Scalar> > {
  enum {
    Cost = 2 * NumTraits<Scalar>::ReadCost + NumTraits<Scalar>::AddCost,
    PacketAccess = false
  };
};
